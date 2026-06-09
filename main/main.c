/*
 * ============================================================
 *  ASTRA CONTROLLER — Firmware Principal
 *  Matheus Amorim & Matheus Braido
 *  APS — Sistemas Embarcados (Insper)
 *
 *  Hardware:
 *    - Raspberry Pi Pico (RP2040) + FreeRTOS
 *    - MPU-6050 (IMU via I2C)
 *    - Joystick analógico (ADC0 = X, ADC1 = Y)
 *    - Trigger, ADS, Reload, Swap (GPIOs com ISR)
 *    - LED RGB (PWM)
 *    - Motor de vibração (GPIO digital)
 *
 *  Protocolo UART para o PC (115 200 baud):
 *    [ 0xFF | axis | val_lo | val_hi ]
 *    axis 0  → movimento X  (IMU roll  → mira horizontal)
 *    axis 1  → movimento Y  (IMU pitch → mira vertical)
 *    axis 2  → clique       (gesto de cutucada na IMU)
 *    axis 3  → joystick X   (WASD horizontal)
 *    axis 4  → joystick Y   (WASD vertical)
 *    axis 5  → trigger      (clique esquerdo / disparo)
 *    axis 6  → ADS          (clique direito  / mira)
 *    axis 7  → reload
 *    axis 8  → swap arma
 *
 *  Sem Bluetooth nesta versão.
 * ============================================================
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#include "mpu6050.h"   /* driver baixo nível — já usado no código original */
#include "Fusion.h"    /* AHRS Madgwick/Mahony                             */

/* ============================================================
 *  PINAGEM
 * ============================================================ */
#define I2C_SDA_GPIO     4
#define I2C_SCL_GPIO     5
#define MPU_ADDRESS      0x68

#define LED_R_PIN        13
#define LED_G_PIN        11
#define LED_B_PIN        12

#define VIBRA_PIN        15   /* motor de vibração — sinal digital */

#define JOYSTICK_X_GPIO  26   /* ADC0 */
#define JOYSTICK_Y_GPIO  27   /* ADC1 */

#define BTN_TRIGGER_PIN  2
#define BTN_ADS_PIN      3
#define BTN_RELOAD_PIN   6
#define BTN_SWAP_PIN     7

/* ============================================================
 *  PARÂMETROS DE CONTROLE
 * ============================================================ */
#define SAMPLE_PERIOD_F   0.01f   /* período da IMU em segundos */
#define SAMPLE_PERIOD_MS  10      /* período da IMU em ms       */

#define MOUSE_SCALE       2
#define DEAD_ZONE_IMU     12.0f

/* detecção de "cutucada" para clique via acelerômetro */
#define PEAK_THRESHOLD    2000
#define RETURN_THRESHOLD  -1000
#define CLICK_TIMEOUT_MS  300
#define CLICK_COOLDOWN_MS 300

/* joystick */
#define JOY_CENTER        2048
#define JOY_DEAD_ZONE     30
#define JOY_SCALE         255
#define JOY_POLL_MS       50
#define JOY_MA_SIZE       5    /* tamanho do filtro média móvel */

/* vibração */
#define VIBRA_SHORT_MS    80
#define VIBRA_SHOT_MS     40

/* ============================================================
 *  TIPOS
 * ============================================================ */
typedef struct {
    float acel_x, acel_y, acel_z;
    float giro_x, giro_y, giro_z;
} dados_mpu_t;

typedef struct {
    float rolagem;   /* roll  → X do mouse */
    float arfagem;   /* pitch → Y do mouse */
    float guinada;
    bool  clique;
} dados_fusao_t;

typedef struct {
    uint8_t vermelho, verde, azul;
} cor_t;

typedef struct {
    float x, y;
    bool  clique;
} dados_pos_t;

typedef struct {
    int axis;
    int val;
} adc_t;

/* eventos de botão enviados para a Task_Haptic */
typedef enum {
    EVT_TRIGGER = 0,
    EVT_ADS,
    EVT_RELOAD,
    EVT_SWAP,
    EVT_CONNECTED,    /* reservado para uso futuro */
    EVT_LOW_BATTERY,
} haptic_event_t;

/* ============================================================
 *  FILAS E SEMÁFOROS
 * ============================================================ */
static QueueHandle_t fila_mpu;        /* IMU bruta        → tarefa_fusao  */
static QueueHandle_t fila_cor;        /* cor RGB          → tarefa_led    */
static QueueHandle_t fila_pos;        /* posição/clique   → tarefa_uart   */
static QueueHandle_t fila_adc;        /* joystick         → tarefa_uart   */
static QueueHandle_t fila_btn;        /* botões (axis)    → tarefa_uart   */
static QueueHandle_t fila_haptic;     /* eventos hápticos → tarefa_haptic */

/* ============================================================
 *  UTILITÁRIOS PWM
 * ============================================================ */
static void pwm_inicializar_pino(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

static void pwm_definir_ciclo(uint pin, uint8_t duty) {
    uint slice   = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, duty);
}

/* ============================================================
 *  PROTOCOLO UART → PC
 *  Pacote: [ 0xFF | axis | val_lo | val_hi ]
 * ============================================================ */
static void uart_enviar(uint8_t axis, int16_t val) {
    uint8_t pkt[4] = {
        0xFF,
        axis,
        (uint8_t)(val & 0xFF),
        (uint8_t)((val >> 8) & 0xFF)
    };
    for (int i = 0; i < 4; i++)
        putchar_raw(pkt[i]);
}

/* ============================================================
 *  FILTRO MÉDIA MÓVEL (usado no joystick)
 * ============================================================ */
static int media_movel(int vetor[], int tamanho, int dado, int *idx, int *soma) {
    *soma -= vetor[*idx];
    vetor[*idx] = dado;
    *soma += dado;
    *idx = (*idx + 1) % tamanho;
    return *soma / tamanho;
}

/* ============================================================
 *  TASK: tarefa_mpu
 *  Lê acelerômetro + giroscópio a 100 Hz e coloca na fila_mpu.
 * ============================================================ */
static void tarefa_mpu(void *p) {
    /* inicialização I2C e MPU-6050 */
    i2c_init(i2c_default, 400000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    uint8_t wake[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, wake, 2, false);

    int16_t    acel[3], giro[3];
    dados_mpu_t dados;
    uint8_t    buffer[14];
    uint8_t    reg = 0x3B;

    while (1) {
        i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
        i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer, 14, false);

        for (int i = 0; i < 3; i++)
            acel[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        for (int i = 0; i < 3; i++)
            giro[i] = (buffer[8 + i * 2] << 8) | buffer[8 + i * 2 + 1];

        dados.acel_x = acel[0] / 16384.0f;
        dados.acel_y = acel[1] / 16384.0f;
        dados.acel_z = acel[2] / 16384.0f;

        dados.giro_x = giro[0] / 131.0f;
        dados.giro_y = giro[1] / 131.0f;
        dados.giro_z = giro[2] / 131.0f;

        xQueueOverwrite(fila_mpu, &dados);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* ============================================================
 *  TASK: tarefa_fusao
 *  Aplica filtro AHRS (Fusion), calcula euler angles e detecta
 *  gesto de clique por pico de aceleração.
 *  Publica em fila_pos (movimento/clique) e fila_cor (LED RGB).
 * ============================================================ */
typedef enum { WAIT_PEAK, WAIT_RETURN } ClickState;

static void tarefa_fusao(void *p) {
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    dados_mpu_t  dados_mpu;
    dados_fusao_t dados_fusao;
    cor_t        cor;
    dados_pos_t  posicao;

    float r_med = 0, g_med = 0, b_med = 0;
    const float alpha = 0.1f;

    int16_t    x_anterior = 0;
    bool       primeiro   = true;

    ClickState  estado       = WAIT_PEAK;
    TickType_t  tempo_pico   = 0;
    TickType_t  ultimo_click = 0;

    while (1) {
        if (!xQueueReceive(fila_mpu, &dados_mpu, portMAX_DELAY))
            continue;

        /* ---- AHRS ---- */
        FusionVector giro_v = { .axis = { dados_mpu.giro_x,
                                          dados_mpu.giro_y,
                                          dados_mpu.giro_z } };
        FusionVector acel_v = { .axis = { dados_mpu.acel_x,
                                          dados_mpu.acel_y,
                                          dados_mpu.acel_z } };

        FusionAhrsUpdateNoMagnetometer(&ahrs, giro_v, acel_v, SAMPLE_PERIOD_F);
        FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        dados_fusao.rolagem = e.angle.roll;
        dados_fusao.arfagem = e.angle.pitch;
        dados_fusao.guinada = e.angle.yaw;
        dados_fusao.clique  = false;

        /* ---- Detecção de clique por gesto (cutucada) ---- */
        int16_t    x_bruto = (int16_t)(dados_mpu.acel_x * 16384.0f);
        int16_t    delta   = x_bruto - x_anterior;
        TickType_t agora   = xTaskGetTickCount();

        if (!primeiro) {
            switch (estado) {
                case WAIT_PEAK:
                    if (delta > PEAK_THRESHOLD &&
                        (agora - ultimo_click) > pdMS_TO_TICKS(CLICK_COOLDOWN_MS)) {
                        estado     = WAIT_RETURN;
                        tempo_pico = agora;
                    }
                    break;

                case WAIT_RETURN:
                    if ((agora - tempo_pico) > pdMS_TO_TICKS(CLICK_TIMEOUT_MS)) {
                        estado = WAIT_PEAK;   /* timeout — cancela */
                    } else if (delta < RETURN_THRESHOLD) {
                        dados_fusao.clique = true;
                        ultimo_click       = agora;
                        estado             = WAIT_PEAK;
                    }
                    break;
            }
        } else {
            primeiro = false;
        }
        x_anterior = x_bruto;

        /* ---- LED RGB proporcional à orientação ---- */
        float rolagem = fmaxf(-90.0f, fminf(90.0f, dados_fusao.rolagem));
        float arfagem = fmaxf(-90.0f, fminf(90.0f, dados_fusao.arfagem));

        float r_tgt = (rolagem > 0) ? (rolagem /  90.0f) * 255.0f : 0.0f;
        float b_tgt = (rolagem < 0) ? (-rolagem / 90.0f) * 255.0f : 0.0f;
        float g_tgt = (arfagem < 0) ? (-arfagem / 90.0f) * 255.0f : 0.0f;

        r_med = alpha * r_tgt + (1.0f - alpha) * r_med;
        g_med = alpha * g_tgt + (1.0f - alpha) * g_med;
        b_med = alpha * b_tgt + (1.0f - alpha) * b_med;

        cor.vermelho = (uint8_t)r_med;
        cor.verde    = (uint8_t)g_med;
        cor.azul     = (uint8_t)b_med;

        posicao.x      = dados_fusao.rolagem;
        posicao.y      = dados_fusao.arfagem;
        posicao.clique = dados_fusao.clique;

        xQueueOverwrite(fila_cor, &cor);
        xQueueOverwrite(fila_pos, &posicao);
    }
}

/* ============================================================
 *  TASK: tarefa_led
 *  Recebe cor da fila_cor e aplica via PWM nos três canais RGB.
 * ============================================================ */
static void tarefa_led(void *p) {
    pwm_inicializar_pino(LED_R_PIN);
    pwm_inicializar_pino(LED_G_PIN);
    pwm_inicializar_pino(LED_B_PIN);

    cor_t cor;
    while (1) {
        if (xQueueReceive(fila_cor, &cor, portMAX_DELAY)) {
            pwm_definir_ciclo(LED_R_PIN, cor.vermelho);
            pwm_definir_ciclo(LED_G_PIN, cor.verde);
            pwm_definir_ciclo(LED_B_PIN, cor.azul);
        }
    }
}

/* ============================================================
 *  TASK: tarefa_haptic
 *  Recebe eventos hápticos e aciona o motor de vibração.
 *  Padrões distintos por evento.
 * ============================================================ */
static void tarefa_haptic(void *p) {
    gpio_init(VIBRA_PIN);
    gpio_set_dir(VIBRA_PIN, GPIO_OUT);
    gpio_put(VIBRA_PIN, 0);

    haptic_event_t evt;
    while (1) {
        if (xQueueReceive(fila_haptic, &evt, portMAX_DELAY)) {
            switch (evt) {
                case EVT_TRIGGER:
                    /* recoil curto */
                    gpio_put(VIBRA_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(VIBRA_SHOT_MS));
                    gpio_put(VIBRA_PIN, 0);
                    break;

                case EVT_ADS:
                    /* pulso duplo suave */
                    for (int p = 0; p < 2; p++) {
                        gpio_put(VIBRA_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(30));
                        gpio_put(VIBRA_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                    break;

                case EVT_RELOAD:
                    /* vibração longa */
                    gpio_put(VIBRA_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(VIBRA_SHORT_MS * 2));
                    gpio_put(VIBRA_PIN, 0);
                    break;

                case EVT_SWAP:
                    /* pulso triplo rápido */
                    for (int p = 0; p < 3; p++) {
                        gpio_put(VIBRA_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(20));
                        gpio_put(VIBRA_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                    break;

                case EVT_LOW_BATTERY:
                    /* 5 pulsos lentos de alerta */
                    for (int p = 0; p < 5; p++) {
                        gpio_put(VIBRA_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_put(VIBRA_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    break;

                default:
                    /* EVT_CONNECTED ou outro: vibração curta única */
                    gpio_put(VIBRA_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(VIBRA_SHORT_MS));
                    gpio_put(VIBRA_PIN, 0);
                    break;
            }
        }
    }
}

/* ============================================================
 *  TASK: tarefa_joystick_x  /  tarefa_joystick_y
 *  Lê ADC com média móvel, aplica zona morta e escala,
 *  e envia para fila_adc.
 * ============================================================ */
static void tarefa_joystick_x(void *p) {
    int vetor[JOY_MA_SIZE] = {0};
    int idx = 0, soma = 0;

    while (1) {
        adc_select_input(0);
        uint16_t raw = adc_read();

        int media   = media_movel(vetor, JOY_MA_SIZE, raw, &idx, &soma);
        int delta   = media - JOY_CENTER;
        delta = (delta * JOY_SCALE) / JOY_CENTER;
        if (delta > -JOY_DEAD_ZONE && delta < JOY_DEAD_ZONE)
            delta = 0;

        adc_t dado = { .axis = 3, .val = delta };  /* axis 3 = joy X */
        xQueueSend(fila_adc, &dado, 0);             /* sem bloquear   */

        vTaskDelay(pdMS_TO_TICKS(JOY_POLL_MS));
    }
}

static void tarefa_joystick_y(void *p) {
    int vetor[JOY_MA_SIZE] = {0};
    int idx = 0, soma = 0;

    while (1) {
        adc_select_input(1);
        uint16_t raw = adc_read();

        int media   = media_movel(vetor, JOY_MA_SIZE, raw, &idx, &soma);
        int delta   = media - JOY_CENTER;
        delta = (delta * JOY_SCALE) / JOY_CENTER;
        if (delta > -JOY_DEAD_ZONE && delta < JOY_DEAD_ZONE)
            delta = 0;

        adc_t dado = { .axis = 4, .val = delta };  /* axis 4 = joy Y */
        xQueueSend(fila_adc, &dado, 0);

        vTaskDelay(pdMS_TO_TICKS(JOY_POLL_MS));
    }
}

/* ============================================================
 *  TASK: tarefa_uart
 *  Drena fila_pos (IMU), fila_adc (joystick) e fila_btn
 *  (botões) e serializa tudo via protocolo de 4 bytes.
 * ============================================================ */
static void tarefa_uart(void *p) {
    dados_pos_t posicao;
    adc_t       adc_dado;
    adc_t       btn_dado;

    while (1) {
        /* --- IMU / mouse --- */
        if (xQueueReceive(fila_pos, &posicao, 0)) {
            int16_t vel_x = (fabsf(posicao.x) > DEAD_ZONE_IMU)
                            ? (int16_t)(posicao.x * MOUSE_SCALE) : 0;
            int16_t vel_y = (fabsf(posicao.y) > DEAD_ZONE_IMU)
                            ? (int16_t)(posicao.y * MOUSE_SCALE) : 0;

            if (vel_x)        uart_enviar(0, vel_x);
            if (vel_y)        uart_enviar(1, vel_y);
            if (posicao.clique) uart_enviar(2, 1);
        }

        /* --- Joystick --- */
        if (xQueueReceive(fila_adc, &adc_dado, 0)) {
            if (adc_dado.val != 0)
                uart_enviar((uint8_t)adc_dado.axis, (int16_t)adc_dado.val);
        }

        /* --- Botões (ISR) --- */
        if (xQueueReceive(fila_btn, &btn_dado, 0)) {
            uart_enviar((uint8_t)btn_dado.axis, (int16_t)btn_dado.val);
        }

        /* Cede CPU caso nenhuma fila tenha dados */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ============================================================
 *  ISR — btn_callback
 *  Disparada em borda de descida de qualquer botão.
 *  Envia evento para fila_btn (axis 5-8) e para fila_haptic.
 * ============================================================ */
static void btn_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    adc_t btn = { .val = 1 };
    haptic_event_t evt;

    // Borda de subida — só trata soltura do ADS
    if (events & GPIO_IRQ_EDGE_RISE) {
        if (gpio == BTN_ADS_PIN) {
            btn.axis = 6;
            btn.val  = 0;  // solto
            xQueueSendFromISR(fila_btn, &btn, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        return;
    }

    // Borda de descida — pressionado
    if (!(events & GPIO_IRQ_EDGE_FALL)) return;

    switch (gpio) {
        case BTN_TRIGGER_PIN: btn.axis = 5; evt = EVT_TRIGGER; break;
        case BTN_ADS_PIN:     btn.axis = 6; evt = EVT_ADS;     break;
        case BTN_RELOAD_PIN:  btn.axis = 7; evt = EVT_RELOAD;  break;
        case BTN_SWAP_PIN:    btn.axis = 8; evt = EVT_SWAP;    break;
        default: return;
    }

    xQueueSendFromISR(fila_btn,    &btn, &xHigherPriorityTaskWoken);
    xQueueSendFromISR(fila_haptic, &evt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* sw=========awdsd===================================================
 *  MAIN
 * ============================================================ */
int main(void) {
    stdio_init_all();

    /* ---- ADC ---- */
    adc_init();
    adc_gpio_init(JOYSTICK_X_GPIO);
    adc_gpio_init(JOYSTICK_Y_GPIO);

    /* ---- Botões com pull-up interno e ISR ---- */
    const uint btns[] = { BTN_TRIGGER_PIN, BTN_ADS_PIN,
                          BTN_RELOAD_PIN,  BTN_SWAP_PIN };
    for (int i = 0; i < 4; i++) {
        gpio_init(btns[i]);
        gpio_set_dir(btns[i], GPIO_IN);
        gpio_pull_up(btns[i]);
        uint32_t eventos = (btns[i] == BTN_ADS_PIN)
                       ? GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE
                       : GPIO_IRQ_EDGE_FALL;

        gpio_set_irq_enabled_with_callback(btns[i], eventos, true, &btn_callback);

    }

    /* ---- Filas ---- */
    fila_mpu    = xQueueCreate(1,  sizeof(dados_mpu_t));
    fila_cor    = xQueueCreate(1,  sizeof(cor_t));
    fila_pos    = xQueueCreate(1,  sizeof(dados_pos_t));
    fila_adc    = xQueueCreate(10, sizeof(adc_t));
    fila_btn    = xQueueCreate(8,  sizeof(adc_t));
    fila_haptic = xQueueCreate(8,  sizeof(haptic_event_t));

    /* ---- Tasks  (nome, stack, param, prioridade, handle) ---- */
    /*
     * Prioridades:
     *   3 — tarefa_mpu       (tempo-real, sensor crítico)
     *   2 — tarefa_fusao     (processamento pesado, mas logo atrás)
     *   2 — tarefa_joystick  (mesma urgência que fusão)
     *   1 — tarefa_uart      (saída — pode ter pequena latência)
     *   1 — tarefa_led       (cosmético)
     *   1 — tarefa_haptic    (feedback, tolerante a atraso leve)
     */
    xTaskCreate(tarefa_mpu,        "MPU",     4096, NULL, 3, NULL);
    xTaskCreate(tarefa_fusao,      "Fusao",   4096, NULL, 2, NULL);
    xTaskCreate(tarefa_joystick_x, "JoyX",    1024, NULL, 2, NULL);
    xTaskCreate(tarefa_joystick_y, "JoyY",    1024, NULL, 2, NULL);
    xTaskCreate(tarefa_uart,       "UART",    2048, NULL, 1, NULL);
    xTaskCreate(tarefa_led,        "LED",     2048, NULL, 1, NULL);
    xTaskCreate(tarefa_haptic,     "Haptic",  1024, NULL, 1, NULL);

    vTaskStartScheduler();

    /* Nunca deve chegar aqui */
    while (1);
}
