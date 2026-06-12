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
 *    - Motor de vibração (GPIO digital) — requer transistor NPN
 *    - HC-06 Bluetooth (UART1, TX=GPIO4... ATENÇÃO: ver pinagem abaixo)
 *
 *  PINAGEM HC-06:
 *    HC-06 VCC → 3V3 do Pico
 *    HC-06 GND → GND do Pico
 *    HC-06 TX  → GPIO 9  (UART1 RX do Pico)
 *    HC-06 RX  → GPIO 8  (UART1 TX do Pico)
 *
 *    ATENÇÃO: I2C já usa GPIO 4 e 5.
 *    Por isso o HC-06 usa UART1 (GPIO 8/9) e não UART0 (GPIO 0/1).
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
 *  Protocolo Bluetooth (mesmo formato):
 *    Todos os eventos acima são espelhados via BT.
 *    O botão reload (GPIO 6) também ativa/desativa o BT.
 *    Primeiro pressionamento: BT liga (LED pisca azul 3x).
 *    Segundo pressionamento: BT desliga (LED pisca vermelho 1x).
 *
 *  Comandos recebidos pelo HC-06 (1 byte):
 *    0x01 → rumble (vibração curta)
 *    0x02 → LED vermelho  (dano)
 *    0x03 → LED verde     (cura)
 *    0x04 → LED azul      (escudo)
 *    0x05 → LED apagado   (reset)
 * ============================================================ */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

#include "mpu6050.h"
#include "Fusion.h"

/* ============================================================
 *  PINAGEM
 * ============================================================ */
#define I2C_SDA_GPIO     4
#define I2C_SCL_GPIO     5
#define MPU_ADDRESS      0x68

#define LED_R_PIN        13
#define LED_G_PIN        11
#define LED_B_PIN        12

#define VIBRA_PIN        15

#define JOYSTICK_X_GPIO  26   /* ADC0 */
#define JOYSTICK_Y_GPIO  27   /* ADC1 */

#define BTN_TRIGGER_PIN  2
#define BTN_ADS_PIN      3
#define BTN_RELOAD_PIN   6
#define BTN_SWAP_PIN     7

/* HC-06 — UART1 */
#define BT_UART          uart1
#define BT_TX_PIN        8    /* UART1 TX */
#define BT_RX_PIN        9    /* UART1 RX */
#define BT_BAUD          9600 /* padrão de fábrica do HC-06 */
                              /* se reconfigurado via AT: trocar para 115200 */

/* ============================================================
 *  PARÂMETROS DE CONTROLE
 * ============================================================ */
#define SAMPLE_PERIOD_F   0.01f
#define SAMPLE_PERIOD_MS  10

#define MOUSE_SCALE       2
#define DEAD_ZONE_IMU     8.0f

#define PEAK_THRESHOLD    2000
#define RETURN_THRESHOLD  -1000
#define CLICK_TIMEOUT_MS  300
#define CLICK_COOLDOWN_MS 300

#define JOY_CENTER        2048
#define JOY_DEAD_ZONE     30
#define JOY_SCALE         255
#define JOY_POLL_MS       50
#define JOY_MA_SIZE       5

#define VIBRA_SHORT_MS    3000
#define VIBRA_SHOT_MS     3000

#define LED_FLASH_MS      100

/* debounce do reload para não comutar BT no bounce */
#define RELOAD_DEBOUNCE_MS  50

/* ============================================================
 *  TIPOS
 * ============================================================ */
typedef struct {
    float acel_x, acel_y, acel_z;
    float giro_x, giro_y, giro_z;
} dados_mpu_t;

typedef struct {
    float rolagem;
    float arfagem;
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

typedef enum {
    EVT_TRIGGER = 0,
    EVT_ADS,
    EVT_RELOAD,
    EVT_SWAP,
    EVT_CONNECTED,
    EVT_LOW_BATTERY,
} haptic_event_t;

/* comandos recebidos via Bluetooth */
typedef enum {
    BT_CMD_RUMBLE    = 0x01,
    BT_CMD_LED_RED   = 0x02,
    BT_CMD_LED_GREEN = 0x03,
    BT_CMD_LED_BLUE  = 0x04,
    BT_CMD_LED_OFF   = 0x05,
} bt_cmd_t;

/* ============================================================
 *  ESTADO GLOBAL DO BLUETOOTH
 *  Protegido por mutex — lido pela tarefa_uart e tarefa_bt_rx.
 * ============================================================ */
static volatile bool     bt_ativo = false;
static SemaphoreHandle_t bt_mutex;

/* ============================================================
 *  FILAS
 * ============================================================ */
static QueueHandle_t fila_mpu;
static QueueHandle_t fila_cor;
static QueueHandle_t fila_pos;
static QueueHandle_t fila_adc;
static QueueHandle_t fila_btn;
static QueueHandle_t fila_haptic;
static QueueHandle_t fila_led_flash;
static QueueHandle_t fila_led_override;  /* comandos BT sobrepõem cor da IMU */

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
 *  PROTOCOLO UART → PC (USB)
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
 *  PROTOCOLO BLUETOOTH → PC (HC-06)
 *  Mesmo formato de 4 bytes. Só envia se bt_ativo = true.
 * ============================================================ */
static void bt_enviar(uint8_t axis, int16_t val) {
    if (xSemaphoreTake(bt_mutex, 0) == pdTRUE) {
        bool ativo = bt_ativo;
        xSemaphoreGive(bt_mutex);
        if (!ativo) return;
    } else {
        return;
    }

    uint8_t pkt[4] = {
        0xFF,
        axis,
        (uint8_t)(val & 0xFF),
        (uint8_t)((val >> 8) & 0xFF)
    };
    uart_write_blocking(BT_UART, pkt, 4);
}

/* ============================================================
 *  FILTRO MÉDIA MÓVEL
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
 * ============================================================ */
static void tarefa_mpu(void *p) {
    i2c_init(i2c_default, 400000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    uint8_t wake[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, wake, 2, false);

    int16_t     acel[3], giro[3];
    dados_mpu_t dados;
    uint8_t     buffer[14];
    uint8_t     reg = 0x3B;

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
 * ============================================================ */
typedef enum { WAIT_PEAK, WAIT_RETURN } ClickState;

static void tarefa_fusao(void *p) {
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    dados_mpu_t   dados_mpu;
    dados_fusao_t dados_fusao;
    cor_t         cor;
    dados_pos_t   posicao;

    float r_med = 0, g_med = 0, b_med = 0;
    const float alpha = 0.1f;

    int16_t    x_anterior = 0;
    bool       primeiro   = true;
    ClickState estado      = WAIT_PEAK;
    TickType_t tempo_pico  = 0;
    TickType_t ultimo_click = 0;

    while (1) {
        if (!xQueueReceive(fila_mpu, &dados_mpu, portMAX_DELAY))
            continue;

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
                        estado = WAIT_PEAK;
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
 *  Prioridade de aplicação:
 *    1. Flash de disparo (branco por LED_FLASH_MS)
 *    2. Override do BT (cor enviada pelo PC via Bluetooth)
 *    3. Cor normal da orientação IMU
 * ============================================================ */
static void tarefa_led(void *p) {
    pwm_inicializar_pino(LED_R_PIN);
    pwm_inicializar_pino(LED_G_PIN);
    pwm_inicializar_pino(LED_B_PIN);

    cor_t   cor;
    cor_t   override     = {0, 0, 0};
    bool    tem_override = false;
    uint8_t flash;

    while (1) {
        /* 1. Flash de disparo */
        if (xQueueReceive(fila_led_flash, &flash, 0)) {
            pwm_definir_ciclo(LED_R_PIN, 255);
            pwm_definir_ciclo(LED_G_PIN, 255);
            pwm_definir_ciclo(LED_B_PIN, 255);
            vTaskDelay(pdMS_TO_TICKS(LED_FLASH_MS));
            xQueueReset(fila_cor);
        }

        /* 2. Override do BT (cor enviada pelo PC) */
        if (xQueueReceive(fila_led_override, &override, 0)) {
            tem_override = true;
        }

        if (tem_override) {
            pwm_definir_ciclo(LED_R_PIN, override.vermelho);
            pwm_definir_ciclo(LED_G_PIN, override.verde);
            pwm_definir_ciclo(LED_B_PIN, override.azul);
            /* override dura 500ms depois do último comando */
            vTaskDelay(pdMS_TO_TICKS(500));
            tem_override = false;
            xQueueReset(fila_cor);
            continue;
        }

        /* 3. Cor normal da IMU */
        if (xQueueReceive(fila_cor, &cor, pdMS_TO_TICKS(10))) {
            pwm_definir_ciclo(LED_R_PIN, cor.vermelho);
            pwm_definir_ciclo(LED_G_PIN, cor.verde);
            pwm_definir_ciclo(LED_B_PIN, cor.azul);
        }
    }
}

/* ============================================================
 *  TASK: tarefa_haptic
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
                    gpio_put(VIBRA_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(VIBRA_SHOT_MS));
                    gpio_put(VIBRA_PIN, 0);
                    break;

                case EVT_ADS:
                    for (int i = 0; i < 2; i++) {
                        gpio_put(VIBRA_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(30));
                        gpio_put(VIBRA_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                    break;

                case EVT_RELOAD:
                    gpio_put(VIBRA_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(VIBRA_SHORT_MS * 2));
                    gpio_put(VIBRA_PIN, 0);
                    break;

                case EVT_SWAP:
                    for (int i = 0; i < 3; i++) {
                        gpio_put(VIBRA_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(20));
                        gpio_put(VIBRA_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                    break;

                case EVT_CONNECTED:
                    /* 3 pulsos ao conectar BT */
                    for (int i = 0; i < 3; i++) {
                        gpio_put(VIBRA_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(80));
                        gpio_put(VIBRA_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(80));
                    }
                    break;

                case EVT_LOW_BATTERY:
                    for (int i = 0; i < 5; i++) {
                        gpio_put(VIBRA_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_put(VIBRA_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    break;

                default:
                    gpio_put(VIBRA_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(VIBRA_SHORT_MS));
                    gpio_put(VIBRA_PIN, 0);
                    break;
            }
        }
    }
}

/* ============================================================
 *  TASK: tarefa_joystick_x
 * ============================================================ */
static void tarefa_joystick_x(void *p) {
    int vetor[JOY_MA_SIZE] = {0};
    int idx = 0, soma = 0;

    while (1) {
        adc_select_input(0);
        uint16_t raw = adc_read();

        int media = media_movel(vetor, JOY_MA_SIZE, raw, &idx, &soma);
        int delta = media - JOY_CENTER;
        delta = (delta * JOY_SCALE) / JOY_CENTER;
        if (delta > -JOY_DEAD_ZONE && delta < JOY_DEAD_ZONE)
            delta = 0;

        adc_t dado = { .axis = 3, .val = -delta };
        xQueueSend(fila_adc, &dado, 0);
        vTaskDelay(pdMS_TO_TICKS(JOY_POLL_MS));
    }
}

/* ============================================================
 *  TASK: tarefa_joystick_y
 * ============================================================ */
static void tarefa_joystick_y(void *p) {
    int vetor[JOY_MA_SIZE] = {0};
    int idx = 0, soma = 0;

    while (1) {
        adc_select_input(1);
        uint16_t raw = adc_read();

        int media = media_movel(vetor, JOY_MA_SIZE, raw, &idx, &soma);
        int delta = media - JOY_CENTER;
        delta = (delta * JOY_SCALE) / JOY_CENTER;
        if (delta > -JOY_DEAD_ZONE && delta < JOY_DEAD_ZONE)
            delta = 0;

        adc_t dado = { .axis = 4, .val = -delta };
        xQueueSend(fila_adc, &dado, 0);
        vTaskDelay(pdMS_TO_TICKS(JOY_POLL_MS));
    }
}

/* ============================================================
 *  TASK: tarefa_uart
 *  Envia para USB e, se bt_ativo, espelha tudo via Bluetooth.
 * ============================================================ */
static void tarefa_uart(void *p) {
    dados_pos_t posicao;
    adc_t       adc_dado;
    adc_t       btn_dado;

    while (1) {
        /* IMU / mouse */
        if (xQueueReceive(fila_pos, &posicao, 0)) {
            int16_t vel_x = (fabsf(posicao.x) > DEAD_ZONE_IMU)
                            ? (int16_t)(posicao.x * MOUSE_SCALE) : 0;
            int16_t vel_y = (fabsf(posicao.y) > DEAD_ZONE_IMU)
                            ? (int16_t)(posicao.y * MOUSE_SCALE) : 0;

            if (vel_x)          { uart_enviar(0, vel_x); bt_enviar(0, vel_x); }
            if (vel_y)          { uart_enviar(1, vel_y); bt_enviar(1, vel_y); }
            if (posicao.clique) { uart_enviar(2, 1);     bt_enviar(2, 1);     }
        }

        /* Joystick — envia sempre (inclusive 0 para soltar tecla) */
        if (xQueueReceive(fila_adc, &adc_dado, 0)) {
            uart_enviar((uint8_t)adc_dado.axis, (int16_t)adc_dado.val);
            bt_enviar  ((uint8_t)adc_dado.axis, (int16_t)adc_dado.val);
        }

        /* Botões */
        if (xQueueReceive(fila_btn, &btn_dado, 0)) {
            uart_enviar((uint8_t)btn_dado.axis, (int16_t)btn_dado.val);
            bt_enviar  ((uint8_t)btn_dado.axis, (int16_t)btn_dado.val);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ============================================================
 *  TASK: tarefa_bt_rx
 *  Recebe comandos do PC via Bluetooth e age localmente.
 *  Comandos:
 *    0x01 → rumble
 *    0x02 → LED vermelho
 *    0x03 → LED verde
 *    0x04 → LED azul
 *    0x05 → LED apagado
 * ============================================================ */
static void tarefa_bt_rx(void *p) {
    while (1) {
        if (uart_is_readable(BT_UART)) {
            uint8_t cmd = uart_getc(BT_UART);
            cor_t override;

            switch ((bt_cmd_t)cmd) {
                case BT_CMD_RUMBLE: {
                    haptic_event_t evt = EVT_TRIGGER;
                    xQueueSend(fila_haptic, &evt, 0);
                    break;
                }
                case BT_CMD_LED_RED:
                    override = (cor_t){255, 0, 0};
                    xQueueOverwrite(fila_led_override, &override);
                    break;
                case BT_CMD_LED_GREEN:
                    override = (cor_t){0, 255, 0};
                    xQueueOverwrite(fila_led_override, &override);
                    break;
                case BT_CMD_LED_BLUE:
                    override = (cor_t){0, 0, 255};
                    xQueueOverwrite(fila_led_override, &override);
                    break;
                case BT_CMD_LED_OFF:
                    override = (cor_t){0, 0, 0};
                    xQueueOverwrite(fila_led_override, &override);
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ============================================================
 *  ISR — btn_callback
 *
 *  Reload (GPIO 6) tem dupla função:
 *    - Pressionar normalmente → envia axis 7 (reload no jogo)
 *    - Segurar 1s             → toggle Bluetooth (liga/desliga)
 *
 *  Como a ISR não pode medir 1s sozinha, ela registra o tempo
 *  de pressão. A lógica de toggle fica na tarefa_bt_toggle
 *  que monitora um semáforo binário.
 * ============================================================ */
static SemaphoreHandle_t sem_reload_press;   /* sinaliza pressão do reload */
static SemaphoreHandle_t sem_reload_release; /* sinaliza soltura do reload */

static void btn_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    adc_t btn = { .val = 1 };
    haptic_event_t evt;

    /* Borda de subida */
    if (events & GPIO_IRQ_EDGE_RISE) {
        if (gpio == BTN_ADS_PIN) {
            btn.axis = 6;
            btn.val  = 0;
            xQueueSendFromISR(fila_btn, &btn, &xHigherPriorityTaskWoken);
        }
        if (gpio == BTN_RELOAD_PIN) {
            xSemaphoreGiveFromISR(sem_reload_release, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    if (!(events & GPIO_IRQ_EDGE_FALL)) return;

    switch (gpio) {
        case BTN_TRIGGER_PIN: {
            btn.axis = 5;
            evt = EVT_TRIGGER;
            uint8_t flash = 1;
            xQueueSendFromISR(fila_led_flash, &flash, &xHigherPriorityTaskWoken);
            xQueueSendFromISR(fila_btn,    &btn, &xHigherPriorityTaskWoken);
            xQueueSendFromISR(fila_haptic, &evt, &xHigherPriorityTaskWoken);
            break;
        }
        case BTN_ADS_PIN:
            btn.axis = 6; evt = EVT_ADS;
            xQueueSendFromISR(fila_btn,    &btn, &xHigherPriorityTaskWoken);
            xQueueSendFromISR(fila_haptic, &evt, &xHigherPriorityTaskWoken);
            break;

        case BTN_RELOAD_PIN:
            /* Sinaliza pressão para tarefa_bt_toggle medir duração */
            xSemaphoreGiveFromISR(sem_reload_press, &xHigherPriorityTaskWoken);
            break;

        case BTN_SWAP_PIN:
            btn.axis = 8; evt = EVT_SWAP;
            xQueueSendFromISR(fila_btn,    &btn, &xHigherPriorityTaskWoken);
            xQueueSendFromISR(fila_haptic, &evt, &xHigherPriorityTaskWoken);
            break;

        default: break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ============================================================
 *  TASK: tarefa_bt_toggle
 *
 *  Monitora o botão reload para distinguir:
 *    - Toque curto (< 1s) → reload normal (axis 7 + vibração)
 *    - Toque longo (≥ 1s) → toggle Bluetooth
 *
 *  LED indica estado do BT:
 *    Liga BT  → pisca azul 3x
 *    Desliga  → pisca vermelho 1x
 * ============================================================ */
static void tarefa_bt_toggle(void *p) {
    while (1) {
        /* Espera pressão do reload */
        if (xSemaphoreTake(sem_reload_press, portMAX_DELAY) != pdTRUE)
            continue;

        TickType_t t_press = xTaskGetTickCount();

        /* Espera soltura com timeout de 1s */
        bool solto_cedo = (xSemaphoreTake(sem_reload_release,
                           pdMS_TO_TICKS(1000)) == pdTRUE);

        TickType_t duracao = xTaskGetTickCount() - t_press;

        if (solto_cedo && duracao < pdMS_TO_TICKS(1000)) {
            /* Toque curto → reload normal */
            adc_t btn = { .axis = 7, .val = 1 };
            xQueueSend(fila_btn, &btn, 0);
            haptic_event_t evt = EVT_RELOAD;
            xQueueSend(fila_haptic, &evt, 0);

        } else {
            /* Toque longo → toggle BT */
            /* Garante semáforo de soltura limpo */
            xSemaphoreTake(sem_reload_release, 0);

            if (xSemaphoreTake(bt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                bt_ativo = !bt_ativo;
                bool ligou = bt_ativo;
                xSemaphoreGive(bt_mutex);

                cor_t flash_cor;
                int   n_pisca;

                if (ligou) {
                    flash_cor = (cor_t){0, 0, 255};   /* azul = BT ligado */
                    n_pisca   = 3;
                    haptic_event_t evt = EVT_CONNECTED;
                    xQueueSend(fila_haptic, &evt, 0);
                } else {
                    flash_cor = (cor_t){255, 0, 0};   /* vermelho = BT desligado */
                    n_pisca   = 1;
                }

                /* Pisca LED para confirmar */
                for (int i = 0; i < n_pisca; i++) {
                    xQueueOverwrite(fila_led_override, &flash_cor);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    cor_t apagado = {0, 0, 0};
                    xQueueOverwrite(fila_led_override, &apagado);
                    vTaskDelay(pdMS_TO_TICKS(150));
                }
            }
        }
    }
}

/* ============================================================
 *  MAIN
 * ============================================================ */
int main(void) {
    stdio_init_all();

    /* HC-06 — UART1 */
    uart_init(BT_UART, BT_BAUD);
    gpio_set_function(BT_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(BT_RX_PIN, GPIO_FUNC_UART);

    /* ADC */
    adc_init();
    adc_gpio_init(JOYSTICK_X_GPIO);
    adc_gpio_init(JOYSTICK_Y_GPIO);

    /* Botões com pull-up interno e ISR
     * Reload recebe EDGE_RISE também para medir duração */
    const uint btns[] = { BTN_TRIGGER_PIN, BTN_ADS_PIN,
                          BTN_RELOAD_PIN,  BTN_SWAP_PIN };
    for (int i = 0; i < 4; i++) {
        gpio_init(btns[i]);
        gpio_set_dir(btns[i], GPIO_IN);
        gpio_pull_up(btns[i]);

        uint32_t eventos;
        if (btns[i] == BTN_ADS_PIN || btns[i] == BTN_RELOAD_PIN)
            eventos = GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE;
        else
            eventos = GPIO_IRQ_EDGE_FALL;

        gpio_set_irq_enabled_with_callback(btns[i], eventos, true, &btn_callback);
    }

    /* Semáforos */
    bt_mutex           = xSemaphoreCreateMutex();
    sem_reload_press   = xSemaphoreCreateBinary();
    sem_reload_release = xSemaphoreCreateBinary();

    /* Filas */
    fila_mpu          = xQueueCreate(1,  sizeof(dados_mpu_t));
    fila_cor          = xQueueCreate(1,  sizeof(cor_t));
    fila_pos          = xQueueCreate(1,  sizeof(dados_pos_t));
    fila_adc          = xQueueCreate(10, sizeof(adc_t));
    fila_btn          = xQueueCreate(8,  sizeof(adc_t));
    fila_haptic       = xQueueCreate(8,  sizeof(haptic_event_t));
    fila_led_flash    = xQueueCreate(4,  sizeof(uint8_t));
    fila_led_override = xQueueCreate(1,  sizeof(cor_t));

    /* Tasks */
    xTaskCreate(tarefa_mpu,        "MPU",       4096, NULL, 3, NULL);
    xTaskCreate(tarefa_fusao,      "Fusao",     4096, NULL, 2, NULL);
    xTaskCreate(tarefa_joystick_x, "JoyX",      1024, NULL, 2, NULL);
    xTaskCreate(tarefa_joystick_y, "JoyY",      1024, NULL, 2, NULL);
    xTaskCreate(tarefa_uart,       "UART",      2048, NULL, 1, NULL);
    xTaskCreate(tarefa_led,        "LED",       2048, NULL, 1, NULL);
    xTaskCreate(tarefa_haptic,     "Haptic",    1024, NULL, 1, NULL);
    xTaskCreate(tarefa_bt_rx,      "BT_RX",     1024, NULL, 1, NULL);
    xTaskCreate(tarefa_bt_toggle,  "BT_Toggle", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}