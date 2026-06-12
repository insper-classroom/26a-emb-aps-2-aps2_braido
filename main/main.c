#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "mpu6050.h"
#include "Fusion.h"

// ───────── CONFIG ─────────
#define SAMPLE_PERIOD 0.01f
#define SAMPLE_PERIOD_MS 10

#define I2C_SDA_GPIO 4
#define I2C_SCL_GPIO 5
#define MPU_ADDRESS  0x68

#define LED_R_PIN 13
#define LED_G_PIN 11
#define LED_B_PIN 12

#define MOUSE_SCALE 3
#define DEAD_ZONE 7.0f

// ───────── CLICK PARAMS ─────────
#define PEAK_THRESHOLD     2000
#define RETURN_THRESHOLD  -1000
#define CLICK_TIMEOUT_MS   300
#define CLICK_COOLDOWN_MS  300

// ───────── STRUCTS ─────────
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} mpu_data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    bool click;
} fusion_data_t;

typedef struct {
    uint8_t r, g, b;
} color_t;

typedef struct {
    float x, y;
    bool click;
} pos_data_t;

// ───────── FILAS ─────────
QueueHandle_t xQueueMPU;
QueueHandle_t xQueueColor;
QueueHandle_t xQueuePos;

// ───────── PWM ─────────
void pwm_init_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

void pwm_set_duty(uint pin, uint8_t duty) {
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, duty);
}

// ───────── MPU ─────────
void mpu6050_init() {
    i2c_init(i2c_default, 400000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[14];
    uint8_t reg = 0x3B;

    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    for (int i = 0; i < 3; i++)
        accel[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];

    for (int i = 0; i < 3; i++)
        gyro[i] = (buffer[8 + i * 2] << 8) | buffer[8 + i * 2 + 1];
}

// ───────── UART ─────────
void uart_send(uint8_t axis, int16_t val) {
    uint8_t pkt[4] = {0xFF, axis, val & 0xFF, (val >> 8) & 0xFF};
    for (int i = 0; i < 4; i++)
        putchar_raw(pkt[i]);
}

// ───────── TASK MPU ─────────
void mpu_task(void *p) {
    mpu6050_init();

    int16_t acc[3], gyro[3];
    mpu_data_t data;

    while (1) {
        mpu6050_read_raw(acc, gyro);

        data.ax = acc[0] / 16384.0f;
        data.ay = acc[1] / 16384.0f;
        data.az = acc[2] / 16384.0f;

        data.gx = gyro[0] / 131.0f;
        data.gy = gyro[1] / 131.0f;
        data.gz = gyro[2] / 131.0f;

        xQueueOverwrite(xQueueMPU, &data);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// ───────── CLICK STATE MACHINE ─────────
typedef enum {
    WAIT_PEAK,
    WAIT_RETURN
} ClickState;

// ───────── TASK FUSION ─────────
void fusion_task(void *p) {
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    mpu_data_t mpu;
    fusion_data_t fdata;
    color_t color;
    pos_data_t pos;

    float r_avg = 0, g_avg = 0, b_avg = 0;
    const float alpha = 0.1f;

    int16_t prev_x = 0;
    bool first = true;

    ClickState state = WAIT_PEAK;
    TickType_t peak_time = 0;
    TickType_t last_click = 0;

    while (1) {
        if (xQueueReceive(xQueueMPU, &mpu, portMAX_DELAY)) {

            FusionVector gyro = { .axis = { mpu.gx, mpu.gy, mpu.gz } };
            FusionVector accel = { .axis = { mpu.ax, mpu.ay, mpu.az } };

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel, SAMPLE_PERIOD);
            FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

            fdata.roll = e.angle.roll;
            fdata.pitch = e.angle.pitch;
            fdata.yaw = e.angle.yaw;
            fdata.click = false;

            int16_t raw_x = (int16_t)(mpu.ax * 16384.0f);
            int16_t delta = raw_x - prev_x;
            TickType_t now = xTaskGetTickCount();

            if (!first) {

                switch (state) {

                    case WAIT_PEAK:
                        if (delta > PEAK_THRESHOLD &&
                            (now - last_click) > pdMS_TO_TICKS(CLICK_COOLDOWN_MS)) {

                            state = WAIT_RETURN;
                            peak_time = now;
                        }
                        break;

                    case WAIT_RETURN:
                        if ((now - peak_time) > pdMS_TO_TICKS(CLICK_TIMEOUT_MS)) {
                            state = WAIT_PEAK;
                        }
                        else if (delta < RETURN_THRESHOLD) {
                            fdata.click = true;
                            last_click = now;
                            state = WAIT_PEAK;
                        }
                        break;
                }

            } else {
                first = false;
            }

            prev_x = raw_x;

            // LED
            float roll = fmaxf(-90, fminf(90, fdata.roll));
            float pitch = fmaxf(-90, fminf(90, fdata.pitch));

            float r = (roll > 0) ? (roll / 90.0f) * 255 : 0;
            float b = (roll < 0) ? (-roll / 90.0f) * 255 : 0;
            float g = (pitch < 0) ? (-pitch / 90.0f) * 255 : 0;

            r_avg = alpha * r + (1 - alpha) * r_avg;
            g_avg = alpha * g + (1 - alpha) * g_avg;
            b_avg = alpha * b + (1 - alpha) * b_avg;

            color.r = (uint8_t)r_avg;
            color.g = (uint8_t)g_avg;
            color.b = (uint8_t)b_avg;

            pos.x = fdata.roll;
            pos.y = fdata.pitch;
            pos.click = fdata.click;

            xQueueOverwrite(xQueueColor, &color);
            xQueueOverwrite(xQueuePos, &pos);
        }
    }
}

// ───────── TASK LED ─────────
void led_task(void *p) {
    pwm_init_pin(LED_R_PIN);
    pwm_init_pin(LED_G_PIN);
    pwm_init_pin(LED_B_PIN);

    color_t color;

    while (1) {
        if (xQueueReceive(xQueueColor, &color, portMAX_DELAY)) {
            pwm_set_duty(LED_R_PIN, color.r);
            pwm_set_duty(LED_G_PIN, color.g);
            pwm_set_duty(LED_B_PIN, color.b);
        }
    }
}

// ───────── TASK UART ─────────
void uart_task(void *p) {
    pos_data_t pos;

    while (1) {
        if (xQueueReceive(xQueuePos, &pos, portMAX_DELAY)) {

            int16_t vx = (fabsf(pos.x) > DEAD_ZONE) ? pos.x * MOUSE_SCALE : 0;
            int16_t vy = (fabsf(pos.y) > DEAD_ZONE) ? pos.y * MOUSE_SCALE : 0;

            if (vx) uart_send(0, vx);
            if (vy) uart_send(1, vy);
            if (pos.click) uart_send(2, 1);
        }
    }
}

// ───────── MAIN ─────────
int main() {
    stdio_init_all();

    xQueueMPU   = xQueueCreate(1, sizeof(mpu_data_t));
    xQueueColor = xQueueCreate(1, sizeof(color_t));
    xQueuePos   = xQueueCreate(1, sizeof(pos_data_t));

    xTaskCreate(mpu_task,    "MPU",    4096, NULL, 3, NULL);
    xTaskCreate(fusion_task, "Fusion", 4096, NULL, 2, NULL);
    xTaskCreate(led_task,    "LED",    2048, NULL, 1, NULL);
    xTaskCreate(uart_task,   "UART",   2048, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}