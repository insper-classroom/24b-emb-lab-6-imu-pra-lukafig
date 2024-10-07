#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include <Fusion.h>

#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BAUD_RATE 115200

QueueHandle_t xQueuexy;

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
#define SAMPLE_PERIOD (0.01f)

typedef struct xy {
    int axis;
    int val;
} xy;

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

void uart_init_setup() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        int xxx = 0;
        int yyy = 0;

        if (euler.angle.yaw > 12 || euler.angle.yaw < -12) {
            xxx = euler.angle.yaw;
        }

        if (xxx > 255) {
            xxx = 255;
        } else if (xxx < -255) {
            xxx = -255;
        }

        xy data;
        data.axis = 0; // eixo Yaw
        data.val = -xxx;
        xQueueSend(xQueuexy, &data, portMAX_DELAY);

        if (euler.angle.pitch > 12 || euler.angle.pitch < -12) {
            yyy = euler.angle.pitch ;
        }

        if (yyy > 255) {
            yyy = 255;
        } else if (yyy < -255) {
            yyy = -255;
        }

        data.axis = 1; // eixo Pitch
        data.val = -yyy;
        xQueueSend(xQueuexy, &data, portMAX_DELAY); // Send pitch value

        if (accelerometer.axis.x > 1.5) {
            data.axis = 2; // eixo Pitch
            data.val = 1;
            xQueueSend(xQueuexy, &data, portMAX_DELAY); // Send pitch value
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p) {
    xy data;
    while (1) {
        if (xQueueReceive(xQueuexy, &data, pdMS_TO_TICKS(10))) {
            int16_t value = (int16_t)data.val; // Directly use the value

            uint8_t val_0 = value & 0xFF;        // Byte menos significativo
            uint8_t val_1 = (value >> 8) & 0xFF; // Byte mais significativo
            
            uart_putc(UART_ID, data.axis); // Enviar o eixo (0 para Yaw, 1 para Pitch)
            uart_putc(UART_ID, val_0); // Enviar o byte menos significativo
            uart_putc(UART_ID, val_1); // Enviar o byte mais significativo
            uart_putc(UART_ID, 0xFF);   // Enviar delimitador

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } 
}

int main() {
    stdio_init_all();
    uart_init_setup();

    xQueuexy = xQueueCreate(10, sizeof(xy));

    xTaskCreate(mpu6050_task, "mpu6050_Task", 4096, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_Task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}
