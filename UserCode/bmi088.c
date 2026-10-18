//
// Created by shiro2233 on 2026/10/18.
//

#include "bmi088.h"
#include "main.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi1;

#define CS_ACCEL_L() HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET)
#define CS_ACCEL_H() HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET)
#define CS_GYRO_L()  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,  CS1_GYRO_Pin,  GPIO_PIN_RESET)
#define CS_GYRO_H()  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,  CS1_GYRO_Pin,  GPIO_PIN_SET)

// 量程对应的最小分辨率
static const float kAccLsb[4] = {0.000898f, 0.001795f, 0.003590f, 0.007180f};
static const float kGyroLsb[5] = {0.001065f, 0.0005325f, 0.000266f, 0.000133f, 0.0000665f};

static float acc_lsb = 0.001795f;
static float gyro_lsb = 0.001065f;

static void spi_tx(uint8_t d) {
    HAL_SPI_Transmit(&hspi1, &d, 1, 1000);
}

static void spi_rx(uint8_t* d, uint8_t n) {
    HAL_SPI_Receive(&hspi1, d, n, 1000);
}

static void gyro_write(uint8_t reg, uint8_t data) {
    CS_GYRO_L();
    spi_tx(reg & 0x7F);
    spi_tx(data);
    CS_GYRO_H();
}

static void gyro_read(uint8_t reg, uint8_t* d, uint8_t n) {
    CS_GYRO_L();
    spi_tx(reg | 0x80);
    spi_rx(d, n);
    CS_GYRO_H();
}

static void acc_write(uint8_t reg, uint8_t data) {
    CS_GYRO_H();
    CS_ACCEL_L();
    spi_tx(reg & 0x7F);
    spi_tx(data);
    CS_ACCEL_H();
}

static void acc_read(uint8_t reg, uint8_t* d, uint8_t n) {
    uint8_t dummy;
    CS_GYRO_H();
    CS_ACCEL_L();
    spi_tx(reg | 0x80);
    spi_rx(&dummy, 1);   // accel 读之前要先丢一个 dummy
    spi_rx(d, n);
    CS_ACCEL_H();
}

uint8_t bmi088_init(void) {
    uint8_t dummy = 0;
    uint8_t acc_id = 0, gyro_id = 0;
    uint8_t acc_power[2] = {0};
    uint8_t acc_config[2] = {0};
    uint8_t gyro_config[3] = {0};

    CS_ACCEL_H();
    CS_GYRO_H();
    HAL_Delay(1);

    // 第一次读只是产生 CS 上升沿，把 accel 切到 SPI
    acc_read(0x00, &dummy, 1);
    HAL_Delay(1);

    acc_write(0x7E, 0xB6);
    HAL_Delay(1);
    gyro_write(0x14, 0xB6);
    HAL_Delay(30);

    acc_read(0x00, &dummy, 1);
    HAL_Delay(1);

    acc_read(0x00, &acc_id, 1);
    if (acc_id != 0x1E) return 0;
    gyro_read(0x00, &gyro_id, 1);
    if (gyro_id != 0x0F) return 0;

    // accel: ±6g, 800Hz
    acc_write(0x7C, 0x00); HAL_Delay(5);
    acc_write(0x7D, 0x04); HAL_Delay(5);
    acc_write(0x41, 0x01); HAL_Delay(1);
    acc_write(0x40, 0xAB); HAL_Delay(40);

    // gyro: ±2000dps, 1000Hz
    gyro_write(0x11, 0x00); HAL_Delay(30);
    gyro_write(0x0F, 0x00); HAL_Delay(1);
    gyro_write(0x10, 0x02); HAL_Delay(10);

    acc_read(0x7C, acc_power, 2);
    if (acc_power[0] != 0x00 || acc_power[1] != 0x04) return 0;
    acc_read(0x40, acc_config, 2);
    if (acc_config[0] != 0xAB || (acc_config[1] & 0x03) != 0x01) return 0;
    gyro_read(0x0F, gyro_config, 3);
    if ((gyro_config[0] & 0x07) != 0x00 ||
        (gyro_config[1] & 0x07) != 0x02 || gyro_config[2] != 0x00) return 0;

    uint8_t r = 0;
    acc_read(0x41, &r, 1);
    acc_lsb = kAccLsb[r & 0x03];
    gyro_read(0x0F, &r, 1);
    uint8_t g = r & 0x07;
    gyro_lsb = kGyroLsb[g > 4 ? 0 : g];

    return 1;
}

void bmi088_read_accel_raw(int16_t raw[3]) {
    uint8_t d[6] = {0};
    acc_read(0x12, d, 6);
    raw[0] = (int16_t)((d[1] << 8) | d[0]);
    raw[1] = (int16_t)((d[3] << 8) | d[2]);
    raw[2] = (int16_t)((d[5] << 8) | d[4]);
}

void bmi088_read_gyro_raw(int16_t raw[3]) {
    uint8_t d[6] = {0};
    gyro_read(0x02, d, 6);
    raw[0] = (int16_t)((d[1] << 8) | d[0]);
    raw[1] = (int16_t)((d[3] << 8) | d[2]);
    raw[2] = (int16_t)((d[5] << 8) | d[4]);
}

float bmi088_accel_lsb(void) { return acc_lsb; }
float bmi088_gyro_lsb(void) { return gyro_lsb; }
