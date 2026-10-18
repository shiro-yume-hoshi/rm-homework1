//
// Created by shiro2233 on 2026/10/18.
//

#include "bmi088.h"

extern SPI_HandleTypeDef hspi1; // 声明外部 SPI 句柄

// ---------------- 底层通信层 ----------------

// 单字节发送
void bmi088_write_byte(uint8_t tx_data) {
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
}

// 多字节接收
void bmi088_read_bytes(uint8_t *rx_data, uint8_t length) {
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
}

// 陀螺仪写寄存器
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data) {
    CS_GYRO_L();
    bmi088_write_byte(reg & 0x7F); // 最高位清 0 表示写
    bmi088_write_byte(data);
    CS_GYRO_H();
}

// 陀螺仪读寄存器
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
    CS_GYRO_L();
    bmi088_write_byte(reg | 0x80); // 最高位置 1 表示读
    bmi088_read_bytes(rx_data, length);
    CS_GYRO_H();
}

// 加速度计写寄存器
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
    CS_GYRO_H(); // 确保另一个片选拉高
    CS_ACCEL_L();
    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(data);
    CS_ACCEL_H();
}

// 加速度计读寄存器（注意：多读一个 dummy byte）
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
    uint8_t dummy;
    CS_GYRO_H();
    CS_ACCEL_L();
    bmi088_write_byte(reg | 0x80); // 读指令
    bmi088_read_bytes(&dummy, 1);  // 丢弃 dummy byte
    bmi088_read_bytes(rx_data, length); // 读有效数据
    CS_ACCEL_H();
}

// ---------------- 初始化层 ----------------

uint8_t bmi088_init(void) {
    uint8_t dummy = 0;
    uint8_t acc_id = 0, gyro_id = 0;
    uint8_t acc_power[2] = {0};
    uint8_t acc_config[2] = {0};
    uint8_t gyro_config[3] = {0};

    CS_ACCEL_H();
    CS_GYRO_H();
    HAL_Delay(1);

    // 1. 唤醒加速度计（切 SPI 模式），第一次读取产生 CS 上升沿
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // 2. 软复位
    bmi088_accel_write_single_reg(0x7E, 0xB6);
    HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x14, 0xB6);
    HAL_Delay(30);

    // 软复位后的虚拟读取
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // 3. 检查 Chip ID
    bmi088_accel_read_reg(0x00, &acc_id, 1);
    if (acc_id != 0x1E) return 0; // 加速度计 ID 错误
    bmi088_gyro_read_reg(0x00, &gyro_id, 1);
    if (gyro_id != 0x0F) return 0; // 陀螺仪 ID 错误

    // 4. 配置加速度计（±6g, 800Hz）
    bmi088_accel_write_single_reg(0x7C, 0x00); HAL_Delay(5); // Active
    bmi088_accel_write_single_reg(0x7D, 0x04); HAL_Delay(5); // Enable accel
    bmi088_accel_write_single_reg(0x41, 0x01); HAL_Delay(1); // Range
    bmi088_accel_write_single_reg(0x40, 0xAB); HAL_Delay(40);// Conf

    // 5. 配置陀螺仪（±2000dps, 1000Hz）
    bmi088_gyro_write_single_reg(0x11, 0x00); HAL_Delay(30); // Normal mode
    bmi088_gyro_write_single_reg(0x0F, 0x00); HAL_Delay(1);  // Range
    bmi088_gyro_write_single_reg(0x10, 0x02); HAL_Delay(10); // Bandwidth

    // 6. 读回校验（防呆）
    bmi088_accel_read_reg(0x7C, acc_power, 2);
    if (acc_power[0] != 0x00 || acc_power[1] != 0x04) return 0;
    bmi088_accel_read_reg(0x40, acc_config, 2);
    if (acc_config[0] != 0xAB || (acc_config[1] & 0x03) != 0x01) return 0;
    bmi088_gyro_read_reg(0x0F, gyro_config, 3);
    if ((gyro_config[0] & 0x07) != 0x00 || (gyro_config[1] & 0x07) != 0x02 || gyro_config[2] != 0x00) return 0;

    return 1; // 初始化成功
}

// ---------------- 数据读取与换算层 ----------------

// 加速度计数据换算，输出单位 m/s²
void bmi088_read_accel(float *accel) {
    uint8_t data[6];
    bmi088_accel_read_reg(0x12, data, 6); // 连续读 6 个字节

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    // ±6g 量程对应的灵敏度系数 (查阅数据手册)
    // ±6g: 1 LSB = 0.001795 m/s²（已经含 g，不要再乘 9.8）
    const float accel_sen = 0.001795f;

    accel[0] = raw_x * accel_sen;
    accel[1] = raw_y * accel_sen;
    accel[2] = raw_z * accel_sen;
}

// 陀螺仪数据换算，输出单位 rad/s
void bmi088_read_gyro(float *gyro) {
    uint8_t data[6];
    bmi088_gyro_read_reg(0x02, data, 6); // 连续读 6 个字节

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    // ±2000°/s 量程对应的灵敏度系数
    // 若为 ±2000°/s，则 1 LSB = 0.061035 °/s = 0.001065 rad/s
    const float gyro_sen = 0.001065f;

    gyro[0] = raw_x * gyro_sen;
    gyro[1] = raw_y * gyro_sen;
    gyro[2] = raw_z * gyro_sen;
}
