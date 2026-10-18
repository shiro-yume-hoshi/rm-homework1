//
// Created by shiro2233 on 2026/10/18.
//

#ifndef LESSON5_SPI_IMU_BMI088_H
#define LESSON5_SPI_IMU_BMI088_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

// 定义读写函数的返回值状态
typedef enum {
    BMI088_OK = 0,
    BMI088_ERROR = 1
} BMI088_Status;

// 片选宏定义
#define CS_ACCEL_L()  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET)
#define CS_ACCEL_H()  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET)
#define CS_GYRO_L()   HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET)
#define CS_GYRO_H()   HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET)

#ifdef __cplusplus
extern "C" {
#endif

// 初始化函数
uint8_t bmi088_init(void);

// 读取数据函数（返回物理量）
void bmi088_read_accel(float *accel);
void bmi088_read_gyro(float *gyro);

// 寄存器级读写（给 IMU 解包用）
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);

#ifdef __cplusplus
}
#endif


#endif //LESSON5_SPI_IMU_BMI088_H
