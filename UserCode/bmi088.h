//
// Created by shiro2233 on 2026/10/18.
//

#ifndef LESSON5_SPI_IMU_BMI088_H
#define LESSON5_SPI_IMU_BMI088_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t bmi088_init(void);
void bmi088_read_accel_raw(int16_t raw[3]);
void bmi088_read_gyro_raw(int16_t raw[3]);
float bmi088_accel_lsb(void);
float bmi088_gyro_lsb(void);

#ifdef __cplusplus
}
#endif

#endif // LESSON5_SPI_IMU_BMI088_H
