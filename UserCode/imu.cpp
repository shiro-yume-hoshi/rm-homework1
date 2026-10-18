//
// Lesson5-SPI-IMU：姿态解算实现
//

#include "imu.hpp"
#include "bmi088.h"
#include "main.h"
#include <cmath>

static const float kRad2Deg = 57.2957795f;
static const float kDeg2Rad = 0.0174532925f;

// 传感器系 -> 机体系（C 板）
const float IMU::R_imu_[3][3] = {
    {-1.0f, 0.0f,  0.0f},
    { 0.0f, 1.0f,  0.0f},
    { 0.0f, 0.0f, -1.0f}
};

static inline void mat3mv(const float R[3][3], const float* v, float* out) {
    for (int i = 0; i < 3; i++) {
        out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
    }
}

IMU::IMU() = default;

void IMU::accCalculate(void) {
    int16_t r[3];
    bmi088_read_accel_raw(r);
    float k = bmi088_accel_lsb();
    for (int i = 0; i < 3; i++) raw_accel_[i] = (float)r[i] * k;
}

void IMU::gyroCalculate(void) {
    int16_t r[3];
    bmi088_read_gyro_raw(r);
    float k = bmi088_gyro_lsb();
    for (int i = 0; i < 3; i++) raw_gyro_[i] = (float)r[i] * k;
}

void IMU::sensorDataTransform(void) {
    float g[3], a[3];
    mat3mv(R_imu_, raw_gyro_, g);
    mat3mv(R_imu_, raw_accel_, a);
    for (int i = 0; i < 3; i++) {
        gyro_sensor_[i]  = g[i] - gyro_bias_[i];
        accel_sensor_[i] = accel_sen_ * a[i] - accel_bias_[i];
    }
}

void IMU::calibrateGyro(uint16_t n) {
    if (n == 0) return;
    float sum[3] = {0, 0, 0}, g[3];
    for (uint16_t k = 0; k < n; k++) {
        gyroCalculate();
        mat3mv(R_imu_, raw_gyro_, g);
        for (int i = 0; i < 3; i++) sum[i] += g[i];
        HAL_Delay(2);
    }
    for (int i = 0; i < 3; i++) gyro_bias_[i] = sum[i] / (float)n;
    b_hat_[0] = b_hat_[1] = b_hat_[2] = 0.0f;
}

// 互补滤波。yaw 没有观测，只能靠陀螺积分，会慢慢漂
void IMU::complementaryUpdate(float dt) {
    float ax = acc_lpf_[0], ay = acc_lpf_[1], az = acc_lpf_[2];
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-3f) return;

    float roll_acc  = atan2f(ay, az) * kRad2Deg;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * kRad2Deg;

    // 机体角速度 -> 欧拉角速率
    float r = euler_[0] * kDeg2Rad;
    float p = euler_[1] * kDeg2Rad;
    float sr = sinf(r), cr = cosf(r);
    float cp = cosf(p);
    if (cp < 0.1f) cp = 0.1f;
    float tp = sinf(p) / cp;
    float gx = gyro_sensor_[0], gy = gyro_sensor_[1], gz = gyro_sensor_[2];
    float roll_rate  = gx + sr * tp * gy + cr * tp * gz;
    float pitch_rate = cr * gy - sr * gz;
    float yaw_rate   = (sr * gy + cr * gz) / cp;

    euler_[0] = comp_k_ * (euler_[0] + roll_rate  * dt * kRad2Deg) + (1.0f - comp_k_) * roll_acc;
    euler_[1] = comp_k_ * (euler_[1] + pitch_rate * dt * kRad2Deg) + (1.0f - comp_k_) * pitch_acc;
    euler_[2] = euler_[2] + yaw_rate * dt * kRad2Deg;

    euler2Quat();
}

void IMU::mahonyUpdate(float dt) {
    float ax = acc_lpf_[0], ay = acc_lpf_[1], az = acc_lpf_[2];
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-3f) return;
    ax /= n; ay /= n; az /= n;

    float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
    float upx = 2.0f * (x * z - w * y);
    float upy = 2.0f * (y * z + w * x);
    float upz = 1.0f - 2.0f * (x * x + y * y);

    float ex = ay * upz - az * upy;
    float ey = az * upx - ax * upz;
    float ez = ax * upy - ay * upx;

    float wcx = gyro_sensor_[0] - b_hat_[0] + Kp_ * ex;
    float wcy = gyro_sensor_[1] - b_hat_[1] + Kp_ * ey;
    float wcz = gyro_sensor_[2] - b_hat_[2] + Kp_ * ez;
    b_hat_[0] -= Ki_ * ex * dt;
    b_hat_[1] -= Ki_ * ey * dt;
    b_hat_[2] -= Ki_ * ez * dt;

    float qd0 = 0.5f * (-x * wcx - y * wcy - z * wcz);
    float qd1 = 0.5f * ( w * wcx + y * wcz - z * wcy);
    float qd2 = 0.5f * ( w * wcy + z * wcx - x * wcz);
    float qd3 = 0.5f * ( w * wcz + x * wcy - y * wcx);
    q_[0] += qd0 * dt;
    q_[1] += qd1 * dt;
    q_[2] += qd2 * dt;
    q_[3] += qd3 * dt;

    float qn = sqrtf(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    if (qn > 1e-6f) {
        q_[0] /= qn; q_[1] /= qn; q_[2] /= qn; q_[3] /= qn;
    }
}

void IMU::quat2Euler(void) {
    float w = q_[0], x = q_[1], y = q_[2], z = q_[3];

    euler_[0] = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * kRad2Deg;

    float sp = 2.0f * (w * y - z * x);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    euler_[1] = asinf(sp) * kRad2Deg;

    euler_[2] = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * kRad2Deg;
}

void IMU::euler2Quat(void) {
    float r = euler_[0] * kDeg2Rad * 0.5f;
    float p = euler_[1] * kDeg2Rad * 0.5f;
    float y = euler_[2] * kDeg2Rad * 0.5f;
    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);

    q_[0] = cr * cp * cy + sr * sp * sy;
    q_[1] = sr * cp * cy - cr * sp * sy;
    q_[2] = cr * sp * cy + sr * cp * sy;
    q_[3] = cr * cp * sy - sr * sp * cy;
}

uint8_t IMU::init(void) {
    if (bmi088_init() != 1) return 0;

    calibrateGyro(500);

    accCalculate();
    sensorDataTransform();
    float ax = accel_sensor_[0], ay = accel_sensor_[1], az = accel_sensor_[2];
    euler_[0] = 0.0f;
    euler_[1] = atan2f(-ax, sqrtf(ay * ay + az * az)) * kRad2Deg;
    euler_[2] = 0.0f;
    euler2Quat();

    acc_lpf_ready_ = false;
    b_hat_[0] = b_hat_[1] = b_hat_[2] = 0.0f;
    return 1;
}

void IMU::update(float dt) {
    accCalculate();
    gyroCalculate();
    sensorDataTransform();

    if (!acc_lpf_ready_) {
        for (int i = 0; i < 3; i++) acc_lpf_[i] = accel_sensor_[i];
        acc_lpf_ready_ = true;
    } else {
        for (int i = 0; i < 3; i++) {
            acc_lpf_[i] += acc_lpf_k_ * (accel_sensor_[i] - acc_lpf_[i]);
        }
    }

    if (fusion_ == MAHONY) {
        mahonyUpdate(dt);
        quat2Euler();
    } else {
        complementaryUpdate(dt);
    }
}
