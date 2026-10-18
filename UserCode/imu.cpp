//
// Lesson5-SPI-IMU：BMI088 姿态解算实现
//

#include "imu.hpp"
#include "bmi088.h"
#include <cmath>

// 安装方向矩阵：传感器系 -> 机体系（C 板实例：xs=-xb, ys=+yb, zs=-zb）
const float IMU::R_imu_[3][3] = {
    {-1.0f, 0.0f,  0.0f},
    { 0.0f, 1.0f,  0.0f},
    { 0.0f, 0.0f, -1.0f}
};

// 量程 -> 最小分辨率
// ACC_RANGE(0x41): 0=±3g 1=±6g 2=±12g 3=±24g，单位 m/s^2 per LSB
static const float kAccLsb[4] = {0.000898f, 0.001795f, 0.003590f, 0.007180f};
// GYRO_RANGE(0x0F): 0=±2000 1=±1000 2=±500 3=±250 4=±125 dps，单位 rad/s per LSB
static const float kGyroLsb[5] = {0.001065f, 0.0005325f, 0.000266f, 0.000133f, 0.0000665f};

IMU::IMU() = default;

// ---------------- 底层解包 ----------------

// 读加速度计：量程 + 0x12 六字节。输出 raw_accel_[3]（传感器系 m/s^2）
void IMU::accCalculate(void) {
    uint8_t range = 0;
    // 1) 读量程，换算成 m/s^2 per LSB
    bmi088_accel_read_reg(0x41, &range, 1);
    acc_lsb_ = kAccLsb[range & 0x03];

    // 2) 连续读 0x12~0x17：X/Y/Z 各 2 字节，小端（低字节在前）
    uint8_t d[6] = {0};
    bmi088_accel_read_reg(0x12, d, 6);
    int16_t rx = (int16_t)((d[1] << 8) | d[0]);
    int16_t ry = (int16_t)((d[3] << 8) | d[2]);
    int16_t rz = (int16_t)((d[5] << 8) | d[4]);

    // 3) 换算成 m/s^2
    raw_accel_[0] = (float)rx * acc_lsb_;
    raw_accel_[1] = (float)ry * acc_lsb_;
    raw_accel_[2] = (float)rz * acc_lsb_;
}

// 读陀螺仪：量程 + 0x02 六字节。输出 raw_gyro_[3]（传感器系 rad/s）
void IMU::gyroCalculate(void) {
    uint8_t range = 0;
    bmi088_gyro_read_reg(0x0F, &range, 1);
    uint8_t r = range & 0x07;
    if (r > 4) r = 0;                 // 保留值按 ±2000 处理
    gyro_lsb_ = kGyroLsb[r];

    uint8_t d[6] = {0};
    bmi088_gyro_read_reg(0x02, d, 6);
    int16_t rx = (int16_t)((d[1] << 8) | d[0]);
    int16_t ry = (int16_t)((d[3] << 8) | d[2]);
    int16_t rz = (int16_t)((d[5] << 8) | d[4]);

    raw_gyro_[0] = (float)rx * gyro_lsb_;
    raw_gyro_[1] = (float)ry * gyro_lsb_;
    raw_gyro_[2] = (float)rz * gyro_lsb_;
}

// 安装方向变换 + 去偏：v^b = R_imu * v^s - bias
void IMU::sensorDataTransform(void) {
    for (int i = 0; i < 3; i++) {
        gyro_sensor_[i] = raw_gyro_[0] * R_imu_[i][0] +
                          raw_gyro_[1] * R_imu_[i][1] +
                          raw_gyro_[2] * R_imu_[i][2] - gyro_bias_[i];

        accel_sensor_[i] = accel_sen_ * (raw_accel_[0] * R_imu_[i][0] +
                                         raw_accel_[1] * R_imu_[i][1] +
                                         raw_accel_[2] * R_imu_[i][2]) - accel_bias_[i];
    }
}

// 静止采样求陀螺零偏（方向变换后、去偏前求平均）
void IMU::calibrateGyro(uint16_t n) {
    if (n == 0) return;
    float sum[3] = {0, 0, 0};
    for (uint16_t k = 0; k < n; k++) {
        gyroCalculate();
        for (int i = 0; i < 3; i++) {
            sum[i] += raw_gyro_[0] * R_imu_[i][0] +
                      raw_gyro_[1] * R_imu_[i][1] +
                      raw_gyro_[2] * R_imu_[i][2];
        }
        HAL_Delay(2);
    }
    for (int i = 0; i < 3; i++) gyro_bias_[i] = sum[i] / (float)n;
    b_hat_[0] = b_hat_[1] = b_hat_[2] = 0.0f;
}

// ---------------- 姿态解算 ----------------

// Mahony：用加速度计方向误差修正角速度，再更新四元数
void IMU::mahonyUpdate(float dt) {
    // 1) 加速度计（低通后）归一化，作为测量方向
    float ax = acc_lpf_[0], ay = acc_lpf_[1], az = acc_lpf_[2];
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-3f) return;             // 自由落体/不可信，跳过修正
    ax /= n; ay /= n; az /= n;

    // 2) 由当前四元数预测机体系下的重力方向 u_p = R^T * (0,0,1)
    float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
    float upx = 2.0f * (x * z - w * y);
    float upy = 2.0f * (y * z + w * x);
    float upz = 1.0f - 2.0f * (x * x + y * y);

    // 3) 方向误差 e = u_m × u_p
    float ex = ay * upz - az * upy;
    float ey = az * upx - ax * upz;
    float ez = ax * upy - ay * upx;

    // 4) 修正角速度，并在线估计残余零偏
    float wcx = gyro_sensor_[0] - b_hat_[0] + Kp_ * ex;
    float wcy = gyro_sensor_[1] - b_hat_[1] + Kp_ * ey;
    float wcz = gyro_sensor_[2] - b_hat_[2] + Kp_ * ez;

    b_hat_[0] -= Ki_ * ex * dt;
    b_hat_[1] -= Ki_ * ey * dt;
    b_hat_[2] -= Ki_ * ez * dt;

    // 5) 四元数微分：q_dot = 0.5 * q ⊗ (0, w_c)（四个分量都用旧 q 计算）
    float qd0 = 0.5f * (-x * wcx - y * wcy - z * wcz);
    float qd1 = 0.5f * ( w * wcx + y * wcz - z * wcy);
    float qd2 = 0.5f * ( w * wcy + z * wcx - x * wcz);
    float qd3 = 0.5f * ( w * wcz + x * wcy - y * wcx);
    q_[0] += qd0 * dt;
    q_[1] += qd1 * dt;
    q_[2] += qd2 * dt;
    q_[3] += qd3 * dt;

    // 6) 归一化
    float qn = sqrtf(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
    if (qn > 1e-6f) {
        q_[0] /= qn; q_[1] /= qn; q_[2] /= qn; q_[3] /= qn;
    }
}

// 四元数 -> 欧拉角（Z-Y'-X''，度）
void IMU::quat2Euler(void) {
    float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
    const float RAD2DEG = 57.2957795f;

    // roll
    euler_[0] = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD2DEG;

    // pitch（clamp 防 asin 域外）
    float sp = 2.0f * (w * y - z * x);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    euler_[1] = asinf(sp) * RAD2DEG;

    // yaw
    euler_[2] = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD2DEG;
}

// ---------------- 对外接口 ----------------

uint8_t IMU::init(void) {
    if (bmi088_init() != 1) return 0;

    // 量程已由 bmi088_init 配好，这里读一次建立换算系数
    uint8_t r = 0;
    bmi088_accel_read_reg(0x41, &r, 1);
    acc_lsb_ = kAccLsb[r & 0x03];
    bmi088_gyro_read_reg(0x0F, &r, 1);
    uint8_t gr = r & 0x07;
    if (gr > 4) gr = 0;
    gyro_lsb_ = kGyroLsb[gr];

    // 静止校准陀螺零偏（约 1s）
    calibrateGyro(500);

    // 用重力初始化倾角（yaw 置 0）
    accCalculate();
    sensorDataTransform();             // 此时 gyro_bias_ 已扣，accel_bias_ 仍为 0
    float ax = accel_sensor_[0], ay = accel_sensor_[1], az = accel_sensor_[2];
    float roll0  = atan2f(ay, az);
    float pitch0 = atan2f(-ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll0 * 0.5f), sr = sinf(roll0 * 0.5f);
    float cp = cosf(pitch0 * 0.5f), sp = sinf(pitch0 * 0.5f);
    // yaw=0 时 q = qy(pitch)*qx(roll)
    q_[0] = cr * cp;
    q_[1] = sr * cp;
    q_[2] = cr * sp;
    q_[3] = -sr * sp;

    acc_lpf_ready_ = false;
    b_hat_[0] = b_hat_[1] = b_hat_[2] = 0.0f;
    quat2Euler();
    return 1;
}

void IMU::update(float dt) {
    accCalculate();
    gyroCalculate();
    sensorDataTransform();

    // 加速度计一阶低通
    if (!acc_lpf_ready_) {
        acc_lpf_[0] = accel_sensor_[0];
        acc_lpf_[1] = accel_sensor_[1];
        acc_lpf_[2] = accel_sensor_[2];
        acc_lpf_ready_ = true;
    } else {
        acc_lpf_[0] += acc_lpf_k_ * (accel_sensor_[0] - acc_lpf_[0]);
        acc_lpf_[1] += acc_lpf_k_ * (accel_sensor_[1] - acc_lpf_[1]);
        acc_lpf_[2] += acc_lpf_k_ * (accel_sensor_[2] - acc_lpf_[2]);
    }

    mahonyUpdate(dt);
    quat2Euler();
}
