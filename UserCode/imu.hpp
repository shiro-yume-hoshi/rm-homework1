//
// Lesson5-SPI-IMU：BMI088 姿态解算（Mahony 六轴）
//

#ifndef LESSON5_SPI_IMU_IMU_HPP
#define LESSON5_SPI_IMU_IMU_HPP

#include <cstdint>

class IMU {
public:
    IMU();

    // 初始化：bmi088_init -> 读量程 -> 静止校准陀螺零偏 -> 用重力初始化倾角
    // 返回 1 成功，0 失败
    uint8_t init(void);

    // 1kHz 调用：读数据 -> 单位换算 -> 安装方向变换/去偏 -> 低通 -> Mahony -> 欧拉角
    void update(float dt);

    // 欧拉角（度），Z-Y'-X'' 内旋：roll/pitch/yaw
    float roll()  const { return euler_[0]; }
    float pitch() const { return euler_[1]; }
    float yaw()   const { return euler_[2]; }

    const float* accel()    const { return accel_sensor_; }  // 机体系 m/s^2（已去偏）
    const float* gyro()     const { return gyro_sensor_;  }  // 机体系 rad/s（已去偏）
    const float* rawAccel() const { return raw_accel_;    }  // 传感器系 m/s^2（未变换）
    const float* rawGyro()  const { return raw_gyro_;     }  // 传感器系 rad/s（未变换）
    const float* quat()     const { return q_;            }  // w,x,y,z（机体系->世界系）

    void setMahonyGains(float kp, float ki) { Kp_ = kp; Ki_ = ki; }

private:
    void accCalculate(void);       // 读 0x41 量程 + 0x12 六字节，换算成 m/s^2
    void gyroCalculate(void);      // 读 0x0F 量程 + 0x02 六字节，换算成 rad/s
    void sensorDataTransform(void);// 安装方向变换 + 去偏 + 加速度比例修正
    void mahonyUpdate(float dt);   // 加速度计修正 + 四元数更新
    void quat2Euler(void);         // 四元数 -> 欧拉角（度）
    void calibrateGyro(uint16_t n);// 静止采样求陀螺零偏（机体系）

    float raw_accel_[3] = {0};     // 传感器系 m/s^2
    float raw_gyro_[3]  = {0};     // 传感器系 rad/s
    float accel_sensor_[3] = {0};  // 机体系 m/s^2（去偏后）
    float gyro_sensor_[3]  = {0};  // 机体系 rad/s（去偏后）

    float acc_lpf_[3] = {0};       // 加速度计一阶低通结果
    bool  acc_lpf_ready_ = false;

    float gyro_bias_[3]  = {0};    // 静态零偏（机体系，rad/s）
    float accel_bias_[3] = {0};    // 加速度计偏置（机体系，m/s^2）
    float accel_sen_ = 1.0f;       // 加速度比例修正（无量纲，默认 1）
    float acc_lsb_   = 0.001795f;  // 量程换算：m/s^2 per LSB
    float gyro_lsb_  = 0.001065f;  // 量程换算：rad/s per LSB

    float q_[4] = {1, 0, 0, 0};    // w,x,y,z
    float euler_[3] = {0, 0, 0};   // roll,pitch,yaw（度）
    float b_hat_[3] = {0};         // Mahony 在线估计的残余零偏 rad/s

    float Kp_ = 0.5f;              // 比例修正
    float Ki_ = 0.0f;              // 积分（残余零偏）修正
    float acc_lpf_k_ = 0.1f;       // 加速度计低通系数（1kHz 下约 16Hz）

    // 安装方向矩阵：把传感器系读数转到机体系（对应 C 板实例，换安装要改）
    static const float R_imu_[3][3];
};

#endif // LESSON5_SPI_IMU_IMU_HPP
