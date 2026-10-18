//
// Lesson5-SPI-IMU：姿态解算
//

#ifndef LESSON5_SPI_IMU_IMU_HPP
#define LESSON5_SPI_IMU_IMU_HPP

#include <cstdint>

class IMU {
public:
    enum Fusion : uint8_t {
        COMPLEMENTARY = 0,
        MAHONY        = 1
    };

    IMU();

    uint8_t init(void);
    void update(float dt);

    float roll()  const { return euler_[0]; }
    float pitch() const { return euler_[1]; }
    float yaw()   const { return euler_[2]; }

    const float* accel()    const { return accel_sensor_; }
    const float* gyro()     const { return gyro_sensor_;  }
    const float* rawAccel() const { return raw_accel_;    }
    const float* rawGyro()  const { return raw_gyro_;     }
    const float* quat()     const { return q_;            }

    void setFusion(Fusion f) { fusion_ = f; }
    void setComplementaryK(float k) { if (k >= 0.0f && k <= 1.0f) comp_k_ = k; }
    void setMahonyGains(float kp, float ki) { Kp_ = kp; Ki_ = ki; }
    void setAccelFilterK(float k) { if (k >= 0.0f && k <= 1.0f) acc_lpf_k_ = k; }

private:
    void accCalculate(void);
    void gyroCalculate(void);
    void sensorDataTransform(void);
    void complementaryUpdate(float dt);
    void mahonyUpdate(float dt);
    void quat2Euler(void);
    void euler2Quat(void);
    void calibrateGyro(uint16_t n);

    float raw_accel_[3] = {0};
    float raw_gyro_[3]  = {0};
    float accel_sensor_[3] = {0};
    float gyro_sensor_[3]  = {0};

    float acc_lpf_[3] = {0};
    bool  acc_lpf_ready_ = false;

    float gyro_bias_[3]  = {0};
    float accel_bias_[3] = {0};
    float accel_sen_ = 1.0f;

    float q_[4] = {1, 0, 0, 0};     // w,x,y,z
    float euler_[3] = {0, 0, 0};    // roll,pitch,yaw 度
    float b_hat_[3] = {0};

    Fusion fusion_ = COMPLEMENTARY;
    float comp_k_ = 0.98f;
    float Kp_ = 0.5f;
    float Ki_ = 0.0f;
    float acc_lpf_k_ = 0.1f;

    static const float R_imu_[3][3];
};

#endif // LESSON5_SPI_IMU_IMU_HPP
