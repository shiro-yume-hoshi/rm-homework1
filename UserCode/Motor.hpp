//
// Created by shiro2233 on 2026/10/3.
//

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>
#include "PID.hpp"

class Motor {
public:
    enum ControlMethod {
        POSITION_SPEED,   // 位置+速度双环
        SPEED,            // 速度环
        TORQUE            // 力矩直接给
    };

    explicit Motor(float ratio);

    void canRxMsgCallback(const uint8_t rx_data[8]);
    void handle();

    void setPosition(float target_angle, float ff_intensity = 0.0f);
    void setSpeed(float target_speed, float ff_intensity = 0.0f);
    void setIntensity(float intensity);
    void setTarget(float target_angle);   // 只改目标角度，不重置 PID（看阶跃响应用）
    void setGains(float pp_kp, float pp_ki, float pp_kd,
                  float sp_kp, float sp_ki, float sp_kd);   // 在线改内外环增益
    void setLimits(float pp_imax, float pp_outmax, float pp_dfilt,
                   float sp_imax, float sp_outmax, float sp_dfilt);   // 在线改限幅/滤波
    void setFfScale(float s) { ff_scale_ = s; }             // 在线改重力前馈系数
    void stop();

    float angle() const       { return angle_; }
    float speedRpm() const    { return speedRpm_; }
    float currentAmps() const { return currentA_; }
    float temperature() const { return tempC_; }
    float targetAngle() const { return target_angle_; }
    float targetSpeed() const { return target_speed_; }
    float intensity() const   { return intensity_; }
    bool  hasFeedback() const { return received_; }
    uint8_t* getTxData()      { return tx_data_; }

    void setTxCurrent(float amperes, uint8_t motor_id);

private:
    const float ratio_;

    // 反馈
    float ecd_angle_ = 0;
    float speedRpm_ = 0;
    float currentA_ = 0;
    float tempC_ = 0;

    // 连续角度
    float angle_ = 0;
    float last_ecd_ = 0;
    bool received_ = false;

    // 待发送
    uint8_t tx_data_[8] = {};

    static constexpr uint16_t kEncoderRange = 8192;

    // PID
    PID ppid_, spid_;
    ControlMethod mode_ = POSITION_SPEED;

    float target_angle_ = 0;
    float target_speed_ = 0;
    float feedforward_intensity_ = 0;
    float ff_scale_ = 1.0f;   // 重力前馈标定系数（可在线上改）

    float fdb_angle_ = 0;
    float fdb_speed_ = 0;
    float intensity_ = 0;

    bool enabled_ = false;
};

#endif
