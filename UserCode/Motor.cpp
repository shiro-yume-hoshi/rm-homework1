//
// Created by shiro2233 on 2026/10/3.
//

#include "Motor.hpp"
#include <cmath>

Motor::Motor(float ratio)
    : ratio_(ratio),
      ppid_(350.0f, 0.4f, 10.0f, 170.0f, 1100.0f, 0.1f),  // 位置环（调好）
      spid_(4.0f, 0.0f, 0.05f, 0.0f, 10000.0f, 0.1f) {   // 速度环（调好）
}

void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    // 拆字节（大端 + 符号）
    uint16_t raw_ecd = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
    int16_t  raw_rpm = static_cast<int16_t>(
        (static_cast<uint16_t>(rx_data[2]) << 8) | rx_data[3]);
    int16_t  raw_cur = static_cast<int16_t>(
        (static_cast<uint16_t>(rx_data[4]) << 8) | rx_data[5]);
    uint8_t  raw_temp = rx_data[6];

    // 物理量换算（M2006 反馈格式与 M3508 相同）
    ecd_angle_ = static_cast<float>(raw_ecd);
    speedRpm_  = static_cast<float>(raw_rpm);
    currentA_  = static_cast<float>(raw_cur) * 10.0f / 10000.0f;  // M2006 是 10A/10000
    tempC_     = static_cast<float>(raw_temp);

    // 环绕修正 + 累加
    if (!received_) {
        last_ecd_ = ecd_angle_;
        received_ = true;
    }

    float delta = ecd_angle_ - last_ecd_;
    if (delta > kEncoderRange / 2.0f)  delta -= kEncoderRange;
    if (delta < -kEncoderRange / 2.0f) delta += kEncoderRange;
    last_ecd_ = ecd_angle_;

    // 折到输出轴，减速比 36.1
    angle_ += delta * 360.0f / kEncoderRange / ratio_;
}

void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    // M2006 + C610 最大 ±10A
    if (amperes > 10.0f)  amperes = 10.0f;
    if (amperes < -10.0f) amperes = -10.0f;

    // 控制量：-10000 ~ 10000 对应 -10A ~ +10A
    int16_t raw = static_cast<int16_t>(amperes * 1000.0f);

    uint8_t idx = (motor_id - 1) * 2;
    if (idx + 1 >= 8) return;
    tx_data_[idx]     = static_cast<uint8_t>((raw >> 8) & 0xFF);
    tx_data_[idx + 1] = static_cast<uint8_t>(raw & 0xFF);
}

void Motor::setPosition(float target_angle, float ff_intensity) {
    mode_ = POSITION_SPEED;
    target_angle_ = target_angle;
    feedforward_intensity_ = ff_intensity;
    enabled_ = true;
    ppid_.reset();
    spid_.reset();
}

void Motor::setTarget(float target_angle) {
    // 只改目标角度，不动 PID 的状态，方便做阶跃
    mode_ = POSITION_SPEED;
    target_angle_ = target_angle;
    enabled_ = true;
}

void Motor::setGains(float pp_kp, float pp_ki, float pp_kd,
                     float sp_kp, float sp_ki, float sp_kd) {
    ppid_.setGains(pp_kp, pp_ki, pp_kd);
    spid_.setGains(sp_kp, sp_ki, sp_kd);
}

void Motor::setLimits(float pp_imax, float pp_outmax, float pp_dfilt,
                      float sp_imax, float sp_outmax, float sp_dfilt) {
    ppid_.setLimits(pp_imax, pp_outmax, pp_dfilt);
    spid_.setLimits(sp_imax, sp_outmax, sp_dfilt);
}

void Motor::setSpeed(float target_speed, float ff_intensity) {
    mode_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = ff_intensity;
    enabled_ = true;
    spid_.reset();
}

void Motor::setIntensity(float intensity) {
    mode_ = TORQUE;
    feedforward_intensity_ = intensity;
    enabled_ = true;
}

void Motor::stop() {
    enabled_ = false;
    ppid_.reset();
    spid_.reset();
    setTxCurrent(0.0f, 1);
}

void Motor::handle() {
    if (!enabled_) {
        setTxCurrent(0.0f, 1);
        return;
    }

    fdb_angle_ = angle_;
    fdb_speed_ = speedRpm_;

    if (mode_ == POSITION_SPEED) {
        // 位置环：输出目标速度
        target_speed_ = ppid_.calc(target_angle_, fdb_angle_);
        // 速度环：输出控制量
        intensity_ = spid_.calc(target_speed_, fdb_speed_);
    } else if (mode_ == SPEED) {
        intensity_ = spid_.calc(target_speed_, fdb_speed_);
    } else {
        intensity_ = 0;
    }

    // 重力前馈：手臂有重量会往下掉，得提前给点力顶住
    // 2006 教具：重物 0.1kg，力臂 0.065m；电机转矩常数 0.18 N·m/A
    // 上电时手臂垂在下面，所以 0° = 竖直向下：竖直不用力，水平最费劲，用 sin
    const float kLoadMass = 0.1f;    // 重物质量 kg
    const float kArmLen   = 0.065f;  // 力臂 m
    const float kTorqueKt = 0.18f;   // M2006 转矩常数 N·m/A
    float rad = fdb_angle_ * 3.1415926f / 180.0f;
    float ff_gravity = (kLoadMass * 9.8f * kArmLen / kTorqueKt) * ff_scale_ * 1000.0f * sinf(rad);
    intensity_ += ff_gravity + feedforward_intensity_;

    // 控制量换算成电流：10000 对应 10A
    float amps = intensity_ * 10.0f / 10000.0f;
    setTxCurrent(amps, 1);
}