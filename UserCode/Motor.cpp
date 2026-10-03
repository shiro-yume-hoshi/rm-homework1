//
// Created by shiro2233 on 2026/10/3.
//

#include "Motor.hpp"

Motor::Motor(float ratio) : ratio_(ratio) {}

void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {

    uint16_t raw_ecd = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
    int16_t  raw_rpm = static_cast<int16_t>(
        (static_cast<uint16_t>(rx_data[2]) << 8) | rx_data[3]);
    int16_t  raw_cur = static_cast<int16_t>(
        (static_cast<uint16_t>(rx_data[4]) << 8) | rx_data[5]);
    uint8_t  raw_temp = rx_data[6];


    ecd_angle_ = static_cast<float>(raw_ecd);
    speedRpm_  = static_cast<float>(raw_rpm);
    currentA_  = static_cast<float>(raw_cur) * 20.0f / 16384.0f;
    tempC_     = static_cast<float>(raw_temp);


    if (!received_) {
        last_ecd_ = ecd_angle_;
        received_ = true;
    }

    float delta = ecd_angle_ - last_ecd_;
    if (delta > kEncoderRange / 2.0f) {
        delta -= kEncoderRange;
    } else if (delta < -kEncoderRange / 2.0f) {
        delta += kEncoderRange;
    }
    last_ecd_ = ecd_angle_;


    float delta_deg_rotor = delta * 360.0f / kEncoderRange;
    angle_ += delta_deg_rotor / ratio_;
}

float Motor::angle() const       { return angle_; }
float Motor::speedRpm() const    { return speedRpm_; }
float Motor::currentAmps() const { return currentA_; }
float Motor::temperature() const { return tempC_; }
bool  Motor::hasFeedback() const { return received_; }

void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    if (amperes >  20.0f) amperes =  20.0f;
    if (amperes < -20.0f) amperes = -20.0f;

    int16_t raw = static_cast<int16_t>(amperes * 16384.0f / 20.0f);

    uint8_t idx = (motor_id - 1) * 2;
    if (idx + 1 >= 8) return;
    tx_data_[idx]     = static_cast<uint8_t>((raw >> 8) & 0xFF);
    tx_data_[idx + 1] = static_cast<uint8_t>(raw & 0xFF);
}

uint8_t* Motor::getTxData() {
    return tx_data_;
}
