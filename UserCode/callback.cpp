//
// Created by shiro2233 on 2026/10/3.
//

#include "can_user.h"
#include "Motor.hpp"
#include "tim.h"

Motor motor(19.2f);
extern "C" void motor_set_current(float amps, uint8_t id) {
    motor.setTxCurrent(amps, id);
}

extern "C" int motor_has_feedback(void) {
    return motor.hasFeedback() ? 1 : 0;
}

extern "C" float motor_get_angle(void) {
    return motor.angle();
}

extern "C" float motor_get_temperature(void) {
    return motor.temperature();
}

constexpr uint32_t kMotor1FeedbackId = 0x201;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    uint8_t data[8];
    if (hcan != &hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) return;

    if (rx_header.IDE == CAN_ID_STD &&
        rx_header.RTR == CAN_RTR_DATA &&
        rx_header.DLC == 8 &&
        rx_header.StdId == kMotor1FeedbackId) {
        motor.canRxMsgCallback(data);
        }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM6) return;

    static uint32_t counter = 0;
    counter++;
    if (counter % 1000 == 0) {          // 每 1 秒
        static bool state = false;
        state = !state;
        motor.setTxCurrent(state ? 2.0f : 0.0f, 1);
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, motor.getTxData(), &can_tx_mailbox);
    }
}
