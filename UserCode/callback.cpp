//
// Created by shiro2233 on 2026/10/3.
//

#include "can_user.h"
#include "Motor.hpp"
#include "tim.h"

Motor motor(19.2f);

constexpr float kModeCurrent[7] = {
    0.0f,   // 0：停
    2.0f,   // 1：正转低速
    4.0f,   // 2：正转中速
    6.0f,   // 3：正转高速
   -2.0f,   // 4：反转低速
   -4.0f,   // 5：反转中速
   -6.0f,   // 6：反转高速
};
volatile uint8_t g_motor_mode = 0;


extern "C" {
volatile uint8_t  g_key_level   = 1;
volatile uint8_t  g_key_pressed = 0;
volatile uint32_t g_key_cnt     = 0;
}

extern "C" void motor_set_current(float amps, uint8_t id) { motor.setTxCurrent(amps, id); }
extern "C" int   motor_has_feedback(void) { return motor.hasFeedback() ? 1 : 0; }
extern "C" float motor_get_angle(void)    { return motor.angle(); }
extern "C" float motor_get_temperature(void) { return motor.temperature(); }
extern "C" uint8_t motor_get_mode(void) { return g_motor_mode; }
extern "C" void motor_set_mode(uint8_t mode) {
    if (mode < 7) {
        g_motor_mode = mode;
    }
}

constexpr uint32_t kMotor1FeedbackId = 0x201;

//用于debug的过程性代码，现在没有用了
//发现视频要提交显示数据，又有用了
//又觉得太乱重新定义了一些
volatile uint32_t g_tim6_cnt   = 0;
volatile uint32_t g_can_tx_ok  = 0;
volatile uint32_t g_can_rx_cnt = 0;
volatile uint32_t g_can_err    = 0;
volatile uint32_t g_rx_id = 0;
volatile uint8_t  g_rx_data[8] = {};
volatile uint16_t g_rx_count = 0;
volatile float    g_tx_current = 0.0f;

//新的
volatile float   g_motor_angle     = 0.0f;
volatile float   g_motor_speed     = 0.0f;
volatile float   g_motor_current   = 0.0f;
volatile float   g_motor_temp      = 0.0f;
volatile uint8_t g_motor_connected = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    uint8_t data[8];
    if (hcan != &hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) return;

    if (rx_header.IDE == CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA &&
        rx_header.DLC == 8 && rx_header.StdId == kMotor1FeedbackId) {
        g_can_rx_cnt++;
        g_rx_id = rx_header.StdId;
        for (int i = 0; i < 8; i++) g_rx_data[i] = data[i];
        g_rx_count++;

        motor.canRxMsgCallback(data);


        g_motor_angle     = motor.angle();
        g_motor_speed     = motor.speedRpm();
        g_motor_current   = motor.currentAmps();
        g_motor_temp      = motor.temperature();
        g_motor_connected = motor.hasFeedback() ? 1 : 0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM6) return;

    g_tx_current = kModeCurrent[g_motor_mode];
    motor.setTxCurrent(kModeCurrent[g_motor_mode], 1);

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, motor.getTxData(), &can_tx_mailbox);
    }
}
