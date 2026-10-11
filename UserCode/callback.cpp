//
// Created by shiro2233 on 2026/10/3.
//

#include "can_user.h"
#include "Motor.hpp"
#include "tim.h"
#include <cmath>

// M2006 + C610，减速比 36.1
Motor motor(36.1f);

// 调试用全局变量
volatile uint8_t stop_flag = 0;   /* 1 = 电机输出立刻变 0 */
volatile float   g_tx_current = 0;
volatile float   g_fdb_angle = 0;
volatile float   g_fdb_speed = 0;
volatile uint32_t g_tim6_cnt   = 0;   // TIM6 中断次数
volatile uint32_t g_can_rx_cnt = 0;   // 收到 0x201 反馈次数（>0 = 通信正常）
volatile uint32_t g_can_tx_ok  = 0;   // CAN 发送成功次数

// ====== 给 CubeMonitor 抓的曲线变量 ======
volatile float   g_target_angle = 90.0f;  // 当前目标角度
volatile float   g_target_speed = 0;      // 位置环算出的目标速度
volatile float   g_intensity    = 0;      // 最终控制量（含前馈）

// ====== 在线可调参数（在 Scope 的 Tuning Pad 里直接改，实时生效）======
volatile float g_kp_pp    = 350.0f; // 位置环 Kp   （调好）
volatile float g_ki_pp    = 0.4f;   // 位置环 Ki   （调好）
volatile float g_kd_pp    = 10.0f;  // 位置环 Kd   （调好）
volatile float g_kp_sp    = 4.0f;   // 速度环 Kp   （调好）
volatile float g_ki_sp    = 0.0f;   // 速度环 Ki
volatile float g_kd_sp    = 0.05f;  // 速度环 Kd
volatile float g_ff_scale = 1.0f;   // 重力前馈系数（前馈不够就调大）
// 限幅 / 滤波（Ki 必须先给 i_max，否则积分被钳 0 不生效）
volatile float g_imax_pp   = 170.0f;    // 位置环积分限幅
volatile float g_outmax_pp = 1100.0f;   // 位置环输出限幅(=目标速度上限 rpm)
volatile float g_dfilt_pp  = 0.1f;      // 位置环 D 滤波
volatile float g_imax_sp   = 0.0f;      // 速度环积分限幅
volatile float g_outmax_sp = 10000.0f;  // 速度环输出限幅(=控制量上限)
volatile float g_dfilt_sp  = 0.1f;      // 速度环 D 滤波

// 10° 阶跃：每 1.5s 在 85° / 95° 之间跳一次（1kHz，1500 个 tick = 1.5s）
#define STEP_ENABLE     1      // 1 = 自动打 10° 阶跃（看响应/调参用）；0 = 保持固定目标
#define STEP_HALF_TICKS 1500u
volatile uint32_t g_step_state = 0xFF;    // 当前处在阶跃的高/低段

constexpr uint32_t kMotor1FeedbackId = 0x201;

// ====== 给 main.c 用的 C 接口 ======
extern "C" void motor_set_position(float angle, float ff) {
    motor.setPosition(angle, ff);
}

extern "C" void motor_set_speed(float speed, float ff) {
    motor.setSpeed(speed, ff);
}

extern "C" void motor_stop(void) {
    motor.stop();
}

extern "C" void motor_set_stop_flag(uint8_t v) {
    stop_flag = v;
}

extern "C" void motor_set_target(float angle) {
    motor.setTarget(angle);
}

extern "C" int motor_has_feedback(void) {
    return motor.hasFeedback() ? 1 : 0;
}

// ====== CAN 接收回调 ======
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    uint8_t data[8];
    if (hcan != &hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) return;

    if (rx_header.IDE == CAN_ID_STD &&
        rx_header.RTR == CAN_RTR_DATA &&
        rx_header.DLC == 8 &&
        rx_header.StdId == kMotor1FeedbackId) {
        g_can_rx_cnt++;
        motor.canRxMsgCallback(data);
        }
}

// ====== TIM6 控制周期（1kHz）======
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM6) return;
    g_tim6_cnt++;

    if (stop_flag) {
        motor.stop();
    } else {
        // 每周期把在线参数写进 PID（Tuning Pad 一改，下一毫秒就生效）
        motor.setGains(g_kp_pp, g_ki_pp, g_kd_pp, g_kp_sp, g_ki_sp, g_kd_sp);
        motor.setLimits(g_imax_pp, g_outmax_pp, g_dfilt_pp,
                        g_imax_sp, g_outmax_sp, g_dfilt_sp);
        motor.setFfScale(g_ff_scale);
#if STEP_ENABLE
        // 10° 阶跃：每半周期切换 85°/95°，改目标但不重置 PID
        uint32_t half = (g_tim6_cnt / STEP_HALF_TICKS) & 1u;
        if (half != g_step_state) {
            g_step_state = half;
            motor.setTarget(half ? 95.0f : 85.0f);
        }
#endif
        motor.handle();
    }

    g_tx_current   = motor.currentAmps();
    g_fdb_angle    = motor.angle();
    g_fdb_speed    = motor.speedRpm();
#if STEP_ENABLE
    g_target_angle = g_step_state ? 95.0f : 85.0f;
#else
    g_target_angle = motor.targetAngle();
#endif
    g_target_speed = motor.targetSpeed();
    g_intensity    = motor.intensity();

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, motor.getTxData(), &can_tx_mailbox) == HAL_OK) {
            g_can_tx_ok++;
        }
    }
}