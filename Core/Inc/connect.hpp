/**
******************************************************************************
 * @file    connect.hpp
 * @brief   Connect state handle. 连接状态处理
 ******************************************************************************
 * Copyright (c) 2026 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef UART_CONNTCT_HPP
#define UART_CONNTCT_HPP
#include "usart.h"

class Connect {
    uint32_t last_tick_;
    uint32_t start_tick_;
    const uint32_t timeout_;
    float freq_;
    bool is_connected_;
    
public:
    Connect(uint32_t timeout)
      :  last_tick_(0), timeout_(timeout), freq_(0), is_connected_(false) {}
    // Check connect status. Return true-connected, false-unconnected
    // 检测连接状态
    bool check(void) {
        if (HAL_GetTick() - last_tick_ > timeout_) {
            is_connected_ = false;
        }
        return is_connected_;
    }
    
    // Refresh connect status(Commonly called in rx callback).
    // Return true-unconnected->connected, false-keep connected
    // 刷新连接状态(一般在接收回调函数调用)
    // 返回 true-未连接->连接，false-保持连接
    void refresh(void) {
        if (!is_connected_) {
            is_connected_ = true;
            start_tick_ = HAL_GetTick();
        } 
        if (HAL_GetTick() - last_tick_ < 1) {
            freq_ = 1e3f;
        } else {
            freq_ = 1e3f / (HAL_GetTick() - last_tick_);
        }
        last_tick_ = HAL_GetTick();
    }
    
    
};

#endif //UART_CONNTCT_HPP
