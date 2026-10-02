/**
******************************************************************************
 * @file    remote.cpp/h
 * @brief   Remote control. 遥控器
 ******************************************************************************
 * Copyright (c) 2026 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */



#include "remote.h"
#include <cstring>

// 超久没收，遥控器断开，仍然是ai兼容cpp和c
static const uint16_t REMOTE_CONNECT_TIMEOUT = 500u;

// 构造函数，把 huart 记下来，顺便把数据清一下
Remote::Remote(UART_HandleTypeDef *huart)
    : huart_(huart), connect_(REMOTE_CONNECT_TIMEOUT) {
    std::memset(rx_buf, 0, sizeof(rx_buf));
    reset();
}

// 开始接收。用串口的 DMA 收，串口空下来（一帧发完）的时候会中断一次
void Remote::init() {
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buf, RC_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);   // 收一半不用管，只等一帧收完
}

// 把数据放回中间的位置，摇杆不动的时候都是 1024
void Remote::reset() {
    rc.ch0 = rc.ch1 = rc.ch2 = rc.ch3 = RC_CH_VALUE_OFFSET;
    rc.s1 = rc.s2 = RC_SW_MID;
    mouse.x = mouse.y = mouse.z = 0;
    mouse.press_l = mouse.press_r = 0;
    key.v = 0;
}


void Remote::rxMsgCheck(UART_HandleTypeDef *huart) const {
    (void)huart;
}

// 收到一帧就进来：先存下来，再解包
void Remote::rxMsgCallback(uint8_t* data) {
    std::memcpy(rx_buf, data, RC_FRAME_LENGTH);
    connect_.refresh();   // 能收到就说明还连着
    handle();
    // DMA 是循环收的，不用再重新开
}

// 把 18 个字节拆成通道、拨杆、鼠标、按键
// 公式是照着说明书抄的，就不自己推导了
void Remote::handle() {
    const uint8_t* p = rx_buf;

    rc.ch0 = (uint16_t)(( p[0]        | (p[1] << 8))                    & 0x07FF);
    rc.ch1 = (uint16_t)(((p[1] >> 3)  | (p[2] << 5))                    & 0x07FF);
    rc.ch2 = (uint16_t)(((p[2] >> 6)  | (p[3] << 2) | (p[4] << 10))     & 0x07FF);
    rc.ch3 = (uint16_t)(((p[4] >> 1)  | (p[5] << 7))                    & 0x07FF);

    rc.s1 = (uint8_t)(((p[5] >> 4) & 0x000C) >> 2);
    rc.s2 = (uint8_t)( (p[5] >> 4) & 0x0003);

    mouse.x = (int16_t)((int16_t)p[6]  | ((int16_t)p[7]  << 8));
    mouse.y = (int16_t)((int16_t)p[8]  | ((int16_t)p[9]  << 8));
    mouse.z = (int16_t)((int16_t)p[10] | ((int16_t)p[11] << 8));

    mouse.press_l = p[12];
    mouse.press_r = p[13];

    key.v = (uint16_t)((int16_t)p[14] | ((int16_t)p[15] << 8));
}

// 建一个遥控器对象，用 3 号串口收
Remote remote(&huart3);

// 下面几个函数是给 main.c（C 语言）用的
extern "C" void remote_init(void)         { remote.init(); }
extern "C" int  remote_is_connected(void) { return remote.connect_.check() ? 1 : 0; }
extern "C" int  remote_get_ch0(void)      { return remote.rc.ch0; }
