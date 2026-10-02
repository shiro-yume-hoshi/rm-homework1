// 遥控器接收的头文件
// 这个是按遥控器说明书写的，接收机发出来的是 DBUS 格式的
// 一帧有 18 个字节，大概 14 毫秒发一帧
// 串口参数：100000 波特率、8 位数据、偶校验、1 位停止位
// 使用AI修改帮我兼容cpp和c的兼容问题

#ifndef UART_REMOTE_H
#define UART_REMOTE_H

#include "connect.hpp"
#include "usart.h"

// 一帧有多少个字节
#define RC_FRAME_LENGTH     ((uint16_t)18)
// 存收到的数据，开大一点放两帧，DMA 转着收的时候不容易乱
#define RC_RX_BUF_SIZE      ((uint16_t)36)

// 摇杆数值：最小 364，中间 1024，最大 1684
#define RC_CH_VALUE_MIN      ((uint16_t)364)
#define RC_CH_VALUE_OFFSET   ((uint16_t)1024)
#define RC_CH_VALUE_MAX      ((uint16_t)1684)

// 拨杆的位置：1 是上，2 是下，3 是中间
#define RC_SW_UP             ((uint8_t)1)
#define RC_SW_MID            ((uint8_t)3)
#define RC_SW_DOWN           ((uint8_t)2)

class Remote {
    UART_HandleTypeDef *huart_;
    uint8_t rx_buf[RC_RX_BUF_SIZE];

public:
    Connect connect_;

    // 从接收机数据里解出来的东西
    struct {
        uint16_t ch0, ch1, ch2, ch3;
        uint8_t  s1, s2;
    } rc;

    struct {
        int16_t x, y, z;
        uint8_t press_l, press_r;
    } mouse;

    struct {
        uint16_t v;
    } key;

    Remote(UART_HandleTypeDef *huart);

    void init(void);
    void reset(void);
    void rxMsgCallback(uint8_t* data);
    void rxMsgCheck(UART_HandleTypeDef* huart) const;
    void handle(void);                 
};

#endif //UART_REMOTE_H
