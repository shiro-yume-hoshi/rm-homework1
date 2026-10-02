#include "main.h"
#include "usart.h"

extern uint8_t rx_msg[32];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        // 判断事件类型，只处理 IDLE 或 TC，过滤掉 Half-Complete
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE ||
            HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC)
        {
            // 把收到的 Size 个字节发回去
            HAL_UART_Transmit_IT(&huart1, rx_msg, Size);

            // 重新开启接收
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_msg, 32);
        }
    }
}