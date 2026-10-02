#include "main.h"
#include "usart.h"

extern uint8_t rx_msg[1];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        if (rx_msg[0] == 'R')
        {
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); // 灭
        }
        else if (rx_msg[0] == 'M')
        {
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); // 亮
        }

        // 重新开启接收，否则只能收一次
        HAL_UART_Receive_IT(&huart1, rx_msg, 1);
    }
}