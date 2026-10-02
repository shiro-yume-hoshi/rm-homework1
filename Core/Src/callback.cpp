#include "main.h"
#include "usart.h"
#include "remote.h"


extern Remote remote;


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    (void)Size;
    if (huart == &huart3) {
        remote.rxMsgCheck(huart);
        remote.rxMsgCallback(huart->pRxBuffPtr);
    }
}
