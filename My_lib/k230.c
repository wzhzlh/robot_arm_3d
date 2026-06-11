#include "k230.h"
#include "usart.h"
#include "FreeRTOS.h"
#include <string.h>


static uint16_t k230_rx_len = 0;
K230_TargetPosTypeDef k230_rx_buf = {0};
K230_TargetPosTypeDef k230_parse_buf = {0};
K230_TargetPosTypeDef k230_target_pos = {0};
K230_StatusTypeDef k230_comm_status = K230_IDLE;

void K230_UART_Init(void)
{
    HAL_UART_AbortReceive(&huart3);
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)&k230_rx_buf, sizeof(K230_TargetPosTypeDef));
    (void)status;
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void K230_ParseFrame(K230_TargetPosTypeDef *buf, uint16_t len)
{
    if(len < sizeof(K230_TargetPosTypeDef))
    {
        k230_comm_status = K230_RECEIVE_ERROR;
        return;
    }
        k230_target_pos.x = buf->y*0.001f;
        k230_target_pos.y = -(buf->x*0.001f+0.20f);
        k230_target_pos.z =  (buf->z*0.001-0.18f);
        K230_UART_Init();
        k230_comm_status = K230_RECEIVED_OK;

    // else if (buf->a == 2)
    // {
    //     if(len < sizeof(K230_TargetPosTypeDef))
    //     {
    //         k230_comm_status = K230_RECEIVE_ERROR;
    //         return;
    //     }
    //     k230_target_pos.x = -buf->y*0.001f;
    //     k230_target_pos.y = (buf->x*0.001f+0.20f);
    //     k230_target_pos.z = (buf->z*0.001-0.18f);
    //
    // }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if(huart == &huart2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        servo_rx_len = size;
        if(servo_rx_len > SERVO_RX_BUF_LEN)
        {
            servo_rx_len = SERVO_RX_BUF_LEN;
        }
        memcpy(servo_rx_data, servo_rx_buf, servo_rx_len);
        ServoBus_ParseReply();
        HAL_UART_DMAStop(&huart2);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, servo_rx_buf, SERVO_RX_BUF_LEN);
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        xSemaphoreGiveFromISR(servo_rx_sem, &xHigherPriorityTaskWoken);
        xSemaphoreGiveFromISR(servo_rx_reply_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if(huart == &huart3)
    {
        k230_rx_len = size;
        memcpy(&k230_parse_buf, &k230_rx_buf, sizeof(K230_TargetPosTypeDef));
        K230_ParseFrame(&k230_parse_buf, k230_rx_len);
    }
}
