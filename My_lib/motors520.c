
//#include "usart.h"

//#include "main.h"
//#include <string.h>


//#define TX_BUF_SIZE 200

//static uint8_t tx_buf[TX_BUF_SIZE];
//static volatile uint16_t tx_len = 0;
//static volatile uint16_t tx_index = 0;
//static volatile uint8_t tx_busy = 0;

//static uint8_t rx_byte;   // 接收缓冲

//void Motor_Usart_Init(void)
//{
//    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
//}

///* 内部启动下一个字节发送 */
//static void start_tx_byte(void)
//{
//    if (tx_index < tx_len) {
//        HAL_UART_Transmit_IT(&huart2, &tx_buf[tx_index], 1);
//    } else {
//        tx_busy = 0;
//        tx_len = 0;
//        tx_index = 0;
//    }
//}

///* 用户调用：发送一帧数据 */
//void Send_Motor_Array(uint8_t *pData, uint16_t Length)
//{
//    if (Length == 0 || Length > TX_BUF_SIZE) return;

//    // 等待上一次发送完成，带超时保护
//    uint32_t timeout = 0xFFFFF;
//    while (tx_busy) {
//        if (--timeout == 0) {
//            tx_busy = 0;
//            HAL_UART_AbortTransmit_IT(&huart2);
//            break;
//        }
//    }

//    memcpy(tx_buf, pData, Length);
//    tx_len = Length;
//    tx_index = 0;
//    tx_busy = 1;
//    start_tx_byte();
//}

///* 发送完成回调 */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart2) {
//        tx_index++;
//        start_tx_byte();
//    }
//}

///* 接收完成回调 - 先重启接收再处理数据，避免丢失 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart2) {
//        uint8_t data = rx_byte;
//        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);  // 立刻准备收下一个
//        Deal_Control_Rxtemp(data);                  // 再处理本次数据
//    }
//}

//// 错误处理
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart2) {
//        HAL_UART_AbortReceive_IT(&huart2);
//        HAL_UART_AbortTransmit_IT(&huart2);

//        // 强制复位发送状态
//        if (tx_busy) {
//            tx_busy = 0;
//            tx_len = 0;
//            tx_index = 0;
//        }

//        // 重启接收
//        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
//    }
//}

