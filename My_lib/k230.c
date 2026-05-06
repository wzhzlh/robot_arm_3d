#include "k230.h"
#include "usart.h"

// ==================== 接收缓冲区（DMA专用） ====================
static uint8_t k230_rx_buf[K230_RX_BUF_LEN];   // DMA原始接收缓存
static uint8_t k230_parse_buf[K230_RX_BUF_LEN];// 解析用缓存
static uint16_t k230_rx_len = 0;               // 本次接收数据长度

// ==================== 全局变量定义 ====================
K230_TargetPosTypeDef k230_target_pos = {0};
K230_StatusTypeDef k230_comm_status = K230_IDLE;

/**
 * @brief  K230串口初始化（启动DMA+空闲中断接收）
 */
void K230_UART_Init(void)
{
    // 官方API：开启DMA接收，空闲中断触发HAL_UARTEx_RxEventCallback
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, k230_rx_buf, K230_RX_BUF_LEN);
    // 关闭半满中断，避免干扰空闲中断
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

/**
 * @brief  发送数据到K230（阻塞发送，如需DMA发送可改）
 */
void K230_SendData(uint8_t *data, uint8_t len)
{
    if(data == NULL || len == 0) return;
    HAL_UART_Transmit(&huart3, data, len, 100); // 超时100ms
}

/**
 * @brief  解析K230帧数据（校验帧头/帧尾/CRC）
 */
void K230_ParseFrame(uint8_t *buf, uint16_t len)
{
    // 最小帧长度：头(1)+长度(1)+数据(0)+CRC(2)+尾(1) = 5字节
    if(len < 5) 
    {
        k230_comm_status = K230_RECEIVE_ERROR;
        return;
    }

    // 1. 校验帧头和帧尾
    if(buf[0] != K230_FRAME_HEAD || buf[len-1] != K230_FRAME_TAIL)
    {
        k230_comm_status = K230_RECEIVE_ERROR;
        return;
    }

    // 2. 校验数据长度
    uint8_t data_len = buf[1];
    if(data_len > K230_MAX_DATA_LEN || (len != (data_len + 5)))
    {
        k230_comm_status = K230_RECEIVE_ERROR;
        return;
    }

    // 3. 校验CRC（帧尾前2字节为CRC值）
    uint16_t received_crc = (buf[len-3] << 8) | buf[len-2];
    uint16_t calculated_crc = crc_ccitt(0, (uint8_t*)buf, len-3); // 计算到数据结束

    if(received_crc != calculated_crc)
    {
        k230_comm_status = K230_RECEIVE_ERROR;
        return;
    }

    // 4. CRC校验通过，解析目标坐标
    if(data_len == sizeof(K230_TargetPosTypeDef))
    {
        // 数据部分：buf[2] ~ buf[2+data_len-1]
        memcpy(&k230_target_pos, &buf[2], data_len);
        k230_target_pos.x += 10.0f;  // X 偏移
        k230_target_pos.y += 0.0f;   // Y 偏移
        k230_target_pos.z += 5.0f;   // Z 偏移
        //相对于摄像头坐标改为相对于机械臂基座坐标
        k230_comm_status = K230_RECEIVED_OK;
    }
    else
    {
        k230_comm_status = K230_RECEIVE_ERROR;
    }
}

/**
 * @brief  HAL库UART接收事件回调（空闲中断自动触发）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    // 只处理K230对应的UART3
    if(huart == &huart3)
    {
        k230_rx_len = size;
        // 拷贝数据到解析缓冲区
        memcpy(k230_parse_buf, k230_rx_buf, k230_rx_len);
        // 解析帧数据
        K230_ParseFrame(k230_parse_buf, k230_rx_len);
        // 重启DMA接收，等待下一帧数据
        K230_UART_Init();
    }
}