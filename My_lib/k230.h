#ifndef K230_H
#define K230_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "crc_ccitt.h"  // 你已有的CRC校验库
#include "drive.h"      // 包含你定义的target_t结构体
#include "commuction.h"
// ==================== 帧格式定义（和K230约定好） ====================
#define K230_FRAME_HEAD     0xAA    // 帧头
#define K230_FRAME_TAIL     0x55    // 帧尾
#define K230_MAX_DATA_LEN   16      // 最大数据长度（目标坐标x/y/z共12字节）
#define K230_RX_BUF_LEN     32      // DMA接收缓冲区长度

// ==================== 通信状态标志 ====================
typedef enum {
    K230_IDLE = 0,
    K230_RECEIVING,
    K230_RECEIVED_OK,   // 接收成功（可直接用数据）
    K230_RECEIVE_ERROR  // 接收失败（帧格式/CRC错误）
} K230_StatusTypeDef;

// ==================== K230发送给STM32的目标数据结构体 ====================
// 直接对应你机械臂的目标坐标
typedef struct {
    float x;
    float y;
    float z;
} K230_TargetPosTypeDef;

// ==================== 全局变量（供外部调用） ====================
extern K230_TargetPosTypeDef k230_target_pos;  // 解析后的目标坐标
extern K230_StatusTypeDef k230_comm_status;    // 通信状态

// ==================== 函数声明 ====================
/**
 * @brief  初始化K230串口（UART3），启动DMA+空闲中断接收
 */
void K230_UART_Init(void);

/**
 * @brief  发送数据到K230（阻塞发送，可改DMA）
 * @param  data: 数据缓冲区
 * @param  len:  数据长度
 */
void K230_SendData(uint8_t *data, uint8_t len);

/**
 * @brief  解析K230发送的帧数据（校验帧头/帧尾/CRC）
 * @param  buf: 接收数据缓冲区
 * @param  len: 接收数据长度
 */
void K230_ParseFrame(uint8_t *buf, uint16_t len);

#endif