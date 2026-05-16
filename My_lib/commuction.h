#ifndef COMMUCTION_H
#define COMMUCTION_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"
// ==================== 众灵舵机 硬件参数 ====================
#define SERVO_TX_TIMEOUT     100       // 发送超时(ms)
#define SERVO_BAUDRATE       115200    // 波特率
#define SERVO_MAX_ID         254       // 最大舵机ID
#define SERVO_POS_MIN        500       // 最小PWM
#define SERVO_POS_MAX        2500      // 最大PWM
#define SERVO_TIME_MIN       0         // 最小运行时间
#define SERVO_TIME_MAX       9999      // 最大运行时间


// ==================== 新增：解析后存储的舵机数据 ====================
extern uint8_t  g_servo_id ;         // 反馈的舵机ID
extern uint16_t g_servo_pwm;        // 反馈的PWM值
extern uint8_t  g_servo_reply_ok;   // 指令执行成功标志
#define SERVO_RX_BUF_LEN 32
extern uint8_t servo_rx_buf[SERVO_RX_BUF_LEN];  // DMA原始接收缓存
extern uint8_t servo_rx_data[SERVO_RX_BUF_LEN]; // 解析用缓存
extern uint16_t servo_rx_len ;   
_Pragma("pack(1)") // 字节对齐，确保结构体紧凑存储
// ==================== 坐标结构体 ====================
typedef struct{
  double x;
	double y;
	double z;
}state_t;

typedef struct{
  double x;
	double y;
	double z;
}target_t;

typedef struct {
  uint8_t id;
  uint16_t motor_pos; // 关节角度值
  float   offset;    // 偏置校准
}motor_t;
// ==================== 机械臂/舵机总线结构体 ====================
typedef struct
{
    target_t target_pos;   // 目标坐标
    state_t  state_pos;    // 当前状态坐标
    uint16_t target_time;  // 运行时间(ms)
    motor_t motor[3];    // 3个关节的舵机信息
} ServoBus_t;

_Pragma("pack()") // 恢复默认对齐

// ==================== 函数声明 ====================
HAL_StatusTypeDef ServoBus_SendCmd(const char *cmd);
HAL_StatusTypeDef ServoBus_Move_One(ServoBus_t *servo);
HAL_StatusTypeDef ServoBus_Move_Many(ServoBus_t *servos, uint8_t count);
HAL_StatusTypeDef ServoBus_ReadAngle(uint8_t id);
HAL_StatusTypeDef ServoBus_SetID(uint8_t old_id, uint8_t new_id);
HAL_StatusTypeDef ServoBus_Unlock(uint8_t id);
HAL_StatusTypeDef ServoBus_Lock(uint8_t id);
void ServoBus_ParseReply();
void ServoBus_Start_Receive(void);

#endif