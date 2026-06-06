#ifndef COMMUCTION_H
#define COMMUCTION_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

#define SERVO_RX_BUF_LEN 32u
#define SERVO_TIME_MIN 0u

extern SemaphoreHandle_t servo_tx_sem;
extern SemaphoreHandle_t servo_rx_sem;
extern SemaphoreHandle_t servo_rx_reply_sem;

extern uint8_t g_servo_id;
extern uint16_t g_servo_pwm;
extern uint8_t g_servo_reply_ok;
extern volatile uint8_t servo_error_pending;

extern uint8_t servo_rx_buf[SERVO_RX_BUF_LEN];
extern uint8_t servo_rx_data[SERVO_RX_BUF_LEN];
extern uint16_t servo_rx_len;

#pragma pack(push, 1)

typedef struct {
    double x;
    double y;
    double z;
} target_t;

typedef struct {
    uint8_t id;
    uint16_t motor_tx_pos;
    uint16_t motor_rx_pos;
    float offset;
} motor_t;

typedef struct {
    target_t target_pos;
    uint16_t target_time;
    motor_t motor[3];
} ServoBus_t;

#pragma pack(pop)

void ServoBus_Init(void);
HAL_StatusTypeDef ServoBus_SendCmd(const char *cmd);
HAL_StatusTypeDef ServoBus_SendAndWaitReply(const char *cmd, uint32_t reply_timeout_ms);
HAL_StatusTypeDef ServoBus_Move_One(ServoBus_t *servo);
HAL_StatusTypeDef ServoBus_Move_Many(ServoBus_t *servos, uint8_t count);
HAL_StatusTypeDef ServoBus_ReadAngle(uint8_t id);
HAL_StatusTypeDef ServoBus_SetID(uint8_t old_id, uint8_t new_id);
HAL_StatusTypeDef ServoBus_Unlock(uint8_t id);
HAL_StatusTypeDef ServoBus_Lock(uint8_t id);
void ServoBus_ParseReply(void);
void ServoBus_Start_Receive(void);
void ServoBus_TaskReceive(void);
void ServoBus_ErrorRecovery(void);

#endif
