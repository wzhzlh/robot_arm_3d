#ifndef COMMUCTION_H
#define COMMUCTION_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define SERVO_TX_TIMEOUT     100
#define SERVO_BAUDRATE       115200
#define SERVO_MAX_ID         254
#define SERVO_POS_MIN        500
#define SERVO_POS_MAX        2500
#define SERVO_TIME_MIN       0
#define SERVO_TIME_MAX       9999
#define SERVO_RX_BUF_LEN     32

extern uint8_t g_servo_id;
extern uint16_t g_servo_pwm;
extern uint8_t g_servo_reply_ok;
extern volatile uint8_t servo_error_pending;

extern SemaphoreHandle_t servo_tx_sem;
extern SemaphoreHandle_t servo_rx_sem;
extern uint8_t servo_rx_buf[SERVO_RX_BUF_LEN];
extern uint8_t servo_rx_data[SERVO_RX_BUF_LEN];
extern uint16_t servo_rx_len;

_Pragma("pack(1)")

typedef struct {
    double x;
    double y;
    double z;
} state_t;

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
    int count;
} motor_t;

typedef struct {
    target_t target_pos;
    state_t state_pos;
    uint16_t target_time;
    motor_t motor[3];
    int count;
} ServoBus_t;

_Pragma("pack()")

void ServoBus_Init(void);
HAL_StatusTypeDef ServoBus_SendCmd(const char *cmd);
HAL_StatusTypeDef ServoBus_Move_One(ServoBus_t *servo);
HAL_StatusTypeDef ServoBus_Move_Many(ServoBus_t *servos, uint8_t count);
HAL_StatusTypeDef ServoBus_ReadAngle(uint8_t id);
HAL_StatusTypeDef ServoBus_SetID(uint8_t old_id, uint8_t new_id);
HAL_StatusTypeDef ServoBus_Unlock(uint8_t id);
HAL_StatusTypeDef ServoBus_Lock(uint8_t id);
void ServoBus_ParseReply(void);
void ServoBus_Start_Receive(void);
void ServoBus_RequestNextAngle(void);
void ServoBus_TaskReceive(void);
void ServoBus_ErrorRecovery(void);

#endif
