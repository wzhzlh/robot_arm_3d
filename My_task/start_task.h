#ifndef START_TASK_H
#define START_TASK_H

#include "cmsis_os.h"
#include "main.h"
#include "drive.h"
#include "commuction.h"
#include "k230.h"
#include "adc.h"
typedef enum {
    ARM_IDLE = 0,
    ARM_MOVE_TO_TARGET,     // 移动到目标位置
    ARM_ARRIVED,           // 已到达目标位置
    ARM_RETURN_HOME        // 返回原点
} ArmStateTypeDef;

extern ServoBus_t arm;
extern ArmStateTypeDef arm_state;
void requirement(void  * argument);
// void mot_rece(void * argument);
void arm_init(void);
void requirement_1(void);
void requirement_2(void);
void requirement_3(void);

#endif