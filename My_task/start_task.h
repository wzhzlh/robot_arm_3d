#ifndef START_TASK_H
#define START_TASK_H

#include "cmsis_os.h"
#include "main.h"
#include "drive.h"
#include "commuction.h"
#include "k230.h"

/* ==================== 枚举类型声明 ==================== */
typedef enum {
    ARM_IDLE = 0,
    ARM_MOVE_TO_TARGET,     // 移动到目标位置
    ARM_ARRIVED,           // 已到达目标位置
    ARM_RETURN_HOME        // 返回原点
} ArmStateTypeDef;

/* ==================== 全局变量声明 ==================== */
extern ServoBus_t arm;
extern ArmStateTypeDef arm_state;
/* ==================== FreeRTOS 任务函数声明 ==================== */
void requirement(void  * argument);
// void mot_rece(void * argument);
// void requirement_2(void *argument);
/* ==================== 非 static 函数声明 ==================== */
void arm_init(void);
void requirement1(void);
void requirement2(void);
void requirement3(void);
void requirement4(void);
void requirement5(void);

#endif
