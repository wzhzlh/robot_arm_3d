#ifndef START_TASK_H
#define START_TASK_H

#include "cmsis_os.h"
#include "main.h"
#include "drive.h"
#include "commuction.h"
#include "k230.h"

/* ---------------------------------------------------------------
 * 矩形轨迹参数（供 requiremnet2 使用）
 * --------------------------------------------------------------- */
#define SQ_Z    0.10f   /* 末端高度，离桌面约 5cm */
#define SQ_X0   0.10f   /* x 最小 */
#define SQ_X1   0.20f   /* x 最大 */
#define SQ_Y0  -0.05f   /* y 最小 */
#define SQ_Y1   0.05f   /* y 最大 */

#define INTERP_STEPS    5
#define INTERP_STEP_MS  200

/* ==================== 全局变量声明 ==================== */
extern ServoBus_t arm;

/* ==================== FreeRTOS 任务函数声明 ==================== */
// void requirement(void  * argument);
void mot_rece(void * argument);
// void k230_receive(void *argument);
// void requiremnet_2(void *argument);
/* ==================== 非 static 函数声明 ==================== */
void arm_init(void);
void requirement1(void);
void requirement2(void);
void requirement3(void);
void requirement3(void);
void requirement4(void);
void requirement5(void);

#endif
