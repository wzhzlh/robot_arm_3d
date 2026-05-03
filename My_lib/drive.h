#ifndef __DRIVE_H
#define __DRIVE_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "commuction.h"
// 机械臂连杆长度 (单位：米)
#define L1          0.15f   // joint1 和 joint2 的固定长度
#define L2          0.15f   // joint2 和 joint3
#define L3          0.05f   // joint3 和末端

#define BASE_HEIGHT 0.05f   // 基座高度

#define RAD_TO_ANGLE 57.2957795f
#define EPS 1e-3f

// 角度限制范围
#define THETA1_MIN 0.0f
#define THETA1_MAX 270.0f
#define THETA2_MIN 0.0f
#define THETA2_MAX 180.0f
#define THETA3_MIN 0.0f
#define THETA3_MAX 180.0f

// 关节角度
extern float motor1;  // joint1 旋转角
extern float motor2;  // joint2 俯仰角
extern float motor3;  // joint3 俯仰角

extern float offset[3];

// 运动学
void IK_3D(ServoBus_t *robot_arm);
void IK_2D(ServoBus_t *robot_arm);
float clamp(float val, float min, float max);
uint16_t angle_to_pwm(float angle);
target_t FK_3D(float theta1, float theta2, float theta3);

#endif
