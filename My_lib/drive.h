#ifndef __DRIVE_H
#define __DRIVE_H

#include <stdint.h>
#include "commuction.h"

extern float L1;
extern float L2;
extern float L3;

void IK_3D(ServoBus_t *robot_arm);
void IK_2D(ServoBus_t *robot_arm);
float clamp(float val, float min, float max);
uint16_t angle_to_pwm_id0(float angle);
uint16_t angle_to_pwm_id1(float angle);
uint16_t angle_to_pwm_id2(float angle);
target_t FK_3D(ServoBus_t *robot_arm);

#endif
