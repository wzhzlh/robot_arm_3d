#include "drive.h"
#include <math.h>

static const float rad_to_angle = 57.2957795f;

float L1 = 0.05550f;
float L2 = 0.20000f;
float L3 = 0.15149f;

float clamp(float val, float min, float max)
{
    if(val < min) return min;
    if(val > max) return max;
    return val;
}

void IK_2D(ServoBus_t *robot_arm)
{
    float A = L2;
    float B = L3;
    float r = sqrtf(robot_arm->target_pos.x * robot_arm->target_pos.x +
                    robot_arm->target_pos.y * robot_arm->target_pos.y);
    float h = robot_arm->target_pos.z - L1;
    float d = sqrtf(r * r + h * h);

    if(d > A + B) d = A + B;
    if(d < fabs(A - B)) d = fabs(A - B);

    float cos3 = -(A * A + B * B - d * d) / (2.0f * A * B);
    cos3 = clamp(cos3, -1.0f, 1.0f);
    float theta3 = acosf(cos3);
    float phi = atan2f(h, r);
    float cos_beta = (A * A + d * d - B * B) / (2.0f * A * d);
    cos_beta = clamp(cos_beta, -1.0f, 1.0f);
    float beta = acosf(cos_beta);
    float theta2 = phi + beta; 

    robot_arm->motor[1].motor_tx_pos = theta2 * rad_to_angle;
    robot_arm->motor[2].motor_tx_pos = theta3 * rad_to_angle;
}

void IK_3D(ServoBus_t *robot_arm)
{
    float theta1_target_deg = atan2f(robot_arm->target_pos.y, robot_arm->target_pos.x) * rad_to_angle;
    if (theta1_target_deg < 0.0f)
        theta1_target_deg += 360.0f;
    robot_arm->motor[0].motor_tx_pos = theta1_target_deg;
    IK_2D(robot_arm);
}

target_t FK_3D(ServoBus_t *robot_arm)
{
    target_t pos;
    float th1 = robot_arm->motor[0].motor_rx_pos / rad_to_angle;
    float th2 = robot_arm->motor[1].motor_rx_pos / rad_to_angle;
    float th3 = robot_arm->motor[2].motor_rx_pos / rad_to_angle;

    float z = L2 * cosf(th2) + L3 * cosf(th2 + th3);
    float r = L2 * sinf(th2) + L3 * sinf(th2 + th3);
    z += L1;
    pos.x = r * cosf(th1);
    pos.y = r * sinf(th1);
    pos.z = z;

    return pos;
}

uint16_t angle_to_pwm_id0(float angle)
{
    angle = clamp(angle, 0.0f, 360.0f);
    return (uint16_t)(angle * 7.407f + 500.0f);
}

uint16_t angle_to_pwm_id1(float angle)
{
    angle = clamp(angle, 0.0f, 270.0f);
    return (uint16_t)(angle * 7.407f + 500.0f);
}

uint16_t angle_to_pwm_id2(float angle)
{
    angle = clamp(angle, -135.0f, 135.0f);
    return (uint16_t)(angle * 7.407f + 1500.0f);
}
