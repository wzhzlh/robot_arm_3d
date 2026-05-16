#include "drive.h"
#include <math.h>

// 共面角度误差阈值(°)
#define COPLANAR_ERROR 1.0f
#define M_PI 3.14159265358979323846f
// 数值限幅
float clamp(float val, float min, float max)
{
    if(val < min) return min;
    if(val > max) return max;
    return val;
}

// 2D逆解：关节2、3
void IK_2D(ServoBus_t *robot_arm)
{
    float A = L2;
    float B = L3;
    float r = sqrtf(robot_arm->target_pos.x * robot_arm->target_pos.x + 
                    robot_arm->target_pos.y * robot_arm->target_pos.y);
    float z = robot_arm->target_pos.z-L1;
    float d = sqrtf(r*r + z*z);

    // 工作空间边界约束
    if(d > A + B)
    {
        r = r * (A+B)/d;
        z = z * (A+B)/d;
        d = A+B;
    }

    // 计算关节3角度
    float cos3 = (A*A + B*B - d*d) / (2*A*B);
    cos3 = clamp(cos3, -1.0f, 1.0f);
    float theta3 = acosf(cos3);

    // 计算关节2角度
    float C = L2 + L3 * cosf(theta3);
    float S = L3 * sinf(theta3);
    float theta2 = atan2f(z, r) + atan2f(S, C) + M_PI/2;

    // 角度转换、偏置、限幅
    robot_arm->motor[1].motor_pos = theta2 * RAD_TO_ANGLE + robot_arm->motor[0].offset;
    robot_arm->motor[2].motor_pos = theta3 * RAD_TO_ANGLE + robot_arm->motor[1].offset;

    robot_arm->motor[1].motor_pos = clamp(robot_arm->motor[1].motor_pos, THETA2_MIN, THETA2_MAX);
    robot_arm->motor[2].motor_pos = clamp(robot_arm->motor[2].motor_pos, THETA3_MIN, THETA3_MAX);
}

// 3D逆解：完整求解
void IK_3D(ServoBus_t *robot_arm)
{
    float x = robot_arm->target_pos.x;
    float y = robot_arm->target_pos.y;

    // 目标关节1角度
    float theta1_target_deg = atan2f(y, x) * RAD_TO_ANGLE;
    // 当前关节1角度
    float theta1_current_deg = robot_arm->motor[0].motor_pos;

    // 非共面：旋转关节1对齐
    if(fabs(theta1_target_deg - theta1_current_deg) > COPLANAR_ERROR)
    {
        robot_arm->motor[0].motor_pos = theta1_target_deg + robot_arm->motor[0].offset;
        robot_arm->motor[0].motor_pos = clamp(robot_arm->motor[0].motor_pos, THETA1_MIN, THETA1_MAX);
    }

    // 共面后2D求解
    IK_2D(robot_arm);
}

// 正运动学：基坐标系坐标
target_t FK_3D(ServoBus_t *robot_arm)
{
    target_t pos;
    float th1 = robot_arm->motor[0].motor_pos / RAD_TO_ANGLE;
    float th2 = robot_arm->motor[1].motor_pos / RAD_TO_ANGLE;
    float th3 = robot_arm->motor[2].motor_pos / RAD_TO_ANGLE;

    float z = L2 * cosf(th2) + L3 * cosf(th2 + th3);
    float r = L2 * sinf(th2) + L3 * sinf(th2 + th3);
    z += L1;

    pos.x = r * cosf(th1);
    pos.y = r * sinf(th1);
    pos.z = z;

    return pos;
}

// 角度转PWM值
// ID0：0° → 500
uint16_t angle_to_pwm_id0(float angle)
{
    angle = clamp(angle, 0.0f, 270.0f);
    return (uint16_t)(angle * 7.407f + 500.0f);
}

// ID1：0° → 500
uint16_t angle_to_pwm_id1(float angle)
{
    angle = clamp(angle, 0.0f, 270.0f);
    return (uint16_t)(angle * 7.407f + 500.0f);
}

// ID2：0° → 1500（你要的特殊零点）
uint16_t angle_to_pwm_id2(float angle)
{
    angle = clamp(angle, 0.0f, 270.0f);
    return (uint16_t)(angle * 7.407f + 1500.0f);
}