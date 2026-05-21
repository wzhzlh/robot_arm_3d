#include "drive.h"
#include <math.h>

// 共面角度误差阈值(°)
#define COPLANAR_ERROR 5.0f
#define M_PI 3.14159265358979323846f
float L1=0.05;
float L2=0.07;
float L3=0.1;
// 数值限幅
float clamp(float val, float min, float max)
{
    if(val < min) return min;
    if(val > max) return max;
    return val;
}

// 全局变量
float A, B, r, h, d, cos3, theta3, C, S, theta2;
void IK_2D(ServoBus_t *robot_arm)
{
    A = L2;
    B = L3;
    r = sqrtf(robot_arm->target_pos.x * robot_arm->target_pos.x + 
              robot_arm->target_pos.y * robot_arm->target_pos.y);
    h = robot_arm->target_pos.z - L1;
    d = sqrtf(r*r + h*h);

    if(d > A + B) d = A + B;
    if(d < fabs(A - B)) d = fabs(A - B);

    // ==============================
    cos3 = -(A*A + B*B - d*d) / (2.0f * A * B);
    cos3 = clamp(cos3, -1.0f, 1.0f);
    theta3 = acosf(cos3);   // 肘上
    // theta3 = -acosf(cos3); // 肘下
    float phi = atan2f(h, r);        
    float cos_beta = (A*A + d*d - B*B) / (2.0f * A * d);
    cos_beta = clamp(cos_beta, -1.0f, 1.0f);
    float beta = acosf(cos_beta);
    
    theta2 = phi + beta;  
    robot_arm->motor[1].motor_tx_pos = theta2 * RAD_TO_ANGLE;
    robot_arm->motor[2].motor_tx_pos = theta3 * RAD_TO_ANGLE;

    // 角度限位
    robot_arm->motor[1].motor_tx_pos = clamp(robot_arm->motor[1].motor_tx_pos, THETA2_MIN, THETA2_MAX);
    robot_arm->motor[2].motor_tx_pos = clamp(robot_arm->motor[2].motor_tx_pos, THETA3_MIN, THETA3_MAX);
}

void IK_3D(ServoBus_t *robot_arm)
{
    float x = robot_arm->target_pos.x;
    float y = robot_arm->target_pos.y;

    // 目标关节1角度
    float theta1_target_deg = atan2f(y, x) * RAD_TO_ANGLE;
    // 当前关节1角度
	ServoBus_Start_Receive();
    float theta1_current_deg = (robot_arm->motor[0].motor_rx_pos-500)/7.04;

    // 非共面：旋转关节1对齐
    if(fabs(theta1_target_deg - theta1_current_deg) > COPLANAR_ERROR)
    {
         robot_arm->motor[0].motor_tx_pos = theta1_target_deg;
    }

    // 共面后2D求解
    IK_2D(robot_arm);
}

// 正运动学：基坐标系坐标
target_t FK_3D(ServoBus_t *robot_arm)
{
    target_t pos;
    float th1 = robot_arm->motor[0].motor_tx_pos / RAD_TO_ANGLE;
    float th2 = robot_arm->motor[1].motor_tx_pos / RAD_TO_ANGLE;
    float th3 = robot_arm->motor[2].motor_tx_pos / RAD_TO_ANGLE;

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
    angle = clamp(angle, -135.0f, 135.0f);
    return (uint16_t)(angle * 7.407f + 1500.0f);
}