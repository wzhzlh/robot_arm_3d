#include "FreeRTOS.h"
#include "start_task.h"
#include "task.h"

/* ---------------------------------------------------------------
 * start_task : 基本要求(1)
 *   上电后依次完成三个基本动作演示：
 *   Step1 - joint1 水平旋转  0 -> 270 -> 0
 *   Step2 - joint2 竖直旋转  0 -> 180 -> 0
 *   Step3 - joint3 末端旋转  0 -> 180 -> 0
 *   每步运动时间 2000 ms，到位后停留 500 ms
 * --------------------------------------------------------------- */

/* 舵机数组：3个关节，ID分别为1/2/3 */
ServoBus_t arm;

void requirement(void *argument)
{
   
requirement1();

requirement2();

requirement3();

requirement4();

requirement5();

}


void mot_rece(void *argument)
{
     ServoBus_ErrorRecovery();
}

void k230_receive(void *argument)
{
  
}

/* ---------------------------------------------------------------
 * comm_task : 通信任务
 *   处理串口通信，包括舵机反馈和K230视觉数据
 * --------------------------------------------------------------- */
void requirement_4(void *argument)
{
    // /* 启动舵机串口接收 */
    // ServoBus_Start_Receive();
    
    // float error_threshold = 2.0f;  // 角度误差阈值
    
    // for(;;)
    // {
    //     /* 处理舵机反馈数据 */
    //     if(g_servo_reply_ok)
    //     {
    //         /* 解析舵机返回的角度数据 */
    //         float angle = (g_servo_pwm - 500) / 7.407f;
            
    //         /* 根据舵机ID存储实际角度并进行闭环控制 */
    //         if(g_servo_id >= 1 && g_servo_id <= 3)
    //         {
    //             /* 计算当前角度误差 */
    //             float expected_angle = 0.0f;
    //             if(g_servo_id == 1) expected_angle = arm.motor[0].motor_tx_pos;
    //             else if(g_servo_id == 2) expected_angle = arm.motor[1].motor_tx_pos;
    //             else if(g_servo_id == 3) expected_angle = arm.motor[2].motor_tx_pos;
                
    //             float error = expected_angle - angle;
                
    //             /* 如果当前实际角度与期望角度差异过大，进行补偿 */
    //             if(fabsf(error) > error_threshold)
    //             {
    //                 /* 这里可以实现简单的比例补偿算法 */
    //                 float new_target_angle = angle + error * 0.8f;  // 80%的误差补偿
                    
    //                 /* 根据ID设置新的目标角度 */
    //                 float th1 = arm.motor[0].motor_tx_pos, th2 = arm.motor[1].motor_tx_pos, th3 = arm.motor[2].motor_tx_pos;
    //                 if(g_servo_id == 1) th1 = new_target_angle;
    //                 else if(g_servo_id == 2) th2 = new_target_angle;
    //                 else if(g_servo_id == 3) th3 = new_target_angle;
                    
    //                 /* 发送修正后的角度命令 */
    //                 set_angles(th1, th2, th3, 500);
    //             }
    //         }
            
    //         g_servo_reply_ok = 0;  // 清除标志
    //     }
        
    //     osDelay(50);
    // }
}

/* ---------------------------------------------------------------
 * sensor_task : 传感器任务
 *   处理传感器数据（预留，可扩展添加更多传感器）
 * --------------------------------------------------------------- */
void requirement_5(void *argument)
{
    // for(;;)
    // {
    //     /* 预留传感器数据处理逻辑 */
    //     osDelay(20);
    // }
}
