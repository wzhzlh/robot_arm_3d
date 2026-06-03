#include "FreeRTOS.h"
#include "start_task.h"
#include "task.h"

ServoBus_t arm;
ArmStateTypeDef arm_state = ARM_IDLE;

typedef struct {
    float k;
    float b;
} LinearAxisTrajectory;

typedef struct {
    LinearAxisTrajectory lx;
    LinearAxisTrajectory ly;
    LinearAxisTrajectory lz;
    uint32_t total_time_ms;
} LinearTrajectory3D;


static void set_angles(float th1, float th2, float th3, uint16_t move_time);
static void move_to(float x, float y, float z, uint16_t move_time);
static float linear_traj_eval(const LinearAxisTrajectory *axis, float time_ms);
static void update_linear_trajectory(LinearTrajectory3D *traj,
                                     float x0, float y0, float z0,
                                     float x1, float y1, float z1,
                                     uint32_t total_time_ms);
static void line_interp(float x0, float y0, float z0,
                        float x1, float y1, float z1,
                        uint8_t steps, uint16_t step_time);



void requirement(void *argument)
{
    requirement1();
    requirement2();
    requirement3();
    requirement4();
    requirement5();
}

//  HAL_StatusTypeDef ret[3];

// void mot_rece(void *argument)
// {
//     arm_init();
//     ServoBus_Start_Receive();

//     TickType_t last_wake_time = xTaskGetTickCount();
//     while(1)
//     {
//         ret[0] = ServoBus_ReadAngle(1);
//         ret[1] = ServoBus_ReadAngle(2);
//         ret[2] = ServoBus_ReadAngle(3);

//         if(ret[0] != HAL_OK || ret[1] != HAL_OK || ret[2] != HAL_OK)
//         {
//             ServoBus_ErrorRecovery();
//         }
//         else
//         {
//             g_servo_reply_ok = 0;
//         }
//         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(300));
//     }
// }


void requirement_2(void *argument)
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



void requirement1(void)
{

    arm_init();
    vTaskDelay(500);

    set_angles(0.0f, 0.0f, 0.0f, 1000);
    vTaskDelay(1000);

    set_angles(270.0f, 0.0f, 0.0f, 2000);
    vTaskDelay(1500);
    ServoBus_ReadAngle(1);
    set_angles(0.0f, 0.0f, 0.0f, 2000);
    vTaskDelay(1500);

    set_angles(0.0f, 180.0f, 0.0f, 2000);
    vTaskDelay(2500);

    set_angles(0.0f, 0.0f, 0.0f, 2000);
    vTaskDelay(2500);

    set_angles(0.0f, 0.0f, -90.0f, 2000);
    vTaskDelay(1500);
    set_angles(0.0f, 0.0f, 90.0f, 2000);
    vTaskDelay(1500);
    
    set_angles(0.0f, 0.0f, 0.0f, 2000);
    vTaskDelay(1500);
    // vTaskDelete(NULL);
}

void requirement2(void)
{
    arm_init();
    K230_UART_Init();

    float last_x = 0.0f, last_y = 0.0f, last_z = 0.0f;
    uint8_t steps = 5;
    uint16_t step_ms = 200;

    for(;;)
    {
        if(k230_comm_status == K230_RECEIVED_OK)
        {
            k230_comm_status = K230_IDLE;
            arm.target_pos.x = (float)k230_target_pos.x;
            arm.target_pos.y = (float)k230_target_pos.y;
            arm.target_pos.z = (float)k230_target_pos.z;
            line_interp(last_x, last_y, last_z,
                        arm.target_pos.x, arm.target_pos.y, arm.target_pos.z,
                        steps, step_ms);
            last_x = arm.target_pos.x;
            last_y = arm.target_pos.y;
            last_z = arm.target_pos.z;
        }
    }
} 

void requirement3(void)
{
    K230_UART_Init();
    for(;;)
    {
        if(k230_comm_status == K230_RECEIVED_OK && arm_state == ARM_IDLE)
        {
            arm.target_pos.x = (float)k230_target_pos.x;
            arm.target_pos.y = (float)k230_target_pos.y;
            arm.target_pos.z = (float)k230_target_pos.z;
            k230_comm_status = K230_IDLE;
            arm_state = ARM_MOVE_TO_TARGET;
            move_to(arm.target_pos.x, arm.target_pos.y, arm.target_pos.z, 2000);
            arm_state = ARM_ARRIVED;
            arm_state = ARM_IDLE;
        }
    }
}

void arm_init(void)
{
    arm.motor[0].offset = 500.0f;
    arm.motor[1].offset = 500.0f;
    arm.motor[2].offset = 1500.0f;
    arm.motor[0].id = 0;
    arm.motor[1].id = 1;
    arm.motor[2].id = 2;
    set_angles(180,0, 0, 1000);
}

/* 将三个关节角度(度)写入舵机结构体并发送 */
static void set_angles(float th1, float th2, float th3, uint16_t move_time)
{
    arm.motor[0].id = 0;
    arm.motor[0].motor_tx_pos = (uint16_t)angle_to_pwm_id0(th1*0.675f);
    arm.target_time  = move_time;

    arm.motor[1].id = 1;
    arm.motor[1].motor_tx_pos = (uint16_t)angle_to_pwm_id1(th2);
    arm.target_time  = move_time;

    arm.motor[2].id = 2;
    arm.motor[2].motor_tx_pos = (uint16_t)angle_to_pwm_id2(th3);
    arm.target_time  = move_time;

    ServoBus_Move_Many(&arm, 3);
}

/* 通过逆运动学将末端移动到指定空间坐标 (x,y,z 单位:米) */
static void move_to(float x, float y, float z, uint16_t move_time)
{
    arm.target_pos.x = (double)x;
    arm.target_pos.y = (double)y;
    arm.target_pos.z = (double)z;
    arm.target_time  = move_time;

    IK_3D(&arm);

    set_angles(arm.motor[0].motor_tx_pos, arm.motor[1].motor_tx_pos, arm.motor[2].motor_tx_pos, move_time);
}

static float linear_traj_eval(const LinearAxisTrajectory *axis, float time_ms)
{
    return axis->k * time_ms + axis->b;
}

static void update_linear_trajectory(LinearTrajectory3D *traj,
                                     float x0, float y0, float z0,
                                     float x1, float y1, float z1,
                                     uint32_t total_time_ms)
{
    float time_ms = (float)total_time_ms;

    traj->total_time_ms = total_time_ms;
    traj->lx.k = (x1 - x0) / time_ms;
    traj->lx.b = x0;
    traj->ly.k = (y1 - y0) / time_ms;
    traj->ly.b = y0;
    traj->lz.k = (z1 - z0) / time_ms;
    traj->lz.b = z0;
}

static void line_interp(float x0, float y0, float z0,
                        float x1, float y1, float z1,
                        uint8_t steps, uint16_t step_time)
{
    LinearTrajectory3D traj;
    uint32_t elapsed_ms = 0;

    if ((steps == 0u) || (step_time == 0u))
    {
        return;
    }

    update_linear_trajectory(&traj, x0, y0, z0, x1, y1, z1, (uint32_t)steps * step_time);

    while (elapsed_ms < traj.total_time_ms)
    {
        uint32_t next_elapsed_ms = elapsed_ms + step_time;
        uint16_t move_time_ms;
        float sample_time_ms;
        float x;
        float y;
        float z;

        if (next_elapsed_ms > traj.total_time_ms)
        {
            next_elapsed_ms = traj.total_time_ms;
        }

        move_time_ms = next_elapsed_ms - elapsed_ms;
        sample_time_ms = (float)next_elapsed_ms;
        x = linear_traj_eval(&traj.lx, sample_time_ms);
        y = linear_traj_eval(&traj.ly, sample_time_ms);
        z = linear_traj_eval(&traj.lz, sample_time_ms);

        move_to(x, y, z, move_time_ms);
        elapsed_ms = next_elapsed_ms;
    }
}
