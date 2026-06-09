#include "FreeRTOS.h"
#include "start_task.h"
#include "task.h"

ServoBus_t arm;
ArmStateTypeDef arm_state = ARM_IDLE;

float key=0;
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
    while(1)
    {
    key = ADC_GetValue();
        if(key>0.0 && key<0.2){
            requirement_1();
        }
        if(key>0.3 && key<0.6)
        {
            requirement_2();
        }
        if(key>0.7 && key<1.0)
        {
            requirement_3();
        }
    }
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


void requirement_1(void)
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

void requirement_2(void)
{
    arm_init();
    K230_UART_Init();

    float last_x = 0.0f, last_y = 0.0f, last_z = 0.0f;
    uint8_t steps = 5;
    uint16_t step_ms = 200;
    k230_comm_status = K230_RECEIVED_OK;
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

void requirement_3(void)
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
float ADC_GetValue(void)
{
    HAL_ADC_Start(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 100);

    float value = HAL_ADC_GetValue(&hadc1);
    float voltage = (float)(value / 4095.0f); // 12位ADC
    HAL_ADC_Stop(&hadc1);
    return voltage;
}