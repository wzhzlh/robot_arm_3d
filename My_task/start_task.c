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

/* ---------------------------------------------------------------
 * arm_control_task : 基本要求(3) - 视觉识别定位红色目标物
 * 基本要求(4) - 移动到识别到的目标位置
 *   通过K230视觉识别模块获取红色目标物坐标，控制机械臂移动到目标位置
 *   使用直线插补方式平滑移动到目标
 * --------------------------------------------------------------- */

/* 状态枚举 */
typedef enum {
    ARM_IDLE = 0,
    ARM_MOVE_TO_TARGET,     // 移动到目标位置
    ARM_ARRIVED,           // 已到达目标位置
    ARM_RETURN_HOME        // 返回原点
} ArmStateTypeDef;

/* 目标高度 (单位: 米) */
#define TARGET_Z  0.10f  // 移动到目标时的高度

static ArmStateTypeDef arm_state = ARM_IDLE;

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

/* ==================== 函数声明 ==================== */
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

/* ==================== FreeRTOS 任务函数 ==================== */

void requirement(void *argument)
{
    requiremnet1();
    requiremnet2();
    requirement3();
    requirement4();
    requirement5();
}

void mot_rece(void *argument)
{
    /* 初始化舵机串口DMA接收 */
    ServoBus_Start_Receive();

    for(;;)
    {
        /* 在任务上下文中等待并处理舵机反馈 */
        ServoBus_TaskReceive();

        /* 帧解析完成，g_servo_reply_ok、g_servo_pwm 等已更新 */
        if(g_servo_reply_ok)
        {
            /* 可在此添加闭环控制等逻辑 */
            g_servo_reply_ok = 0;  /* 清除标志 */
        }
    }
}

void k230_receive(void *argument)
{

}

/* ---------------------------------------------------------------
 * comm_task : 通信任务
 *   处理串口通信，包括舵机反馈和K230视觉数据
 * --------------------------------------------------------------- */
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


/* ==================== requiremnet 函数 ==================== */

void requiremnet1(void)
{
    /* 等待系统稳定 */
    arm_init();
    vTaskDelay(500);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    // /* ---- 归零：所有关节回到 0 度 ---- */
    set_angles(0.0f, 0.0f, 0.0f, 1000);
    vTaskDelay(1000);

    /* ====================================================
     * Step 1：joint1 水平旋转  0 -> 270 -> 0
     * ==================================================== */
    set_angles(270.0f, 0.0f, 0.0f, 2000);
    vTaskDelay(1500);
    ServoBus_ReadAngle(1);
    set_angles(0.0f, 0.0f, 0.0f, 2000);
    vTaskDelay(1500);

    /* ====================================================
     * Step 2：joint2 竖直旋转  0 -> 180 -> 0
     * ==================================================== */
    set_angles(0.0f, 180.0f, 0.0f, 2000);
    vTaskDelay(2500);

    set_angles(0.0f, 0.0f, 0.0f, 2000);
    vTaskDelay(2500);

    /* ====================================================
     * Step 3：joint3 末端旋转  0 -> 180 -> 0
     * ==================================================== */
    set_angles(0.0f, 0.0f, -90.0f, 2000);
    vTaskDelay(1500);
    set_angles(0.0f, 0.0f, 90.0f, 2000);
    vTaskDelay(1500);
    set_angles(0.0f, 0.0f, 0.0f, 2000);
    vTaskDelay(1500);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(6000));
    // vTaskDelete(NULL);
}

void requiremnet2(void)
{
    /* ---- 初始化舵机参数 ---- */
    arm_init();

    for(;;)
    {
        line_interp(0.05f, 0.05f, 0.15f,
                    -0.05f, 0.05f, 0.15f,
                    INTERP_STEPS, INTERP_STEP_MS);
        line_interp(-0.05f, 0.05f, 0.15f,
                    -0.05f, 0.05f, 0.05f,
                    INTERP_STEPS, INTERP_STEP_MS);
        line_interp(-0.05f, 0.05f, 0.05f,
                    0.05f, 0.05f, 0.05f,
                    INTERP_STEPS, INTERP_STEP_MS);
        line_interp(0.05f, 0.05f, 0.05f,
                    0.05f, 0.05f, 0.15f,
                    INTERP_STEPS, INTERP_STEP_MS);
    }
}

void requiremnet3(void)
{
    // /* 等待start_task完成基本动作演示 */
    // osDelay(25000);
    
    // /* 初始化K230视觉通信 */
    // K230_UART_Init();
    
    // for(;;)
    // {
    //     /* 检查是否有新的视觉目标数据 */
    //     if(k230_comm_status == K230_RECEIVED_OK && arm_state == ARM_IDLE)
    //     {
    //         /* 获取K230识别到的目标坐标 */
    //         float target_x = (float)k230_target_pos.x;
    //         float target_y = (float)k230_target_pos.y;
    //         float target_z = (float)k230_target_pos.z;
            
    //         /* 重置通信状态 */
    //         k230_comm_status = K230_IDLE;
            
    //         /* 开始移动到目标位置 */
    //         arm_state = ARM_MOVE_TO_TARGET;
            
    //         /* 移动到识别到的目标位置 */
    //         move_to(target_x, target_y, TARGET_Z, 2000);
    //         osDelay(2500);
            
    //         /* 到达目标位置 */
    //         arm_state = ARM_ARRIVED;
            
    //         /* 在目标位置停留一段时间 */
    //         osDelay(1000);
            
    //         // /* 返回原点 */
    //         //  move_to(0.15f, 0.0f, 0.15f, 2000);
    //         // osDelay(2500);
            
    //         /* 状态重置 */
    //         arm_state = ARM_IDLE;
    //     }
        
    //     osDelay(10);
    // }
}

void requirement3(void)
{

}
void requirement4(void)
{

}
void requirement5(void)
{

}

/* ==================== 辅助函数 ==================== */

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

/* ---------------------------------------------------------------
 * 直线轨迹：先建立 x(t)=kx*t+b, y(t)=ky*t+b, z(t)=kz*t+b
 *   再按 step_time 周期采样执行，保证末端按笛卡尔直线运动
 * --------------------------------------------------------------- */
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
        // osDelay(move_time_ms + 20u);

        elapsed_ms = next_elapsed_ms;
    }
}
