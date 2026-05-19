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
void arm_init(void)
{
    arm.motor[0].offset = 500.0f;
    arm.motor[1].offset = 500.0f;
    arm.motor[2].offset = 1500.0f;
    arm.motor[0].id = 0;
    arm.motor[1].id = 1;
    arm.motor[2].id = 2;
}
/* 将三个关节角度(度)写入舵机结构体并发送 */
static void set_angles(float th1, float th2, float th3, uint16_t move_time)
{
	 
    arm.motor[0].id = 0;
    arm.motor[0].motor_tx_pos = (uint16_t)angle_to_pwm_id0(th1);
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
//    ServoBus_t tmp;
    arm.target_pos.x = (double)x;
    arm.target_pos.y = (double)y;
    arm.target_pos.z = (double)z;
    arm.target_time  = move_time;

    IK_3D(&arm);   /* 结果写入全局 motor1/motor2/motor3 */

    set_angles(arm.motor[0].motor_tx_pos, arm.motor[1].motor_tx_pos, arm.motor[2].motor_tx_pos, move_time);
}

/* ---------------------------------------------------------------
 * 直线插补：从 (x0,y0,z0) 到 (x1,y1,z1)，分 steps 步
 *   step_time : 每步运动时间(ms)，同时也是每步等待时间
 * --------------------------------------------------------------- */
static void line_interp(float x0, float y0, float z0,
                        float x1, float y1, float z1,
                        uint8_t steps, uint16_t step_time)
{
    for (uint8_t i = 1; i <= steps; i++)
    {
        float t = (float)i / (float)steps;
        float x = x0 + t * (x1 - x0);
        float y = y0 + t * (y1 - y0);
        float z = z0 + t * (z1 - z0);
        move_to(x, y, z, step_time);
        osDelay(step_time + 20);   /* 等待到位，留20ms余量 */
    }
}

void requirement_1(void *argument)
{
//    /* 等待系统稳定 */
//	   arm_init();
//     vTaskDelay(500);

//TickType_t xLastWakeTime = xTaskGetTickCount();
////    /* ---- 归零：所有关节回到 0 度 ---- */
//    set_angles(0.0f, 0.0f, 0.0f, 1000);
//    vTaskDelay(1000);

//    /* ====================================================
//     * Step 1：joint1 水平旋转  0 -> 270 -> 0
//     * ==================================================== */
//    set_angles(270.0f, 0.0f, 0.0f, 2000);
//    vTaskDelay(1500);
//     ServoBus_ReadAngle(1);
//    set_angles(0.0f, 0.0f, 0.0f, 2000);
//    vTaskDelay(1500);

//    /* ====================================================
//     * Step 2：joint2 竖直旋转  0 -> 180 -> 0
//     * ==================================================== */
//    set_angles(0.0f, 180.0f, 0.0f, 2000);
//    vTaskDelay(2500);

//    set_angles(0.0f, 0.0f, 0.0f, 2000);
//    vTaskDelay(2500);

//    /* ====================================================
//     * Step 3：joint3 末端旋转  0 -> 180 -> 0
//     * ==================================================== */
//    set_angles(0.0f, 0.0f, -90.0f, 2000);
//    vTaskDelay(1500);
//    set_angles(0.0f, 0.0f, 90.0f, 2000);
//    vTaskDelay(1500);
//    set_angles(0.0f, 0.0f, 0.0f, 2000);
//    vTaskDelay(1500);
//    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(6000));  /* 等待20秒，确保演示完成 */
//    /* 基本动作演示完毕，删除本任务 */
////    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------
 * main_task : 基本要求(2)
 *   机械臂末端在水平面上走 10cm x 10cm 正方形轨迹
 *
 *   坐标系：x 向前，y 向左，z 向上（单位：米）
 *   正方形中心：(0.15, 0, 0.10)，末端高度 z=0.10m（离桌面约5cm）
 *   四个顶点（顺时针）：
 *     P1 (0.10, -0.05, 0.10)
 *     P2 (0.20, -0.05, 0.10)
 *     P3 (0.20,  0.05, 0.10)
 *     P4 (0.10,  0.05, 0.10)
 *   每条边插补 10 步，每步 100ms，边长 10cm 匀速运动
 *   走完一圈后回到起点，循环执行
 * --------------------------------------------------------------- */

/* 正方形顶点坐标 (单位: 米) */
#define SQ_Z    0.10f   /* 末端高度，离桌面约 5cm */
#define SQ_X0   0.10f   /* x 最小 */
#define SQ_X1   0.20f   /* x 最大 */
#define SQ_Y0  -0.05f   /* y 最小 */
#define SQ_Y1   0.05f   /* y 最大 */

/* 每条边插补步数和每步时间 */
#define INTERP_STEPS    10
#define INTERP_STEP_MS  100
float x,y,z;
void requirement_2(void *argument)
{
    /* 等待 start_task 完成基本动作演示 */
//    osDelay(20000);

    /* 先移动到正方形起点 P1，抬升到位 */
    /* 确保舵机参数已初始化，移动到起点 P1 */
    arm_init();
        move_to(0, 0, SQ_Z, 1500);
//	    move_to(x, y, z, 1500);
       osDelay(1600);

//    for (;;)
//    {
//        /* P1 -> P2 : x 方向正向 */
//        line_interp(SQ_X0, SQ_Y0, SQ_Z,
//                    SQ_X1, SQ_Y0, SQ_Z,
//                    INTERP_STEPS, INTERP_STEP_MS);

//        /* P2 -> P3 : y 方向正向 */
//        line_interp(SQ_X1, SQ_Y0, SQ_Z,
//                    SQ_X1, SQ_Y1, SQ_Z,
//                    INTERP_STEPS, INTERP_STEP_MS);

//        /* P3 -> P4 : x 方向反向 */
//        line_interp(SQ_X1, SQ_Y1, SQ_Z,
//                    SQ_X0, SQ_Y1, SQ_Z,
//                    INTERP_STEPS, INTERP_STEP_MS);

//        /* P4 -> P1 : y 方向反向，闭合 */
//        line_interp(SQ_X0, SQ_Y1, SQ_Z,
//                    SQ_X0, SQ_Y0, SQ_Z,
//                    INTERP_STEPS, INTERP_STEP_MS);

//        /* 每圈结束停留 500ms */
//        osDelay(500);
//    }
}

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

void requirement_3(void *argument)
{
    /* 等待start_task完成基本动作演示 */
    osDelay(25000);
    
    /* 初始化K230视觉通信 */
    K230_UART_Init();
    
    for(;;)
    {
        /* 检查是否有新的视觉目标数据 */
        if(k230_comm_status == K230_RECEIVED_OK && arm_state == ARM_IDLE)
        {
            /* 获取K230识别到的目标坐标 */
            float target_x = (float)k230_target_pos.x;
            float target_y = (float)k230_target_pos.y;
            float target_z = (float)k230_target_pos.z;
            
            /* 重置通信状态 */
            k230_comm_status = K230_IDLE;
            
            /* 开始移动到目标位置 */
            arm_state = ARM_MOVE_TO_TARGET;
            
            /* 移动到识别到的目标位置 */
            move_to(target_x, target_y, TARGET_Z, 2000);
            osDelay(2500);
            
            /* 到达目标位置 */
            arm_state = ARM_ARRIVED;
            
            /* 在目标位置停留一段时间 */
            osDelay(1000);
            
            // /* 返回原点 */
            //  move_to(0.15f, 0.0f, 0.15f, 2000);
            // osDelay(2500);
            
            /* 状态重置 */
            arm_state = ARM_IDLE;
        }
        
        osDelay(10);
    }
}

/* ---------------------------------------------------------------
 * comm_task : 通信任务
 *   处理串口通信，包括舵机反馈和K230视觉数据
 * --------------------------------------------------------------- */
void requirement_4(void *argument)
{
    /* 启动舵机串口接收 */
    ServoBus_Start_Receive();
    
    float error_threshold = 2.0f;  // 角度误差阈值
    
    for(;;)
    {
        /* 处理舵机反馈数据 */
        if(g_servo_reply_ok)
        {
            /* 解析舵机返回的角度数据 */
            float angle = (g_servo_pwm - 500) / 7.407f;
            
            /* 根据舵机ID存储实际角度并进行闭环控制 */
            if(g_servo_id >= 1 && g_servo_id <= 3)
            {
                /* 计算当前角度误差 */
                float expected_angle = 0.0f;
                if(g_servo_id == 1) expected_angle = arm.motor[0].motor_tx_pos;
                else if(g_servo_id == 2) expected_angle = arm.motor[1].motor_tx_pos;
                else if(g_servo_id == 3) expected_angle = arm.motor[2].motor_tx_pos;
                
                float error = expected_angle - angle;
                
                /* 如果当前实际角度与期望角度差异过大，进行补偿 */
                if(fabsf(error) > error_threshold)
                {
                    /* 这里可以实现简单的比例补偿算法 */
                    float new_target_angle = angle + error * 0.8f;  // 80%的误差补偿
                    
                    /* 根据ID设置新的目标角度 */
                    float th1 = arm.motor[0].motor_tx_pos, th2 = arm.motor[1].motor_tx_pos, th3 = arm.motor[2].motor_tx_pos;
                    if(g_servo_id == 1) th1 = new_target_angle;
                    else if(g_servo_id == 2) th2 = new_target_angle;
                    else if(g_servo_id == 3) th3 = new_target_angle;
                    
                    /* 发送修正后的角度命令 */
                    set_angles(th1, th2, th3, 500);
                }
            }
            
            g_servo_reply_ok = 0;  // 清除标志
        }
        
        osDelay(50);
    }
}

/* ---------------------------------------------------------------
 * sensor_task : 传感器任务
 *   处理传感器数据（预留，可扩展添加更多传感器）
 * --------------------------------------------------------------- */
void requirement_5(void *argument)
{
    for(;;)
    {
        /* 预留传感器数据处理逻辑 */
        osDelay(20);
    }
}


