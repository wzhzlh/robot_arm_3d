#include "commuction.h"
#include "usart.h"

static uint8_t servo_tx_busy = 0;

// ==================== 舵机接收缓冲区 ====================

uint8_t servo_rx_buf[SERVO_RX_BUF_LEN];  // DMA原始接收缓存
uint8_t servo_rx_data[SERVO_RX_BUF_LEN]; // 解析用缓存
uint16_t servo_rx_len = 0;               // 接收数据长度

// ==================== 新增：解析后存储的舵机数据 ====================
uint8_t  g_servo_id = 0;         // 反馈的舵机ID
uint16_t g_servo_pwm = 0;        // 反馈的PWM值
uint8_t  g_servo_reply_ok = 0;   // 指令执行成功标志

/** 
 * @brief  底层：DMA方式发送舵机指令
 */
HAL_StatusTypeDef ServoBus_SendCmd(const char *cmd)
{
    if(cmd == NULL || strlen(cmd) == 0)
        return HAL_ERROR;

    if(servo_tx_busy == 1)
        return HAL_BUSY;

    servo_tx_busy = 1;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)cmd, strlen(cmd));

    if(status != HAL_OK)
    {
        servo_tx_busy = 0;
    }
    return status;
}
/**
 * @brief  DMA发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        servo_tx_busy = 0;
    }
}
/**
 * @brief  启动DMA+空闲中断接收
 */
void ServoBus_Start_Receive(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, servo_rx_buf, SERVO_RX_BUF_LEN);
}



/**
 * @brief  解析众灵舵机反馈帧（严格遵循手册 #...! 格式）
 * @note   手册反馈格式：
 *          1. 执行成功  -> #OK!
 *          2. 读角度返回 -> #001P1500!
 *          3. 其他指令 -> 对应格式帧
 */
void ServoBus_ParseReply()
{
    // 无数据直接返回
    if(servo_rx_len == 0) return;

    // 1. 校验帧格式：手册规定必须 #开头 !结尾
    if(servo_rx_data[0] != '#' || servo_rx_data[servo_rx_len-1] != '!')
    {
        memset(servo_rx_data, 0, SERVO_RX_BUF_LEN);
        servo_rx_len = 0;
        return;
    }

    // 2. 解析指令执行成功：#OK!
    if(strstr((char *)servo_rx_data, "OK") != NULL)
    {
        g_servo_reply_ok = 1;
    }
    // 3. 解析角度/位置反馈：#XXXPYYYY!
    else if(strstr((char *)servo_rx_data, "P") != NULL)
    {
        sscanf((char *)servo_rx_data, "#%03uP%04u!", &g_servo_id, &g_servo_pwm);
        g_servo_reply_ok = 1;
    }

    // 清空缓存，准备下一次接收
    memset(servo_rx_data, 0, SERVO_RX_BUF_LEN);
    servo_rx_len = 0;
}

// ===========================================================================
/**
 * @brief  控制单个舵机运动
 */
HAL_StatusTypeDef ServoBus_Move_One(ServoBus_t *servo)
{
    if(servo == NULL || servo->motor[0].id > SERVO_MAX_ID)
        return HAL_ERROR;

    uint16_t pos = (uint16_t)servo->target_pos.x;
    if (pos < SERVO_POS_MIN)
    {
        pos = SERVO_POS_MIN;
    }
    if (pos > SERVO_POS_MAX)
    {
        pos = SERVO_POS_MAX;
    }

    if (servo->target_time < SERVO_TIME_MIN)
    {
        servo->target_time = SERVO_TIME_MIN;
    }
    if (servo->target_time > SERVO_TIME_MAX)
    {
        servo->target_time = SERVO_TIME_MAX;
    }

    char cmd[32] = {0};
    sprintf(cmd, "#%03uP%04uT%04u!", servo->motor[0].id, pos, servo->target_time);

    return ServoBus_SendCmd(cmd);
}

/**
 * @brief  控制多个舵机同步运动
 */
HAL_StatusTypeDef ServoBus_Move_Many(ServoBus_t *servos, uint8_t count, uint16_t time)
{
    if(servos == NULL || count == 0 || count > 16)
        return HAL_ERROR;

    time = (time < SERVO_TIME_MIN) ? SERVO_TIME_MIN : time;
    time = (time > SERVO_TIME_MAX) ? SERVO_TIME_MAX : time;

    char cmd[128] = "#";
    char temp[32] = {0};

    for(uint8_t i = 0; i < count; i++)
    {
        uint16_t pos = (uint16_t)(servos->motor[i].motor_pos);
        if (pos < SERVO_POS_MIN)
        {
            pos = SERVO_POS_MIN;
        }
        if (pos > SERVO_POS_MAX)
        {
            pos = SERVO_POS_MAX;
        }

        sprintf(temp, "%03uP%04u", servos->motor[i].id, pos);
        strcat(cmd, temp);
    }

    sprintf(temp, "T%04u!", time);
    strcat(cmd, temp);

    return ServoBus_SendCmd(cmd);
}

/**
 * @brief  读取舵机当前角度
 */
HAL_StatusTypeDef ServoBus_ReadAngle(uint8_t id)
{
    if(id > SERVO_MAX_ID) return HAL_ERROR;
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPRAD!", id);
    return ServoBus_SendCmd(cmd);
}

/**
 * @brief  修改舵机ID
 */
HAL_StatusTypeDef ServoBus_SetID(uint8_t old_id, uint8_t new_id)
{
    if(old_id > SERVO_MAX_ID || new_id > SERVO_MAX_ID) return HAL_ERROR;
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPID%03u!", old_id, new_id);
    return ServoBus_SendCmd(cmd);
}

/**
 * @brief  解锁舵机参数
 */
HAL_StatusTypeDef ServoBus_Unlock(uint8_t id)
{
    if(id > SERVO_MAX_ID) return HAL_ERROR;
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPULK!", id);
    return ServoBus_SendCmd(cmd);
}

/**
 * @brief  锁定舵机参数
 */
HAL_StatusTypeDef ServoBus_Lock(uint8_t id)
{
    if(id > SERVO_MAX_ID) return HAL_ERROR;
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPLOK!", id);
    return ServoBus_SendCmd(cmd);
}

