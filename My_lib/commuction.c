#include "commuction.h"

#include <string.h>

uint8_t servo_tx_busy = 0;
char a[52];  // 拼接指令
char b[52];// 发送指令（对比用）
char c[52];
int error_cnt;
// ==================== 舵机接收缓冲区 ====================
uint8_t servo_rx_buf[SERVO_RX_BUF_LEN];  // DMA原始接收缓存
uint8_t servo_rx_data[SERVO_RX_BUF_LEN]; // 解析用缓存
uint16_t servo_rx_len = 0;               // 接收数据长度

// ==================== 解析后存储的舵机数据 ====================
uint8_t  g_servo_id = 0;         // 反馈的舵机ID
uint16_t g_servo_pwm = 0;        // 反馈的PWM值
uint8_t  g_servo_reply_ok = 0;   // 指令执行成功标志

HAL_StatusTypeDef ServoBus_SendCmd(const char *cmd)
{
    if(cmd == NULL || strlen(cmd) == 0)
        return HAL_ERROR;

    if(servo_tx_busy == 1)
        return HAL_BUSY;
    memcpy(c,cmd,strlen(cmd));
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
 * @brief  ========== 新增：串口错误回调函数 ==========
 * @param  huart: 串口句柄
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        // 出错后复位所有状态
        servo_tx_busy = 0;                // 清除发送忙标志
        servo_rx_len = 0;
        // 清空接收缓存
        memset(servo_rx_buf, 0, SERVO_RX_BUF_LEN);
        memset(servo_rx_data, 0, SERVO_RX_BUF_LEN);

        // 重启接收，防止串口卡死
        ServoBus_Start_Receive();
			error_cnt++;
    }
}


/**
 * @brief  启动DMA+空闲中断接收
 */
void ServoBus_Start_Receive(void)
{
    HAL_UART_AbortReceive(&huart2);
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, servo_rx_buf, SERVO_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	  ServoBus_ReadAngle(0);
		ServoBus_ReadAngle(1);
		ServoBus_ReadAngle(2);
}

/**
 * @brief  解析众灵舵机反馈帧
 */
void ServoBus_ParseReply(void)
{
    if(servo_rx_len == 0) return;

    // 帧格式校验
    if(servo_rx_data[0] != '#' || servo_rx_data[servo_rx_len-1] != '!')
    {
        memset(servo_rx_data, 0, SERVO_RX_BUF_LEN);
        servo_rx_len = 0;
        return;
    }

    // 解析 #OK!
    if(strstr((char *)servo_rx_data, "OK") != NULL)
    {
        g_servo_reply_ok = 1;
    }
    // 解析角度 #XXXPXXXX!
    else if(strstr((char *)servo_rx_data, "P") != NULL)
    {
        sscanf((char *)servo_rx_data, "#%03uP%04u!", &g_servo_id, &g_servo_pwm);
        g_servo_reply_ok = 1;
    }

    // 清空缓存
    memset(servo_rx_data, 0, SERVO_RX_BUF_LEN);
    servo_rx_len = 0;
}

// ===========================================================================
HAL_StatusTypeDef ServoBus_Move_One(ServoBus_t *servo)
{
    if(servo == NULL || servo->motor[0].id > SERVO_MAX_ID)
        return HAL_ERROR;

    uint16_t pos = (uint16_t)servo->target_pos.x;
    if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;

    if (servo->target_time < SERVO_TIME_MIN) servo->target_time = SERVO_TIME_MIN;
    if (servo->target_time > SERVO_TIME_MAX) servo->target_time = SERVO_TIME_MAX;

    char cmd[32] = {0};
    sprintf(cmd, "#%03uP%04uT%04u!", servo->motor[0].id, pos, servo->target_time);
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_Move_Many(ServoBus_t *servos, uint8_t count)
{
    if(servos == NULL || count == 0 || count > 16)
        return HAL_ERROR;

    char cmd[256] = "{G0000";
    char temp[32] = {0};

    for(uint8_t i = 0; i < count; i++)
    {
        uint16_t pos = (uint16_t)(servos->motor[i].motor_pos);
        if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
        if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;

        uint16_t time = servos->target_time;
        if (time < SERVO_TIME_MIN) time = SERVO_TIME_MIN;
        if (time > SERVO_TIME_MAX) time = SERVO_TIME_MAX;

        sprintf(temp, "#%03uP%04uT%04u!", servos->motor[i].id, pos, time);
        strcat(cmd, temp);
    }

    strcat(cmd, "}");
    memcpy(a, cmd, strlen(cmd));
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_ReadAngle(uint8_t id)
{
    if(id > SERVO_MAX_ID) return HAL_ERROR;
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPRAD!", id);
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_SetID(uint8_t old_id, uint8_t new_id)
{
    if(old_id > SERVO_MAX_ID || new_id > SERVO_MAX_ID) return HAL_ERROR;
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPID%03u!", old_id, new_id);
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_Unlock(uint8_t id)
{
    if(id > SERVO_MAX_ID) return HAL_ERROR;
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPULK!", id);
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_Lock(uint8_t id)
{
    if(id > SERVO_MAX_ID) return HAL_ERROR;
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPLOK!", id);
    return ServoBus_SendCmd(cmd);
}