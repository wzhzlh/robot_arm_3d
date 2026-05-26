#include "commuction.h"
#include "start_task.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

uint8_t servo_tx_busy = 0;                     // 保留兼容
SemaphoreHandle_t servo_tx_sem = NULL;         // 发送完成二值信号量
SemaphoreHandle_t servo_rx_sem = NULL;         // 接收完成二值信号量
char a[52];  // 拼接指令
char b[52];// 发送指令（对比用）
char c[52];
int error_cnt;
// ==================== DMA安全发送缓冲区（全局！不能用栈上变量传给DMA） ====================
static char dma_tx_buf[256];       // 最大的指令长度
// ==================== 舵机接收缓冲区 ====================
uint8_t servo_rx_buf[SERVO_RX_BUF_LEN];  // DMA原始接收缓存
uint8_t servo_rx_data[SERVO_RX_BUF_LEN]; // 解析用缓存
uint16_t servo_rx_len = 0;               // 接收数据长度

// ==================== 解析后存储的舵机数据 ====================
uint8_t  g_servo_id = 0;         // 反馈的舵机ID
uint16_t g_servo_pwm = 0;        // 反馈的PWM值
uint8_t  g_servo_reply_ok = 0;   // 指令执行成功标志

/**
 * @brief  舵机总线初始化（创建二值信号量）
 *         在首次调用ServoBus相关函数前执行
 */
void ServoBus_Init(void)
{
    if(servo_tx_sem == NULL)
        servo_tx_sem = xSemaphoreCreateBinary();
    if(servo_rx_sem == NULL)
        servo_rx_sem = xSemaphoreCreateBinary();
}

HAL_StatusTypeDef ServoBus_SendCmd(const char *cmd)
{
    if(cmd == NULL || strlen(cmd) == 0)
        return HAL_ERROR;

    // 确保信号量已创建
    if(servo_tx_sem == NULL) ServoBus_Init();

    uint16_t len = strlen(cmd);
    if(len >= sizeof(dma_tx_buf))
        return HAL_ERROR;

    // 拷贝到全局DMA安全缓冲区再发送，避免DMA读栈上已释放内存
    memcpy(dma_tx_buf, cmd, len);
    memcpy(c, cmd, len);

    // 清除信号量初始态，然后启动DMA发送（仿485_bus.c模式）
    xSemaphoreTake(servo_tx_sem, 0);
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dma_tx_buf, len);
    memcpy(b, cmd, len);

    if(status == HAL_OK)
    {
        /* 阻塞等待DMA发送完成信号量（由TxCplt ISR释放），超时100ms */
        if(xSemaphoreTake(servo_tx_sem, pdMS_TO_TICKS(SERVO_TX_TIMEOUT)) != pdTRUE)
        {
            status = HAL_TIMEOUT;
        }
    }
    return status;
}

/**
 * @brief  DMA发送完成回调（ISR中释放发送信号量）
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        servo_tx_busy = 0;  // 保留兼容
        xSemaphoreGiveFromISR(servo_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  ========== 新增：串口错误回调函数 ==========
 * @param  huart: 串口句柄
 * @retval None
 */
// 错误恢复请求标志（在中断中设置，任务中处理）
volatile uint8_t servo_error_pending = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        // 中断中只设置标志，不调用任何 HAL DMA 函数！
        servo_tx_busy = 0;
        servo_error_pending = 1;
        error_cnt++;
    }
}


/**
 * @brief  启动DMA+空闲中断接收
 * @note   在任务上下文中调用（用于初始启动和错误恢复后重启）
 */
void ServoBus_Start_Receive(void)
{
    // 确保信号量已创建
    if(servo_rx_sem == NULL) ServoBus_Init();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, servo_rx_buf, SERVO_RX_BUF_LEN);
    
    ServoBus_ReadAngle(1);
    ServoBus_ReadAngle(2);
    ServoBus_ReadAngle(3);
    
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

// 在任务上下文中安全恢复串口（不能中断中调用）
void ServoBus_ErrorRecovery(void)
{
    if(!servo_error_pending) return;
    servo_error_pending = 0;

    // 中止所有UART2的DMA操作
    HAL_UART_Abort(&huart2);

    servo_rx_len = 0;
    memset(servo_rx_buf, 0, SERVO_RX_BUF_LEN);
    memset(servo_rx_data, 0, SERVO_RX_BUF_LEN);

    // 重新启动接收
    ServoBus_Start_Receive();
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
    if(strstr((char *)servo_rx_data, "P") != NULL)
    {
      unsigned int tmp_id, tmp_pwm;
      sscanf((char *)servo_rx_data, "#%03uP%04u!", &tmp_id, &tmp_pwm);
      g_servo_id = (uint8_t)tmp_id;
      g_servo_pwm = (uint16_t)tmp_pwm;
      g_servo_reply_ok = 1;
      if(g_servo_id < 3) {
          arm.motor[g_servo_id].motor_rx_pos = g_servo_pwm;
      }
    }
    // 清空缓存
    memset(servo_rx_data, 0, SERVO_RX_BUF_LEN);
    servo_rx_len = 0;
}

/**
 * @brief  在任务上下文中等待并处理舵机反馈数据
 * @note   阻塞等待二值信号量（ISR收到数据时释放），然后解析帧数据
 *         将帧解析从ISR移到任务上下文，降低中断负载
 */
void ServoBus_TaskReceive(void)
{
    // 阻塞等待接收信号量（由HAL_UARTEx_RxEventCallback的ISR中释放）
    if(xSemaphoreTake(servo_rx_sem, portMAX_DELAY) == pdTRUE)
    {
        // 在任务上下文中安全地解析舵机反馈帧
        ServoBus_ParseReply();
    }
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
        uint16_t pos = (uint16_t)(servos->motor[i].motor_tx_pos);
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