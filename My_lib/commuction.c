#include "commuction.h"
#include "start_task.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* ==============================================================
 *  二值信号量机制 —— 将异步 DMA 收发转为同步"一问一答"
 * ==============================================================
 *
 *  信号量             |  用途                        |  ISR 给出点
 *  ───────────────────┼──────────────────────────────┼──────────────────────────
 *  servo_tx_sem       |  标记 "发送完毕"              |  HAL_UART_TxCpltCallback
 *                     |  ServoBus_SendCmd 内部 Take   |  (TX DMA 传输完成)
 *                     |  阻塞等待，确保前一帧发完再   |
 *                     |  发下一帧                     |
 *  ───────────────────┼──────────────────────────────┼──────────────────────────
 *  servo_rx_sem       |  标记 "有数据/错误到达"       |  HAL_UARTEx_RxEventCallback
 *                     |  用于 ServoBus_TaskReceive    |  HAL_UART_ErrorCallback
 *                     |  轮询模式的唤醒               |
 *  ───────────────────┼──────────────────────────────┼──────────────────────────
 *  servo_rx_reply_sem |  标记 "接收完毕"              |  HAL_UARTEx_RxEventCallback
 *                     |  ServoBus_SendAndWaitReply    |  (IDLE 中断, 回复帧收齐)
 *                     |  内部 Take，实现一问一答的    |  HAL_UART_ErrorCallback
 *                     |  同步阻塞等待                 |  (错误时防止死锁)
 * ============================================================== */

uint8_t servo_tx_busy = 0;
SemaphoreHandle_t servo_tx_sem = NULL;       /* 发送完毕信号量 */
SemaphoreHandle_t servo_rx_sem = NULL;       /* 接收通知信号量（轮询模式用） */
SemaphoreHandle_t servo_rx_reply_sem = NULL; /* 接收完毕信号量（一问一答同步模式用） */
char a[52];
char b[52];
char c[52];
int error_cnt;
int succse_cnt=0;
static char dma_tx_buf[256];

uint8_t servo_rx_buf[SERVO_RX_BUF_LEN];
uint8_t servo_rx_data[SERVO_RX_BUF_LEN];
uint16_t servo_rx_len = 0;

uint8_t g_servo_id = 0;
uint16_t g_servo_pwm = 0;
uint8_t g_servo_reply_ok = 0;
volatile uint8_t servo_error_pending = 0;

static void ServoBus_ClearRxState(void)
{
    servo_rx_len = 0;
    memset(servo_rx_buf, 0, sizeof(servo_rx_buf));
    memset(servo_rx_data, 0, sizeof(servo_rx_data));
}

static void ServoBus_RestartRxDma(void)
{
    HAL_UART_DMAStop(&huart2);
    ServoBus_ClearRxState();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, servo_rx_buf, SERVO_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

void ServoBus_Init(void)
{
    /* 发送完毕信号量：初始为 0，HAL_UART_TxCpltCallback ISR 中 Give */
    if(servo_tx_sem == NULL)
    {
        servo_tx_sem = xSemaphoreCreateBinary();
    }
    /* 接收通知信号量：轮询模式下由 HAL_UARTEx_RxEventCallback ISR Give */
    if(servo_rx_sem == NULL)
    {
        servo_rx_sem = xSemaphoreCreateBinary();
    }
    /* 接收完毕信号量：一问一答模式下由 HAL_UARTEx_RxEventCallback ISR Give */
    if(servo_rx_reply_sem == NULL)
    {
        servo_rx_reply_sem = xSemaphoreCreateBinary();
    }
}

HAL_StatusTypeDef ServoBus_SendCmd(const char *cmd)
{
    if(cmd == NULL || strlen(cmd) == 0)
    {
        return HAL_ERROR;
    }

    if(servo_tx_sem == NULL)
    {
        ServoBus_Init();
    }

    uint16_t len = strlen(cmd);
    if(len >= sizeof(dma_tx_buf))
    {
        return HAL_ERROR;
    }

    memcpy(dma_tx_buf, cmd, len);
    memcpy(c, cmd, len);

    /* ──── 发送握手 ────
     * ① TakeNonBlocking: 清空上次可能残留的信号量（确保本次是新鲜等待）
     * ② 启动 DMA 发送
     * ③ TakeBlocking:    阻塞等待 ISR Give → 标记 "发送完毕"
     *                    超时代表 DMA 故障，返回 HAL_TIMEOUT
     */
    xSemaphoreTake(servo_tx_sem, 0);  /* 非阻塞，清残留 */
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dma_tx_buf, len);
    memcpy(b, cmd, len);

    if(status == HAL_OK)
    {
        if(xSemaphoreTake(servo_tx_sem, pdMS_TO_TICKS(SERVO_TX_TIMEOUT)) != pdTRUE)
        {
            status = HAL_TIMEOUT;  /* DMA 发送超时 */
        }
    }

    return status;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        servo_tx_busy = 0;
        /* ──── ★ 发送完毕 ────
         * TX DMA 传输完成，Give servo_tx_sem 唤醒阻塞在 ServoBus_SendCmd 的任务
         */
        xSemaphoreGiveFromISR(servo_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        /* STM32F4: 通过读SR再读DR清除所有错误标志(PE/FE/NE/ORE) */
        {
            volatile uint32_t tmp = huart->Instance->SR;
            tmp = huart->Instance->DR;
            (void)tmp;
        }
        servo_tx_busy = 0;
        servo_error_pending = 1;
        error_cnt++;
        /* ──── 错误时释放两个信号量 ────
         * servo_rx_sem:       通知轮询任务 (ServoBus_TaskReceive)
         * servo_rx_reply_sem: 通知同步任务 (ServoBus_SendAndWaitReply)
         * 必须同时 Give，否则同步模式会永久阻塞
         */
        if(servo_rx_sem != NULL)
        {
            xSemaphoreGiveFromISR(servo_rx_sem, &xHigherPriorityTaskWoken);
        }
        if(servo_rx_reply_sem != NULL)
        {
            xSemaphoreGiveFromISR(servo_rx_reply_sem, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void ServoBus_Start_Receive(void)
{
    if(servo_rx_sem == NULL)
    {
        ServoBus_Init();
    }
    ServoBus_RestartRxDma();
}



void ServoBus_ErrorRecovery(void)
{
    if(!servo_error_pending)
    {
        return;
    }

    servo_error_pending = 0;
//		error_cnt=0;
    HAL_UART_Abort(&huart2);
    ServoBus_Start_Receive();
}

void ServoBus_ParseReply(void)
{
    if(servo_rx_len == 0)
    {
        return;
    }

    if(servo_rx_data[0] != '#' || servo_rx_data[servo_rx_len - 1] != '!')
    {
        memset(servo_rx_data, 0, sizeof(servo_rx_data));
        servo_rx_len = 0;

        return;
    }

    if(strstr((char *)servo_rx_data, "P") != NULL)
    {
        unsigned int tmp_id = 0;
        unsigned int tmp_pwm = 0;

        if(sscanf((char *)servo_rx_data, "#%03uP%04u!", &tmp_id, &tmp_pwm) == 2)
        {
            g_servo_id = (uint8_t)tmp_id;
            g_servo_pwm = (uint16_t)tmp_pwm;
            g_servo_reply_ok = 1;

            if(g_servo_id < 3)
            {
                arm.motor[g_servo_id].motor_rx_pos = g_servo_pwm;
										succse_cnt++;
            }
        }
    }

    memset(servo_rx_data, 0, sizeof(servo_rx_data));
    servo_rx_len = 0;
}

/* ==============================================================
 * ServoBus_SendAndWaitReply : 同步"一问一答"阻塞调用
 *
 *  时序 (Task 与 ISR 通过两个二值信号量握手):
 *    Task:  Take(rx_reply_sem, 0)  ──── 清残留
 *    Task:  ServoBus_SendCmd()      ──── 发送命令
 *              | 内部: Take(tx_sem, 0) 清残留
 *              |       HAL_UART_Transmit_DMA()
 *              |       Take(tx_sem, BLOCK)  <-- 阻塞...
 *    ISR:      |                    ────  TxCplt: Give(tx_sem)  [★ 发送完毕]
 *    Task:     <--- 唤醒            ────  退出 SendCmd
 *    Task:  Take(rx_reply_sem, BLOCK) <-- 阻塞...
 *    ISR:                          ────  RxEvent:
 *                                           copy + ParseReply()
 *                                           Give(rx_reply_sem)  [★ 接收完毕]
 *    Task:     <--- 唤醒            ────  return g_servo_reply_ok
 *
 *  此时 g_servo_id / g_servo_pwm / arm.motor[].motor_rx_pos 已更新
 * ============================================================== */
HAL_StatusTypeDef ServoBus_SendAndWaitReply(const char *cmd, uint32_t reply_timeout_ms)
{
    /* ① 清残留：非阻塞 Take，确保拿到的是本次回复的信号量 */
    xSemaphoreTake(servo_rx_reply_sem, 0);
    /* ② 发送：内部阻塞等待 servo_tx_sem (标记 "发送完毕") */
    HAL_StatusTypeDef status ;
    /* ③ 接收：阻塞等待 servo_rx_reply_sem (标记 "接收完毕")
     *    回复已在 ISR 中由 ServoBus_ParseReply 解析完毕 */
    if(xSemaphoreTake(servo_rx_reply_sem, pdMS_TO_TICKS(reply_timeout_ms)) == pdTRUE)
    {
         status = ServoBus_SendCmd(cmd);
            return HAL_OK;
    }
        if(status != HAL_OK)
    {
        return status;
    }
}

void ServoBus_TaskReceive(void)
{
    if(xSemaphoreTake(servo_rx_sem, pdMS_TO_TICKS(SERVO_RX_TIMEOUT)) == pdTRUE)
    {
        if(servo_error_pending)
        {
            ServoBus_ErrorRecovery();
        }
        else if(servo_rx_len > 0)
        {
            ServoBus_ParseReply();
        }
    }
    else
    {
        if(servo_error_pending)
        {
            ServoBus_ErrorRecovery();
        }
        else
        {
            ServoBus_Start_Receive();
        }
    }
}

HAL_StatusTypeDef ServoBus_Move_One(ServoBus_t *servo)
{
    if(servo == NULL || servo->motor[0].id > SERVO_MAX_ID)
    {
        return HAL_ERROR;
    }

    uint16_t pos = (uint16_t)servo->target_pos.x;
    if(pos < SERVO_POS_MIN)
    {
        pos = SERVO_POS_MIN;
    }
    if(pos > SERVO_POS_MAX)
    {
        pos = SERVO_POS_MAX;
    }

#if SERVO_TIME_MIN > 0
    if(servo->target_time < SERVO_TIME_MIN)
    {
        servo->target_time = SERVO_TIME_MIN;
    }
#endif
    if(servo->target_time > SERVO_TIME_MAX)
    {
        servo->target_time = SERVO_TIME_MAX;
    }

    char cmd[32] = {0};
    sprintf(cmd, "#%03uP%04uT%04u!", servo->motor[0].id, pos, servo->target_time);
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_Move_Many(ServoBus_t *servos, uint8_t count)
{
    if(servos == NULL || count == 0 || count > 16)
    {
        return HAL_ERROR;
    }

    char cmd[256] = "{G0000";
    char temp[32] = {0};

    for(uint8_t i = 0; i < count; i++)
    {
        uint16_t pos = (uint16_t)servos->motor[i].motor_tx_pos;
        if(pos < SERVO_POS_MIN)
        {
            pos = SERVO_POS_MIN;
        }
        if(pos > SERVO_POS_MAX)
        {
            pos = SERVO_POS_MAX;
        }

        uint16_t time = servos->target_time;
#if SERVO_TIME_MIN > 0
        if(time < SERVO_TIME_MIN)
        {
            time = SERVO_TIME_MIN;
        }
#endif
        if(time > SERVO_TIME_MAX)
        {
            time = SERVO_TIME_MAX;
        }

        sprintf(temp, "#%03uP%04uT%04u!", servos->motor[i].id, pos, time);
        strcat(cmd, temp);
    }

    strcat(cmd, "}");
    memcpy(a, cmd, strlen(cmd));
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_ReadAngle(uint8_t id)
{
    if(id > SERVO_MAX_ID)
    {
        return HAL_ERROR;
    }
    char cmd[16] = {0};
    sprintf(cmd, "#%03uPRAD!", id);
    return ServoBus_SendAndWaitReply(cmd, SERVO_RX_TIMEOUT);  /* 同步一问一答 */
}

HAL_StatusTypeDef ServoBus_SetID(uint8_t old_id, uint8_t new_id)
{
    if(old_id > SERVO_MAX_ID || new_id > SERVO_MAX_ID)
    {
        return HAL_ERROR;
    }

    char cmd[16] = {0};
    sprintf(cmd, "#%03uPID%03u!", old_id, new_id);
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_Unlock(uint8_t id)
{
    if(id > SERVO_MAX_ID)
    {
        return HAL_ERROR;
    }

    char cmd[16] = {0};
    sprintf(cmd, "#%03uPULK!", id);
    return ServoBus_SendCmd(cmd);
}

HAL_StatusTypeDef ServoBus_Lock(uint8_t id)
{
    if(id > SERVO_MAX_ID)
    {
        return HAL_ERROR;
    }

    char cmd[16] = {0};
    sprintf(cmd, "#%03uPLOK!", id);
    return ServoBus_SendCmd(cmd);
}
