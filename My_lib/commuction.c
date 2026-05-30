#include "commuction.h"
#include "start_task.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

uint8_t servo_tx_busy = 0;
SemaphoreHandle_t servo_tx_sem = NULL;
SemaphoreHandle_t servo_rx_sem = NULL;
char a[52];
char b[52];
char c[52];
int error_cnt;

static char dma_tx_buf[256];
static uint8_t servo_poll_id = 0;

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
    if(servo_tx_sem == NULL)
    {
        servo_tx_sem = xSemaphoreCreateBinary();
    }
    if(servo_rx_sem == NULL)
    {
        servo_rx_sem = xSemaphoreCreateBinary();
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

    xSemaphoreTake(servo_tx_sem, 0);
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dma_tx_buf, len);
    memcpy(b, cmd, len);

    if(status == HAL_OK)
    {
        if(xSemaphoreTake(servo_tx_sem, pdMS_TO_TICKS(SERVO_TX_TIMEOUT)) != pdTRUE)
        {
            status = HAL_TIMEOUT;
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
        xSemaphoreGiveFromISR(servo_tx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        __HAL_UART_CLEAR_FLAG(huart,
                              UART_CLEAR_OREF |
                              UART_CLEAR_FEF |
                              UART_CLEAR_NEF |
                              UART_CLEAR_PEF);
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
        servo_tx_busy = 0;
        servo_error_pending = 1;
        error_cnt++;
        if(servo_rx_sem != NULL)
        {
            xSemaphoreGiveFromISR(servo_rx_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void ServoBus_Start_Receive(void)
{
    if(servo_rx_sem == NULL)
    {
        ServoBus_Init();
    }

    ServoBus_RestartRxDma();
    ServoBus_RequestNextAngle();
}

void ServoBus_RequestNextAngle(void)
{
    ServoBus_ReadAngle(servo_poll_id);
    servo_poll_id++;
    if(servo_poll_id >= 3)
    {
        servo_poll_id = 0;
    }
}

void ServoBus_ErrorRecovery(void)
{
    if(!servo_error_pending)
    {
        return;
    }

    servo_error_pending = 0;
    HAL_UART_Abort(&huart2);
    servo_poll_id = 0;
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
        ServoBus_RequestNextAngle();
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
            }
        }
    }

    memset(servo_rx_data, 0, sizeof(servo_rx_data));
    servo_rx_len = 0;
    ServoBus_RequestNextAngle();
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

    if(servo->target_time < SERVO_TIME_MIN)
    {
        servo->target_time = SERVO_TIME_MIN;
    }
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
        if(time < SERVO_TIME_MIN)
        {
            time = SERVO_TIME_MIN;
        }
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
    return ServoBus_SendCmd(cmd);
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
