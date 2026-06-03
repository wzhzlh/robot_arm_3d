#ifndef K230_H
#define K230_H

#include <stdint.h>
#include "commuction.h"
#include "crc_ccitt.h"

#define K230_RX_BUF_LEN 32u

typedef enum {
    K230_IDLE = 0,
    K230_RECEIVING,
    K230_RECEIVED_OK,
    K230_RECEIVE_ERROR
} K230_StatusTypeDef;

typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t z;
} K230_TargetPosTypeDef;

extern K230_TargetPosTypeDef k230_target_pos;
extern K230_StatusTypeDef k230_comm_status;

void K230_UART_Init(void);
void K230_ParseFrame(uint8_t *buf, uint16_t len);

#endif
