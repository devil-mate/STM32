#ifndef __CAN_HANDLER_H
#define __CAN_HANDLER_H

#include "common.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"

#define CAN_MSG_BUF_LEN 32

#pragma pack(push,1)

typedef struct 
{
    CAN_TxHeaderTypeDef     header;
    uint8_t                 data[8];
} CanTxMsg_s;

typedef struct 
{
    CAN_RxHeaderTypeDef     header;
    uint8_t                 data[8];
} CanRxMsg_s;



#pragma pack(pop)

//extern CanRxBuffer_s        CanRxBuffer;
//extern CanTxBuffer_s        CanTxBuffer;

void initCANHandler(void);

uint16_t pushTxMsg(CanTxMsg_s* msg);
uint16_t pushTxMsg0(CAN_TxHeaderTypeDef* header, uint8_t data[]);

#endif
