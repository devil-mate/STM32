#include "can_handler.h"
#include "cmsis_os.h"
#include <string.h>
#include "common.h"



//can波特率计算
//PCLK1/((CAN_SJW + CAN_BS1 + CAN_BS2)*CAN_Prescaler) AHB1总线
//42M/（（（1+6+7）* 6）=512k
extern CAN_HandleTypeDef    hcan1;

extern CanRxBuffer_s        CanRxMsgBuffer;
//extern QueueHandle_t      canRxMsgQueueHandle;
extern SemaphoreHandle_t BinarySem_CanRx;
static CAN_RxHeaderTypeDef* nextRxHeader(CanRxBuffer_s*);
static uint8_t* nextRxData(CanRxBuffer_s*);
static void updateRxCnt(CanRxBuffer_s*);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    HAL_StatusTypeDef HAL_RetVal;
	
	static BaseType_t xHigherPriorityTaskWoken;
	//__set_PRIMASK(1); //关总中断
    if (hcan == &hcan1)
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data); //获取接收到的数据
        if (HAL_OK == HAL_RetVal)
        {
            /*复制到RxBuffer*/
            memcpy(nextRxHeader(&CanRxMsgBuffer), &header, sizeof(header));
            memcpy(nextRxData(&CanRxMsgBuffer), data, sizeof(data));
            updateRxCnt(&CanRxMsgBuffer);
        }
    }
	//__set_PRIMASK(0); //开总中断
	xSemaphoreGiveFromISR(BinarySem_CanRx,&xHigherPriorityTaskWoken);
//	//xQueueSendFromISR(canRxMsgQueueHandle,&CanRxMsgBuffer,&xHigherPriorityTaskWoken);
	
}

static CAN_RxHeaderTypeDef* nextRxHeader(CanRxBuffer_s* rxBuffer)
{
    return &(rxBuffer->msg[rxBuffer->cnt].header);
}

static uint8_t* nextRxData(CanRxBuffer_s* rxBuffer)
{
    return (uint8_t*)rxBuffer->msg[rxBuffer->cnt].data;
}

static void updateRxCnt(CanRxBuffer_s* rxBuffer)
{
    rxBuffer->cnt = (rxBuffer->cnt + 1) % CAN_MSG_BUF_LEN;
}
