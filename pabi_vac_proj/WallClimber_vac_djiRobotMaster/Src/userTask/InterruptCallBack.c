#include "main.h"


#include "cmsis_os.h"
#include <string.h>

#include "can_handler.h"
#include "ringbuf.h"

#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#define RXBUFFERSIZE   20 //缓存大小
#define MAXDIS 12000.0   //mm

extern SemaphoreHandle_t BinarySem_Rx;
extern QueueHandle_t      rs485RxMsgQueueHandle;
int16_t errorCount_local=0;
extern uint8_t aRxBuffer[];//HAL库使用的串口接收缓冲


extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart8;
uint8_t  ultrasonic_uart_temp[REC_LENGTH] = {0};       //USART接收数据缓存
/* can 中断接收 */

extern CAN_HandleTypeDef    hcan1;
extern SemaphoreHandle_t BinarySem_CanRx;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static CAN_RxHeaderTypeDef header;
    static uint8_t data[8];
    static HAL_StatusTypeDef HAL_RetVal;
	static BaseType_t xHigherPriorityTaskWoken;
	if (hcan == &hcan1)
	{
		HAL_RetVal = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data); //获取接收到的数据
		if (HAL_OK == HAL_RetVal)
		{
//            /*写入到RxBuffer*/
			//TODO 指针,以及memcpy问题
			static CanRxMsg_s tempCanRxMsgs;
			memcpy(&tempCanRxMsgs.header,&header,sizeof(tempCanRxMsgs.header));
			memcpy(&tempCanRxMsgs.data,data,sizeof(tempCanRxMsgs.data));
			RxRingBuff_writeOneMsg(tempCanRxMsgs);
			//sFlag_g.canInitReciveFlag=true;
			xSemaphoreGiveFromISR(BinarySem_CanRx,&xHigherPriorityTaskWoken);
		}
		
	}
	
}

// 串口接收中断回调


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(USART6 == huart->Instance){
		//static uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
//		//TODO  xHigherPriorityTaskWoken 参数：用于保存是否有高优先级任务准备就绪，
//		//如果函数执行完毕后，此参数的数值是pdTRUE，说明有高优先级任务要执行，否则没有。
		static BaseType_t xHigherPriorityTaskWoken;
//		//放入环形缓冲区
		Bluetooth_Write(aRxBuffer,RXBUFFERSIZE);
//		//写入缓冲区（非环形缓冲区）
		//bufferWrite(aRxBuffer,RXBUFFERSIZE);
		xSemaphoreGiveFromISR(BinarySem_Rx,&xHigherPriorityTaskWoken);
	//中断接收	
		HAL_UART_Receive_IT(&huart6,(uint8_t*)aRxBuffer,RXBUFFERSIZE);
		errorCount_local=0;
		

	}
	//__set_PRIMASK(0); //开总中断

//	//超声波485
	if(huart->Instance==UART8){
		static float distance=0;
		static BaseType_t xHigherPriorityTaskWoken;
////每次接收两个字节
		distance= ultrasonic_uart_temp[0]*256 + ultrasonic_uart_temp[1];

		//最大距离
		if (distance>MAXDIS){
			distance = ultrasonic_uart_temp[1]*256 + ultrasonic_uart_temp[0];
			//TODO 仍然大于10，则报错
		}
		xQueueSendFromISR(rs485RxMsgQueueHandle,&distance,&xHigherPriorityTaskWoken);
		HAL_UART_Receive_IT(&huart8,(uint8_t *)ultrasonic_uart_temp,REC_LENGTH);
		
  }
	

}


/*USB 接收*/


//static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
//{

//  /* USER CODE BEGIN 6 */

//  USBD_CDC_SetRxBuffer(hUsbDevice_0, &Buf[0]);

//  USBD_CDC_ReceivePacket(hUsbDevice_0);

//  return (USBD_OK);

//}








//错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	
	if(USART6 == huart->Instance){
		errorCount_local++;
		if(errorCount_local>50){
			__set_FAULTMASK(1);// 关闭所有中断
			NVIC_SystemReset(); 
			errorCount_local=0;	
		}
		if (huart->ErrorCode & HAL_UART_ERROR_ORE){
			__HAL_UART_CLEAR_OREFLAG(&huart6);
		}
		//__HAL_UART_CLEAR_FEFLAG(&huart3);
		__HAL_UART_CLEAR_OREFLAG(&huart6);
		__HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_PE);
		__HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_FE);
		__HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_NE);
		__HAL_UART_CLEAR_FLAG(&huart6,UART_FLAG_ORE);
//		
		HAL_UART_Receive_IT(&huart6,(uint8_t*)aRxBuffer,RXBUFFERSIZE);
		printf("linux--stm32 uart error----- \r\n");
	}
	if(huart->Instance == UART8 && huart->ErrorCode){
		__HAL_UART_CLEAR_OREFLAG(&huart8);
		HAL_UART_Receive_IT(&huart8,(uint8_t *)ultrasonic_uart_temp,REC_LENGTH);
		printf("ultrasonic uart error--\r\n");
	}


}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	//__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);

}