#include "main.h"


#include "cmsis_os.h"
#include <string.h>

#include "can_handler.h"
#include "ringbuf.h"

#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#define RXBUFFERSIZE   20 //�����С
#define MAXDIS 12000.0   //mm

extern SemaphoreHandle_t BinarySem_Rx;
extern QueueHandle_t      rs485RxMsgQueueHandle;
int16_t errorCount_local=0;
extern uint8_t aRxBuffer[];//HAL��ʹ�õĴ��ڽ��ջ���


extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart8;
uint8_t  ultrasonic_uart_temp[REC_LENGTH] = {0};       //USART�������ݻ���
/* can �жϽ��� */

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
		HAL_RetVal = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data); //��ȡ���յ�������
		if (HAL_OK == HAL_RetVal)
		{
//            /*д�뵽RxBuffer*/
			//TODO ָ��,�Լ�memcpy����
			static CanRxMsg_s tempCanRxMsgs;
			memcpy(&tempCanRxMsgs.header,&header,sizeof(tempCanRxMsgs.header));
			memcpy(&tempCanRxMsgs.data,data,sizeof(tempCanRxMsgs.data));
			RxRingBuff_writeOneMsg(tempCanRxMsgs);
			//sFlag_g.canInitReciveFlag=true;
			xSemaphoreGiveFromISR(BinarySem_CanRx,&xHigherPriorityTaskWoken);
		}
		
	}
	
}

// ���ڽ����жϻص�


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(USART6 == huart->Instance){
		//static uint8_t aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���
//		//TODO  xHigherPriorityTaskWoken ���������ڱ����Ƿ��и����ȼ�����׼��������
//		//�������ִ����Ϻ󣬴˲�������ֵ��pdTRUE��˵���и����ȼ�����Ҫִ�У�����û�С�
		static BaseType_t xHigherPriorityTaskWoken;
//		//���뻷�λ�����
		Bluetooth_Write(aRxBuffer,RXBUFFERSIZE);
//		//д�뻺�������ǻ��λ�������
		//bufferWrite(aRxBuffer,RXBUFFERSIZE);
		xSemaphoreGiveFromISR(BinarySem_Rx,&xHigherPriorityTaskWoken);
	//�жϽ���	
		HAL_UART_Receive_IT(&huart6,(uint8_t*)aRxBuffer,RXBUFFERSIZE);
		errorCount_local=0;
		

	}
	//__set_PRIMASK(0); //�����ж�

//	//������485
	if(huart->Instance==UART8){
		static float distance=0;
		static BaseType_t xHigherPriorityTaskWoken;
////ÿ�ν��������ֽ�
		distance= ultrasonic_uart_temp[0]*256 + ultrasonic_uart_temp[1];

		//������
		if (distance>MAXDIS){
			distance = ultrasonic_uart_temp[1]*256 + ultrasonic_uart_temp[0];
			//TODO ��Ȼ����10���򱨴�
		}
		xQueueSendFromISR(rs485RxMsgQueueHandle,&distance,&xHigherPriorityTaskWoken);
		HAL_UART_Receive_IT(&huart8,(uint8_t *)ultrasonic_uart_temp,REC_LENGTH);
		
  }
	

}


/*USB ����*/


//static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
//{

//  /* USER CODE BEGIN 6 */

//  USBD_CDC_SetRxBuffer(hUsbDevice_0, &Buf[0]);

//  USBD_CDC_ReceivePacket(hUsbDevice_0);

//  return (USBD_OK);

//}








//����ص�
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	
	if(USART6 == huart->Instance){
		errorCount_local++;
		if(errorCount_local>50){
			__set_FAULTMASK(1);// �ر������ж�
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