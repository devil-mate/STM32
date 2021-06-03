#include "bluetooth_rutrans.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stm32f4xx_hal.h"


/************************�Ƕ��л�����/����************************/
//��tempRxBuff ����װ
#include "common.h"
extern UART_HandleTypeDef huart5;
//UART_HandleTypeDef huart3;
uint8_t aRxBuffer[RXBUFFERSIZE];
SemaphoreHandle_t BinarySem_Rx = NULL;

RxBuf_s tempRxBuff;
/************************�Ƕ��л�����/����************************/

/************************���λ�����************************/
static uint8_t bluetooth_ringbuf[BLUE_RINGBUF_MAX_SIZE]; //__attribute__ ((section ("EXRAM")));
static uint32_t blue_writeldx=0;
static uint32_t blue_readldx=0;
static uint32_t gBluetooth_ringbuf_length = 0;

static int32_t usartError=0;
static HAL_UART_StateTypeDef usartSate=0;

//public
uint16_t DMA_SendData(UART_HandleTypeDef *huart,uint8_t *buf,uint16_t len)
{
	static uint16_t counter=0;
	taskENTER_CRITICAL();
	if(HAL_UART_Transmit_DMA(huart, buf,len)!= HAL_OK){
		usartError = huart->ErrorCode;
		usartSate= huart->gState;
		
            //Error_Handler();
		
		counter++;
		return 0;
	}
	taskEXIT_CRITICAL();
	//TODO �жϴ������
//	while(1){
//		if(__HAL_DMA_GET_FLAG(&huart3,DMA_FLAG_TCIF3_7)){
//			__HAL_DMA_CLEAR_FLAG(&huart3,DMA_FLAG_TCIF3_7);
//			huart3.gState =HAL_UART_STATE_READY;
//			break;
//		}
//	}
	return len;

}

uint16_t DMA_ReadData(uint8_t *Buf,uint16_t IdleLen)
{
	//���´�DMA����
	//HAL_UART_DMAStop(&huart3); //
	uint16_t resultLen;
	if (Buf == NULL){
        return 0;
	}
	/*****ʹ�û��λ�����*****/
	//resultLen=Bluetooth_Read(Buf,IdleLen,0);
	
	/*****��ʹ�û��λ�����*****/
	resultLen= bufferRead(Buf,IdleLen);

	return resultLen;	

}
uint16_t USART_SendData(UART_HandleTypeDef *huart,uint8_t *buf,uint16_t len)
{

	if(HAL_UART_Transmit(huart, buf,len,0xFF)!= HAL_OK){
        //Error_Handler();
		return 0;
	}
		
	return len;
	

}
uint16_t USART_ReadData(uint8_t *Buf,uint16_t IdleLen)
{
	uint16_t resultLen;
	if (Buf == NULL){
        return 0;
	}
	/*****ʹ�û��λ�����*****/
	//resultLen=Bluetooth_Read(Buf,IdleLen,0);
	/*****��ʹ�û��λ�����*****/
	resultLen= bufferRead(Buf,IdleLen);
	return resultLen;	
//	if (Buf == NULL){
//		
//        return 0;
//	}
//    if ( IdleLen<USBUart.ReadLen)
//    {
//        taskENTER_CRITICAL(); //��ֹUSB�����ж�ͬʱд
//        memcpy(Buf, USBUart.RxBuf, IdleLen);
//        USBUart.ReadLen -= IdleLen;
//        memmove(USBUart.RxBuf, USBUart.RxBuf + IdleLen, USBUart.ReadLen - IdleLen);
//        taskEXIT_CRITICAL();
//        return IdleLen;
//		
//    }
//    else
//    {
//        taskENTER_CRITICAL(); //��ֹUSB�����ж�ͬʱд
//        uint16_t Count = USBUart.ReadLen;
//        memcpy(Buf, USBUart.RxBuf, USBUart.ReadLen);
//        USBUart.ReadLen = 0;
//        taskEXIT_CRITICAL();
//        return Count;
//		
//    }
}


/**
 * @description: sum from index [0, DataLen-1]
 * @param {type} 
 * @return: 
 */
uint8_t CheckSum(uint8_t *Data, uint32_t DataLen)
{
    uint8_t sum = 0;
	if(DataLen>=RX_BUF_LEN){
		return 0;
	}
    for (int i = 0; i < DataLen; i++)
    {
        sum += Data[i];
    }
    return sum;
}


uint32_t Get_Ble_Ringbuf_Length(void)
{
	return gBluetooth_ringbuf_length;
}
 
uint32_t blue_next_data_handle(uint32_t addr)     
{     
    return (addr+1) == BLUE_RINGBUF_MAX_SIZE ? 0:(addr+1) ;     
}     

void Flush_Bluetooth_Ring_buf(void)
{
//	CPU_SR_ALLOC(); //
//	OS_CRITICAL_ENTER();			//�����ٽ���(�޷����жϴ��)
//	CPU_CRITICAL_ENTER();
	taskENTER_CRITICAL();
	blue_readldx = blue_writeldx;
	gBluetooth_ringbuf_length = 0;
	taskEXIT_CRITICAL();	//freeRTos
//	CPU_CRITICAL_EXIT();	// ucos3
//	OS_CRITICAL_EXIT();				//�˳��ٽ���(���Ա��жϴ��) ucos2
}

uint8_t Bluetooth_Write_Byte(uint8_t data)  
{  
//	CPU_SR_ALLOC();
	if(gBluetooth_ringbuf_length < BLUE_RINGBUF_MAX_SIZE)
	{
//		OS_CRITICAL_ENTER();			//�����ٽ���(�޷����жϴ��)
		bluetooth_ringbuf[blue_writeldx] = data;
		blue_writeldx = blue_next_data_handle(blue_writeldx);
		gBluetooth_ringbuf_length ++;
//		OS_CRITICAL_EXIT();				//�˳��ٽ���(���Ա��жϴ��)
		//�������״̬��������ʱ״̬������ȡ����ʱ���������
//	if(gBluetooth_delaying_flag == 1)
//			OSTimeDlyResume(BLE_SRV_TASK_PRIO);
		return 0;
	}
	else
	{
//		printf("��������\r\n");
		return 1;
	}
}  

uint8_t Bluetooth_Read_Byte(void)  
{  	
	//CPU_SR_ALLOC();
	uint8_t res;
	if(gBluetooth_ringbuf_length>0)
	{
		//OS_CRITICAL_ENTER();			//�����ٽ���(�޷����жϴ��)
		//CPU_CRITICAL_ENTER();
		taskENTER_CRITICAL();
		res = bluetooth_ringbuf[blue_readldx];
		blue_readldx = blue_next_data_handle(blue_readldx); 
		gBluetooth_ringbuf_length --;
		taskEXIT_CRITICAL();
		//OS_CRITICAL_EXIT();				//�˳��ٽ���(���Ա��жϴ��)
		//CPU_CRITICAL_EXIT();
		return res;
	}
	else
	{
//		printf("����Ϊ��\r\n");
		return 0x00;
	}
}  

uint32_t Bluetooth_Read(uint8_t *readBuf,uint32_t length,uint32_t offset)
{
	uint32_t i;
	
	if(length > gBluetooth_ringbuf_length)
	{
		length = gBluetooth_ringbuf_length;
	}

	for(i = 0;i<length;i++)
	{
		readBuf[i] = Bluetooth_Read_Byte();
	}
	return length;
	
}
void Bluetooth_Write(uint8_t *Buf,uint32_t length)
{
	uint32_t i, remainLen;
	//taskENTER_CRITICAL();
	remainLen=BLUE_RINGBUF_MAX_SIZE-gBluetooth_ringbuf_length;
	
	if(length > remainLen){
		length=remainLen;	
	}
	for(int i=0;i<length;i++){
		Bluetooth_Write_Byte(*(Buf+i));
	}
	//taskEXIT_CRITICAL();
	
	
}
/************************���λ�����************************/


/************************�Ƕ��л�����/����************************/

uint16_t bufferRead(uint8_t *readBuf,uint32_t length){
//	//////*************printf**********/////
//	for(int i=0;i<tempRxBuff.RxLen;i++){
//		printf(" %x, ",tempRxBuff.RxBuf[i]);
//	}
//	printf("\r\n");
//	//////*************printf**********/////
	if ( length<tempRxBuff.RxLen)
	{
		taskENTER_CRITICAL(); //��ֹUSB�����ж�ͬʱд
		memcpy(readBuf, tempRxBuff.RxBuf, length);
		
		memmove(tempRxBuff.RxBuf, tempRxBuff.RxBuf + length, tempRxBuff.RxLen - length);
		tempRxBuff.RxLen -= length;
		taskEXIT_CRITICAL();
		return length;
		
	}
	else
	{
		taskENTER_CRITICAL(); //��ֹUSB�����ж�ͬʱд
		uint16_t Count = tempRxBuff.RxLen;
		memcpy(readBuf, tempRxBuff.RxBuf, tempRxBuff.RxLen);
		tempRxBuff.RxLen = 0;
		taskEXIT_CRITICAL();
		return Count;
		
	}


}
void bufferWrite(uint8_t *Buf,uint32_t length){
		if(tempRxBuff.RxLen<RX_BUF_LEN){
			memcpy(tempRxBuff.RxBuf+tempRxBuff.RxLen,Buf,length);
			tempRxBuff.RxLen+=length;
		}
}
/************************�Ƕ��л�����/����************************/



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	//__set_PRIMASK(1); //�����ж�
	if(UART5 == huart->Instance){
		//TODO  xHigherPriorityTaskWoken ���������ڱ����Ƿ��и����ȼ�����׼��������
		//�������ִ����Ϻ󣬴˲�������ֵ��pdTRUE��˵���и����ȼ�����Ҫִ�У�����û�С�
		static BaseType_t xHigherPriorityTaskWoken;
//		if(USBUart.ReadLen<USB_UART_BUF_LEN){
//			memcpy(USBUart.RxBuf+USBUart.ReadLen,aRxBuffer,RXBUFFERSIZE);
//			USBUart.ReadLen+=RXBUFFERSIZE;
//		}
		//-----���뻷�λ�����
		//Bluetooth_Write(aRxBuffer,RXBUFFERSIZE);
		//д�뻺�������ǻ��λ�������
		bufferWrite(aRxBuffer,RXBUFFERSIZE);
		xSemaphoreGiveFromISR(BinarySem_Rx,&xHigherPriorityTaskWoken);
		//�жϽ���	
		//HAL_UART_Receive_IT(&huart3,(uint8_t*)aRxBuffer,RXBUFFERSIZE);

	}
//	if(USART3 == huart->Instance){
//	
//	}
	//__set_PRIMASK(0); //�����ж�

  
	
	

}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	
	if(UART5 == huart->Instance){
		if (huart->ErrorCode & HAL_UART_ERROR_ORE){
			__HAL_UART_CLEAR_OREFLAG(huart);
			HAL_UART_Receive_IT(&huart5,(uint8_t*)aRxBuffer,RXBUFFERSIZE);
		}
	}

}
//TODO ����IDELE�ж�--�ص��У�clear ��disnabel DMA