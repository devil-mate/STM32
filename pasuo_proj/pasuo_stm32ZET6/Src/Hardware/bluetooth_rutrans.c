#include "bluetooth_rutrans.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"


/************************非队列缓冲区/数组************************/
//对tempRxBuff 读封装
#include "common.h"

RxBuf_s tempRxBuff;
/************************非队列缓冲区/数组************************/

/************************环形缓冲区************************/
static uint8_t bluetooth_ringbuf[BLUE_RINGBUF_MAX_SIZE]; //__attribute__ ((section ("EXRAM")));
static uint32_t blue_writeldx=0;
static uint32_t blue_readldx=0;
static uint32_t gBluetooth_ringbuf_length = 0;

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
//	OS_CRITICAL_ENTER();			//进入临界区(无法被中断打断)
//	CPU_CRITICAL_ENTER();
	taskENTER_CRITICAL();
	blue_readldx = blue_writeldx;
	gBluetooth_ringbuf_length = 0;
	taskEXIT_CRITICAL();	//freeRTos
//	CPU_CRITICAL_EXIT();	// ucos3
//	OS_CRITICAL_EXIT();				//退出临界区(可以被中断打断) ucos2
}

uint8_t Bluetooth_Write_Byte(uint8_t data)  
{  
//	CPU_SR_ALLOC();
	if(gBluetooth_ringbuf_length < BLUE_RINGBUF_MAX_SIZE)
	{
//		OS_CRITICAL_ENTER();			//进入临界区(无法被中断打断)
		bluetooth_ringbuf[blue_writeldx] = data;
		blue_writeldx = blue_next_data_handle(blue_writeldx);
		gBluetooth_ringbuf_length ++;
//		OS_CRITICAL_EXIT();				//退出临界区(可以被中断打断)
		//如果接收状态机处于延时状态，立刻取消延时并进入就绪
//	if(gBluetooth_delaying_flag == 1)
//			OSTimeDlyResume(BLE_SRV_TASK_PRIO);
		return 0;
	}
	else
	{
//		printf("缓存区满\r\n");
		return 1;
	}
}  

uint8_t Bluetooth_Read_Byte(void)  
{  	
	//CPU_SR_ALLOC();
	uint8_t res;
	if(gBluetooth_ringbuf_length>0)
	{
		//OS_CRITICAL_ENTER();			//进入临界区(无法被中断打断)
		//CPU_CRITICAL_ENTER();
		taskENTER_CRITICAL();
		res = bluetooth_ringbuf[blue_readldx];
		blue_readldx = blue_next_data_handle(blue_readldx); 
		gBluetooth_ringbuf_length --;
		taskEXIT_CRITICAL();
		//OS_CRITICAL_EXIT();				//退出临界区(可以被中断打断)
		//CPU_CRITICAL_EXIT();
		return res;
	}
	else
	{
//		printf("缓存为空\r\n");
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
/************************环形缓冲区************************/


/************************非队列缓冲区/数组************************/
/**
  * @brief  读取到RxBuffer 处理数据
  * @param  readBuf 数组
  *                      
  * @param  length 最大可读取长度，readBuf剩余空间.
  * 
  */
uint16_t bufferRead(uint8_t *readBuf,uint32_t length){
//	//////*************printf**********/////
// length	
//	for(int i=0;i<tempRxBuff.RxLen;i++){
//		printf(" %x, ",tempRxBuff.RxBuf[i]);
//	}
//	printf("\r\n");
//	//////*************printf**********/////
	if ( length<tempRxBuff.RxLen)
	{
		//taskENTER_CRITICAL(); //防止USB接收中断同时写
		memcpy(readBuf, tempRxBuff.RxBuf, length);
		
		memmove(tempRxBuff.RxBuf, tempRxBuff.RxBuf + length, tempRxBuff.RxLen - length);
		tempRxBuff.RxLen -= length;
		//taskEXIT_CRITICAL();
		return length;
		
	}
	else
	{
		//taskENTER_CRITICAL(); //防止USB接收中断同时写
		uint16_t Count = tempRxBuff.RxLen;
		memcpy(readBuf, tempRxBuff.RxBuf, tempRxBuff.RxLen);
		tempRxBuff.RxLen = 0;
		//taskEXIT_CRITICAL();
		return Count;
		
	}


}
void bufferWrite(uint8_t *Buf,uint32_t length){
		if(tempRxBuff.RxLen<RX_BUF_LEN-length){
			memcpy(tempRxBuff.RxBuf+tempRxBuff.RxLen,Buf,length);
			tempRxBuff.RxLen+=length;
		}
//		else{
//			tempRxBuff.RxLen=0;
//		}
}
/************************非队列缓冲区/数组************************/
