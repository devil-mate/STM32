#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <assert.h>
#include "ringbuf.h"



#define BUFFER_SIZE  512   //缓冲区的长度,可以修改
  


static u32 validLen;//已使用的数据长度
static u8* pHead = NULL;//环形存储区的首地址
static u8* pTail = NULL;//环形存储区的结尾地址
static u8* pValid = NULL;//已使用的缓冲区的首地址
static u8* pValidTail = NULL;//已使用的缓冲区的尾地址

// can 收发环形缓冲区


SemaphoreHandle_t TxRingBuffMutex;
SemaphoreHandle_t RxRingBuffMutex;

static CanTxBuffer_s CanTxMsgBuffer;//
static CanRxBuffer_s CanRxMsgBuffer;
/*
 * 初始化环形缓冲区
 * 环形缓冲区这里可以是malloc申请的内存,也可以是Flash存储介质
 * */
void initRingbuffer(void)
{
    if(pHead == NULL)
    {
        pHead = (u8*) malloc(BUFFER_SIZE);
    }
    pValid = pValidTail = pHead;
    pTail = pHead + BUFFER_SIZE;
    validLen = 0;
}

/*
 * function:向缓冲区中写入数据
 * param:@buffer 写入的数据指针
 *       @addLen 写入的数据长度
 * return:-1:写入长度过大
 *        -2:缓冲区没有初始化
 * */
int wirteRingbuffer(u8* buffer,u32 addLen)
{
    if(addLen > BUFFER_SIZE) return -2;
    if(pHead==NULL) return -1;
    //assert(buffer);

    //将要存入的数据copy到pValidTail处
    if(pValidTail + addLen > pTail)//需要分成两段copy
    {
        int len1 = pTail - pValidTail;
        int len2 = addLen - len1;
        memcpy( pValidTail, buffer, len1);
        memcpy( pHead, buffer + len1, len2);
        pValidTail = pHead + len2;//新的有效数据区结尾指针
    }else
    {
        memcpy( pValidTail, buffer, addLen);
        pValidTail += addLen;//新的有效数据区结尾指针
    }

    //需重新计算已使用区的起始位置
    if(validLen + addLen > BUFFER_SIZE)
    {
        int moveLen = validLen + addLen - BUFFER_SIZE;//有效指针将要移动的长度
        if(pValid + moveLen > pTail)//需要分成两段计算
        {
            int len1 = pTail - pValid;
            int len2 = moveLen - len1;
            pValid = pHead + len2;
        }else
        {
            pValid = pValid + moveLen;
        }
        validLen = BUFFER_SIZE;
    }else
    {
        validLen += addLen;
    }

    return 0;
}

/*
 * function:从缓冲区内取出数据
 * param   :@buffer:接受读取数据的buffer
 *          @len:将要读取的数据的长度
 * return  :-1:没有初始化
 *          >0:实际读取的长度
 * */
int readRingbuffer(u8* buffer,u32 len)
{
    if(pHead==NULL) return -1;

    //assert(buffer);

    if(validLen ==0) return 0;

    if( len > validLen) len = validLen;

    if(pValid + len > pTail)//需要分成两段copy
    {
        int len1 = pTail - pValid;
        int len2 = len - len1;
        memcpy( buffer, pValid, len1);//第一段
        memcpy( buffer+len1, pHead, len2);//第二段，绕到整个存储区的开头
        pValid = pHead + len2;//更新已使用缓冲区的起始
    }else
    {
        memcpy( buffer, pValid, len);
        pValid = pValid +len;//更新已使用缓冲区的起始
    }
    validLen -= len;//更新已使用缓冲区的长度

    return len;
}

/*
 * function:获取已使用缓冲区的长度
 * return  :已使用的buffer长度
 * */
u32 getRingbufferValidLen(void)
{
    return validLen;
}

/*
 * function:释放环形缓冲区
 * */
void releaseRingbuffer(void)
{
    if(pHead!=NULL) free(pHead);
    pHead = NULL;
}



/***环形缓冲区CanTxMsgBuffer******/
//TODO 锁/互斥锁
uint16_t next_data_index(uint16_t addr)     
{     
    return (addr+1) == CAN_MSG_BUF_LEN ? 0:(addr+1) ;     
} 

uint8_t TxRingBuff_writeOneMsg(const CanTxMsg_s canTxMsgs)  
{  

	if(CanTxMsgBuffer.buf_length < CAN_MSG_BUF_LEN)
	{
		xSemaphoreTake( TxRingBuffMutex, portMAX_DELAY );
		//CanTxMsgBuffer.msg[CanTxMsgBuffer.writeIdx] = canTxMsgs;
		memcpy(&CanTxMsgBuffer.msg[CanTxMsgBuffer.writeIdx],&canTxMsgs,sizeof(canTxMsgs));
		CanTxMsgBuffer.writeIdx = next_data_index(CanTxMsgBuffer.writeIdx);
		CanTxMsgBuffer.buf_length ++;
		xSemaphoreGive( TxRingBuffMutex ); 

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

uint8_t TxRingBuff_ReadOneMsg(CanTxMsg_s *canTxMsgs)  
{  	
	
	//uint8_t res;
	if(CanTxMsgBuffer.buf_length>0)
	{
		xSemaphoreTake( TxRingBuffMutex, portMAX_DELAY );
		memcpy(canTxMsgs,&CanTxMsgBuffer.msg[CanTxMsgBuffer.readIdx],sizeof(CanTxMsgBuffer.msg[CanTxMsgBuffer.readIdx]));
		CanTxMsgBuffer.readIdx = next_data_index(CanTxMsgBuffer.readIdx); 
		//memcpy(canTxMsgs,&CanTxMsgBuffer.msg[0],sizeof(CanTxMsgBuffer.msg[0]));
		//canTxMsgs->header.DataLength = FDCAN_DLC_BYTES_8;
		CanTxMsgBuffer.buf_length --;
		xSemaphoreGive( TxRingBuffMutex ); 
		return 0;
	}
	else
	{
//		printf("缓存为空\r\n");
		return 1;
	}
}  

/***环形缓冲区CanRxMsgBuffer******/

uint8_t RxRingBuff_writeOneMsg(const CanRxMsg_s canRxMsgs)  
{  
	static BaseType_t xHigherPriorityTaskWoken;
	if(CanRxMsgBuffer.buf_length < CAN_MSG_BUF_LEN)
	{
		
		//xSemaphoreTakeFromISR( RxRingBuffMutex, &xHigherPriorityTaskWoken);
		CanRxMsgBuffer.msg[CanRxMsgBuffer.writeIdx] = canRxMsgs;
		CanRxMsgBuffer.writeIdx = next_data_index(CanRxMsgBuffer.writeIdx);
		CanRxMsgBuffer.buf_length ++;
		//xSemaphoreGiveFromISR( RxRingBuffMutex,&xHigherPriorityTaskWoken ); 
		return 0;
	}
	else
	{

		return 1;
	}
}  

uint8_t RxRingBuff_ReadOneMsg(CanRxMsg_s *canRxMsgs)  
{  	
	uint8_t res;
	
	if(CanRxMsgBuffer.buf_length>0)
	{	
		//xSemaphoreTake( RxRingBuffMutex, portMAX_DELAY );
		memcpy(canRxMsgs,&CanRxMsgBuffer.msg[CanRxMsgBuffer.readIdx],sizeof(CanRxMsgBuffer.msg[CanRxMsgBuffer.readIdx]));
		CanRxMsgBuffer.readIdx = next_data_index(CanRxMsgBuffer.readIdx); 
		CanRxMsgBuffer.buf_length --;
		//xSemaphoreGive( RxRingBuffMutex ); 
		return 0;
	}
	else
	{
		return 1;
	}
	
} 
uint16_t getRxRingBuffLength(){
	return CanRxMsgBuffer.buf_length;
}


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
	//taskENTER_CRITICAL();
	blue_readldx = blue_writeldx;
	gBluetooth_ringbuf_length = 0;
	//taskEXIT_CRITICAL();	//freeRTos
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
	remainLen=BLUE_RINGBUF_MAX_SIZE-gBluetooth_ringbuf_length;
	
	if(length > remainLen){
		length=remainLen;	
	}
	for(int i=0;i<length;i++){
		Bluetooth_Write_Byte(*(Buf+i));
	}

	
	
}
/************************环形缓冲区************************/



/************************非队列缓冲区/数组************************/










