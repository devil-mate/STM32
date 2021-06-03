#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_
#include <stdint.h>
#include "can_handler.h"
#define CAN_MSG_BUF_LEN 32

typedef struct 
{
    CanTxMsg_s              msg[CAN_MSG_BUF_LEN];
    uint16_t                writeIdx;
	uint16_t                readIdx;
	uint16_t				buf_length;
} CanTxBuffer_s;

typedef struct 
{
    CanRxMsg_s              msg[CAN_MSG_BUF_LEN];
    uint16_t                writeIdx;
	uint16_t                readIdx;
	uint16_t				buf_length;
} CanRxBuffer_s;

typedef unsigned char u8;
typedef unsigned int u32;

void initRingbuffer(void);
int wirteRingbuffer(u8* buffer,u32 len);
int readRingbuffer(u8* buffer,u32 len);
u32 getRingbufferValidLen(void);
void releaseRingbuffer(void);

extern SemaphoreHandle_t TxRingBuffMutex;
extern SemaphoreHandle_t RxRingBuffMutex;
uint8_t 	TxRingBuff_ReadOneMsg(CanTxMsg_s *canTxMsgs);
uint8_t 	TxRingBuff_writeOneMsg(const CanTxMsg_s canTxMsgs);
uint8_t 	RxRingBuff_ReadOneMsg(CanRxMsg_s *canRxMsgs);
uint8_t 	RxRingBuff_writeOneMsg(const CanRxMsg_s canRxMsgs);

uint16_t	getRxRingBuffLength();



#define BLUE_RINGBUF_MAX_SIZE 1024 //1k  

uint32_t Get_Ble_Ringbuf_Length(void);
uint8_t Bluetooth_Write_Byte(uint8_t data)  ;
uint8_t Bluetooth_Read_Byte(void);
void Flush_Bluetooth_Ring_buf(void);
uint32_t Bluetooth_Read(uint8_t *readBuf,uint32_t length,uint32_t offset);
void Bluetooth_Write(uint8_t *Buf,uint32_t length);

void bufferWrite(uint8_t *Buf,uint32_t length);
uint16_t bufferRead(uint8_t *readBuf,uint32_t length);

#endif /* RINGBUFFER_H_ */
