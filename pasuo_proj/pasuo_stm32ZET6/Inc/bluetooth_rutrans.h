#ifndef _BLUETOOTH_RUTRANS_H_
#define _BLUETOOTH_RUTRANS_H_

#include "stdint.h"
#define BLUE_RINGBUF_MAX_SIZE 1024 //1k  

uint32_t Get_Ble_Ringbuf_Length(void);
uint8_t Bluetooth_Write_Byte(uint8_t data)  ;
uint8_t Bluetooth_Read_Byte(void);
void Flush_Bluetooth_Ring_buf(void);
uint32_t Bluetooth_Read(uint8_t *readBuf,uint32_t length,uint32_t offset);
void Bluetooth_Write(uint8_t *Buf,uint32_t length);

void bufferWrite(uint8_t *Buf,uint32_t length);
uint16_t bufferRead(uint8_t *readBuf,uint32_t length);
	

#endif
