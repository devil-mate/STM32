#ifndef _BLUETOOTH_RUTRANS_H_
#define _BLUETOOTH_RUTRANS_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

#include "stdint.h"
#define BLUE_RINGBUF_MAX_SIZE 1024 //1k  
#define RXBUFFERSIZE   20 //缓存大小
extern uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

uint32_t Get_Ble_Ringbuf_Length(void);
uint8_t Bluetooth_Write_Byte(uint8_t data)  ;
uint8_t Bluetooth_Read_Byte(void);
void Flush_Bluetooth_Ring_buf(void);
uint32_t Bluetooth_Read(uint8_t *readBuf,uint32_t length,uint32_t offset);
void Bluetooth_Write(uint8_t *Buf,uint32_t length);

void bufferWrite(uint8_t *Buf,uint32_t length);
uint16_t bufferRead(uint8_t *readBuf,uint32_t length);
//	
//uint16_t DMA_SendData(UART_HandleTypeDef *huart,uint8_t *buf,uint16_t len);
//uint16_t DMA_ReadData(uint8_t *buf,uint16_t len);
uint16_t USART_SendData(UART_HandleTypeDef *huart,uint8_t *buf,uint16_t len);
uint16_t USART_ReadData(uint8_t *buf,uint16_t len);
//uint8_t CheckSum(uint8_t *Data, uint32_t DataLen);
uint8_t CheckSum(uint8_t *Data, uint32_t DataLen);



#endif
