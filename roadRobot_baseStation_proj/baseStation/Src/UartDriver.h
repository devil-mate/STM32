#ifndef __UARTDRIVE_H
#define __UARTDRIVE_H

#include "stm32f2xx_hal.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif 
//#include "ring_buf.h"
#define DMA_BUF_SIZE (512)
#define SEND_BUF_SIZE (1024*2)
#define READBUFSIZE (2)
typedef void (* OnRecvFun)(unsigned char * buf ,int len);

typedef struct
{
    uint8_t buf[DMA_BUF_SIZE];
    uint16_t len;
} UartDMABuf_t;

typedef enum
{
    Uart_4G = 0,
    Uart_GPS1, //GPS2
    Uart_GPS2, //GPS3
    Uart_WDR,
    Uart_232,
    Uart_BLE,
    Uart_NULL 
} PortId_e;

typedef struct 
{
    UART_HandleTypeDef *huart;
    char *desc;
    UartDMABuf_t dmaRxBuf[READBUFSIZE];
    UartDMABuf_t dmaTxBuf;
		GPIO_TypeDef *CTR485;
		uint16_t     PIN485;
	
		unsigned char RxBufIndex;
		uint16_t SendHead;//未发送完成头
		uint16_t SendTail;//下一次写入的位置
		uint16_t SendFull;
		uint16_t SendPre;//本次发送完成后的头
		unsigned char SendBuf[SEND_BUF_SIZE];
		void (* OnUartRecv)(unsigned char * buf ,int len);
		uint8_t FullCounter;//for send buffer full
		uint8_t IdeaActve;//for Idea
} ComPort_t;

#define __ENUM_STR(e) #e
#define ENUM_STR(e) __ENUM_STR(e)
void InitUart();
int UartSendData(ComPort_t *ComPort,unsigned char * buf ,int len);
char  UartSetParameter(ComPort_t *ComPort,int rate);
int UartPrintf(const char *format, ...);
char SetPrintPort(uint8_t port);
extern ComPort_t ComPorts[Uart_NULL];
#ifdef __cplusplus
}
#endif 
#endif // __FORWARD_CONTROL_H
