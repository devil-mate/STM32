#ifndef __USB_TYPE_H
#define __USB_TYPE_H

#include <stdint.h>

#define USB_UART_BUF_LEN (1024)

#pragma pack(push,1)

typedef struct 
{
    uint8_t 	RxBuf[USB_UART_BUF_LEN];
    uint8_t 	TxBuf[USB_UART_BUF_LEN];
    uint16_t    ReadLen;	
    uint16_t   	SendLen;
}USBUart_s;

#pragma pack(pop)

#define IdleLen(u) (USB_UART_BUF_LEN - u.ReadLen)

extern USBUart_s USBUart;

#endif
