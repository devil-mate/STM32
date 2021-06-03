#ifndef __USBDRIVE_H
#define __USBDRIVE_H
#include "usbd_cdc_if.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "USBData.h"
#ifdef __cplusplus
extern "C" {
#endif 
#pragma pack(push)                   //?????? 
#pragma pack(1) 
typedef struct
{
	unsigned char 	fresh;
	unsigned char 	sending;
	unsigned short 	len;
	unsigned short  bufSize;
	unsigned char 	* data;
}USBBufCell;
typedef struct
{
	unsigned char 	cellSize;
	unsigned char 	cellIn;
	unsigned char 	cellOut;
	USBBufCell * cells;
	unsigned char 	* sendBuf;
}USBChannel;
#pragma pack(pop)
extern const unsigned char CELL_SIZE[];
extern const unsigned short CELL_BUF_SIZE[];
extern const unsigned char USB_CHANNEL_SIZE;
extern USBChannel usbBuf[];
typedef unsigned char (* OnUSBRecvFunc)(unsigned char * buf ,int len);

void DisableAllIRQ();
void EnableAllIRQ();
void iniUSBDriver();
void sendUSBBuf();
void StatusWatcherTask(void * argument);
char sendUSBData(unsigned char channel,unsigned short 	len,unsigned char 	* data);
void getUSBData(uint8_t* buf, uint32_t * len);
void onUSBDataRecv(unsigned char code,unsigned short 	len,unsigned char 	* data);
char sendChannelData(unsigned char channel,unsigned char code,unsigned short 	len,unsigned char 	* data);
int USBPrintf(const char *format, ...);
void setUSBCallBackFunc(USB_Down_Code code,OnUSBRecvFunc func);
void sendLiveFrame(int index);
#ifdef __cplusplus
}
#endif 
#endif // __FORWARD_CONTROL_H