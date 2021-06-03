#ifndef __BLEDRIVER_H__
#define __BLEDRIVER_H__
#include "UartDriver.h"
extern OnRecvFun OnBLERecvEx;
void IniBLE(void);
void OnBLERecv(unsigned char * buf ,int len);
int  BLESend(unsigned char * buf ,int len);
void setBlueToothName(char * name);
#endif