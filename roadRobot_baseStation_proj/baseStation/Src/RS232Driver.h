#ifndef __RS232DRIVER_H__
#define __RS232DRIVER_H__
#include "UartDriver.h"
extern OnRecvFun OnRS232RecvEx;
void IniRS232(void);
void OnRS23Recv(unsigned char * buf ,int len);
int  RS232Send(unsigned char * buf ,int len);
#endif