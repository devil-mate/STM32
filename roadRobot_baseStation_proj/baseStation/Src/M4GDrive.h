#ifndef __M4GDRIVER_H__
#define __M4GDRIVER_H__
#include "UartDriver.h"
#include <stdbool.h>
typedef enum
{
    NotNet = 0,//网络未连接
    ActNet , //网络连接成功
    ActServer //服务器连接成功
}NetState;

extern OnRecvFun OnM4GRecvEx;
extern NetState  M4GState;
void Ini4G(void);
void On4GRecv(unsigned char * buf ,int len);
int  M4GSend(unsigned char * buf ,int len);
void Restart4G();
void M4GStatePuls();
int SetServerIP(char  * IP);
int SetServerPort(int  Port);
void IniDevTask(void * argument);
void Ini4GCommand();
//void M4G_CRC_test(unsigned char * buf ,int len);
void setRTKData(unsigned char * buf ,int len);

#define BASE_4G_TEST 
#define BASE_4G_TEST_SEND
#define BASE_4G_TEST_SR 
extern bool Test4GFlag;
#endif