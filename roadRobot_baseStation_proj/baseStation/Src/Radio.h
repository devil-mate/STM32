#ifndef __RADIO_H__
#define __RADIO_H__
#include "UartDriver.h"
typedef enum 
{
	AIRSPDK3=0,
	AIRSPD1K2=1,
	AIRSPD2K4=2,
	AIRSPD4K8=3,
	AIRSPD9K6=4,
	AIRSPD19K2=5
} AirSpeed_m;
extern OnRecvFun OnRadioRecvEx;

void IniRadio(void);
void OnRadioRecv(unsigned char * buf ,int len);
char RadioConfig(unsigned char cmd[6]);
int  RadioSend(unsigned char * buf ,int len);
char SetAirSpeed(char mode);
char SetRadioTest(char mode);
char SetRadioConfig(uint8_t speed,uint8_t chan);
void radio_test(unsigned char * buf ,int len);


#endif