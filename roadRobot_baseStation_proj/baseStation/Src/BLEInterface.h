#ifndef __BLEINTERFACE_H__
#define __BLEINTERFACE_H__
#include "stm32f2xx_hal.h"
#include "stdint.h"
#include "TaskList.h"
typedef enum
{
	CMDOK=0,
	CMDILLEGAL=1,
	PARAERROR=2,
	OUTTIME=3,
	REPLYERROR=4,
	ROBOTERROR=5,
	WRITEERROR=6
}CmdReply_e;

typedef enum
{
	GETGPS=1,
	GETDEVINF=2,
	
	SETRTKSOURCE=50,
	SETBASEPOS=51,
	SETAVETIME=52,
	SETSERVERPORT=53,
	SETSERVERIP=54,
	SETRADIO=55,
	SETGPSMODE=56,
	ERRORINF=99
}BLECmd_e;
#pragma pack(push)                   //?????? 
#pragma pack(4) 
typedef struct
{
   int GPSMode;
   int RTKSource;
   int fixPos;
   double        latitude,longitude,altitude;
  double         ave;
  char             ServerIP[20];
  int         ServerPort;
	int  RadioSpeed;
	int  RadioChan;
}SysStatusBack_s;
#pragma pack(pop)
void IniBLEInterface();
void BLEMessage(char * cmd , uint16_t len);
void BELGetGPS();
void replyInf(BLECmd_e cmd,CmdReply_e reply);
#endif