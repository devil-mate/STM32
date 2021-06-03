#ifndef __STATUSMANAGER_H__
#define __STATUSMANAGER_H__
#include "stm32f2xx_hal.h"
#include "stdint.h"
#include "UartDriver.h"
typedef enum 
{
	MobileS=0,
	BaseS,
	LinkS
} SwitchMode_m;
//typedef enum 
//{
//	  S_4G = 0,
//    S_GPS1, //GPS2
//    S_GPS2, //GPS3
//    S_WDR,
//    S_BLE,
//    S_NULL = 0xFF //nonexist uart
//} LinkPort_m;
typedef enum
{
	RTK4G=0,
	RTKWDR
} RTKSource_m;
#pragma  pack (push,4) 
typedef struct 
{
	SwitchMode_m  GPSMode;
	RTKSource_m   RTKSource;
	uint8_t       FixPos;
	double        Latitude,Longitude,Altitude;
	double        AveTime;
	char					ServerIP[20];
	int           ServerPort;
	PortId_e 			InterfacePort;
	uint8_t				RadioSpeed;
	uint8_t       RadioChan;
}SysStatus_s;
#pragma pack(pop)
typedef struct 
{
	SwitchMode_m  SwitchMode;
	PortId_e      LinkPort;
	PortId_e      ReadPort;
}SysRamStatus_s;

extern SysStatus_s SysStatus;
extern SysRamStatus_s SysRamStatus;
void IniStatusManager(void);
HAL_StatusTypeDef WriteStatus();
void ReadStatus();
void ResetStatus();
#endif