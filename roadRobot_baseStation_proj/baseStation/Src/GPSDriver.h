#ifndef __GPSDRIVER_H__
#define __GPSDRIVER_H__

#include "UartDriver.h"
#include "StatusManager.h"
#include "stdint.h"
#define BASE_4G_TEST_HEAD (2021.520f)
typedef enum
{
	NOPOSITION=0,
	POSITION=1,
	RTKPOSITION=2
}PositionStatus_e;

void IniGPS(void);
void OnRTKRecv(unsigned char * buf ,int len);
void OnGPSRecv(unsigned char * buf ,int len);
void OnGPS1Recv(unsigned char * buf ,int len);
int  GPS1Send(unsigned char * buf ,int len);
int  GPS2Send(unsigned char * buf ,int len);
int  GetRTK(unsigned char * buf ,int len);
int SetBaseMode();
int SetMobileMode();
int RestartGPS();
int SetAveTime(double Time);
int SetRTKSource(RTKSource_m Source);
int SetFixPos(double BLH[3]);
void GPSPPSPlus();
PositionStatus_e GetGPS(double *B,double *L,double *H);
extern OnRecvFun OnGPSRecvEx;
extern OnRecvFun OnRTKRecvEx;
extern OnRecvFun OnGPSSetRecvEx;
extern double CurLatitude,CurLongitude ,CurAltitude,CurTime;
extern int CurRTK;
#endif