#include "BLEInterface.h"
#include "UserInterface.h"
#include "CRC.h"
#include "BLEDriver.h"
#include "GPSDriver.h"
#include "Radio.h"
#include "UartDriver.h"
 #include "M4GDrive.h"

#define CMDBUFSIZE 512
#define FRAMEHEAD (0xB3BB)
#define FRAMETAIL (0xD3DD)
#define BLE_BUF_SIZE (512)
void belSetGPSMode(int mode);
uint8_t bleCmdBuf[BLE_BUF_SIZE];
uint16_t bleBufSize=0;
uint16_t bleBufTail=0;

typedef enum
{
	FRAMEERROR=0,
	VERIFYERROR=2,
	TYPEERROR=3
}BLECmdErr_e;


typedef struct
{
	uint16_t head;
	uint16_t size;
	uint32_t type;
}BLEHead;

typedef struct
{
	uint16_t CrcVerify;
	uint16_t Tail;
}BLETail;

#pragma pack(push, 4)
typedef struct
{
	double  latitude;
	double  longitude ;
	double  altitude;
	int   PositionStatus;
}GPSInf_s;
#pragma pack(pop)

typedef struct
{
	char type;
	char keep;
}SetRTKSource_s;

uint16_t cmdLen;
void BELGetDevInf();
void parseBLEData(void * para);
void sendErrorInf(int error);

void IniBLEInterface()
{

}

void BLEMessage(char * cmd , uint16_t len)
{
	if(len<(CMDBUFSIZE-bleBufSize))
	{
		memcpy(bleCmdBuf+bleBufSize,cmd,len);
		bleBufSize+=len;
	}
	else
	{
		bleBufSize=0;
	}
	cmdLen=len;
	UartPrintf("BEL RECV size %d len %d\r\n",bleBufSize,len);
	AddTaskFromISR(UIPerformer,parseBLEData,bleCmdBuf);
}

void parseBLEData(void * para)
{
	BLEHead * frameHead;
	BLETail * frameTail;
	uint16_t  CrcVerify;
	int16_t  frameHeadPos=-1;
	int16_t  frameTailPos=-1;
	int16_t  searchEnd=bleBufSize-sizeof(BLEHead);
	if(searchEnd<=0)
		return;
	for(uint16_t pos =0;pos<searchEnd;pos++)
	{
		frameHead=(BLEHead *)(bleCmdBuf+pos);
		if(frameHead->head==FRAMEHEAD)
		{
			frameHeadPos=pos;
			break;
		}			
	}
	if(frameHeadPos!=-1)
	{
		if(frameHead->size+frameHeadPos<=bleBufSize)
		{
				frameTail=(BLETail *)(((uint8_t *)frameHead)+frameHead->size-sizeof(BLETail));
				if(frameTail->Tail!=FRAMETAIL)
				{
					sendErrorInf(FRAMEERROR);
					bleBufSize=0;
				}
				else
				{
					  frameTailPos=frameHead->size+frameHeadPos;
						CrcVerify=CRC_Cal((unsigned char *)&(frameHead->size),frameHead->size-6);
						if(CrcVerify!=frameTail->CrcVerify)
						{
							sendErrorInf(VERIFYERROR);
							bleBufSize=0;
						}
						else
						{
							switch (frameHead->type)
							{
								case GETGPS:
										BELGetGPS();
									break;
								case GETDEVINF:
										BELGetDevInf();
									break;
								case SETRTKSOURCE:
										SetRTKSource(*(uint8_t *)(((uint8_t*)frameHead)+sizeof(BLEHead)));
									break;
								case SETBASEPOS:
										SetFixPos((double *)(((uint8_t*)frameHead)+sizeof(BLEHead)));
									break;
								case SETAVETIME:
										SetAveTime(*(double *)(((uint8_t*)frameHead)+sizeof(BLEHead)));
									break;
								case SETSERVERIP:
										 SetServerIP((char  *) (((uint8_t*)frameHead)+sizeof(BLEHead)));
									break;
								case SETSERVERPORT:
										 SetServerPort(*((int  * ) (((uint8_t*)frameHead)+sizeof(BLEHead))));
									break;
								case SETRADIO:
										SetRadioConfig((uint8_t   ) *(((uint8_t*)frameHead)+sizeof(BLEHead)),(uint8_t   ) *(((uint8_t*)frameHead)+1+sizeof(BLEHead)));
									break;
								case SETGPSMODE:
										belSetGPSMode((uint32_t   ) *(((uint8_t*)frameHead)+sizeof(BLEHead)));
									break;								
								default:
									sendErrorInf(TYPEERROR);
									break;
							}
						}
				}
		}
	}
	if(frameTailPos!=-1)
	{
		memmove(bleBufSize,bleBufSize+frameTailPos,bleBufSize-frameTailPos);
		bleBufSize=bleBufSize-frameTailPos;
	}
	else if(frameHeadPos!=-1)
	{
		memmove(bleBufSize,bleBufSize+frameHeadPos,bleBufSize-frameHeadPos);
		bleBufSize=bleBufSize-frameHeadPos;
	}
	else
	{
		memmove(bleBufSize,bleBufSize+searchEnd,bleBufSize-searchEnd);
		bleBufSize=bleBufSize-searchEnd;		
	}
}
void belSetGPSMode(int mode)
{
	if(mode==0)
		SetMobileMode();
	else if(mode==1)
		SetBaseMode();
	else
		replyInf(SETGPSMODE,PARAERROR);
}
void sendErrorInf(int error)
{
	char Inf[sizeof(BLEHead)+sizeof(BLETail)+sizeof(int32_t)];
	BLEHead * frameHead=(BLEHead *)Inf;
	BLETail * frameTail=(BLETail *)(Inf+sizeof(BLEHead)+sizeof(int32_t));
	int32_t * errorInf= (int32_t *)(Inf+sizeof(BLEHead));
	frameHead->head=FRAMEHEAD;
	frameHead->size=sizeof(BLEHead)+sizeof(BLETail)+sizeof(int32_t);
	frameHead->type=ERRORINF;
	*errorInf=error;
	frameTail->CrcVerify=CRC_Cal((unsigned char *)&(frameHead->size),sizeof(BLEHead)-sizeof(uint8_t)*2+sizeof(int32_t));
	frameTail->Tail=FRAMETAIL;
	BLESend(Inf,sizeof(Inf));
}
void BELGetGPS()
{
	static char Inf[sizeof(BLEHead)+sizeof(BLETail)+sizeof(GPSInf_s)];
	BLEHead *frameHead=(BLEHead*)Inf;
	BLETail *frameTail=(BLETail*) (Inf+sizeof(BLEHead)+sizeof(GPSInf_s));
	GPSInf_s *GPSInf=(GPSInf_s *)(Inf+sizeof(BLEHead));
	frameHead->head=FRAMEHEAD;
	frameHead->size=sizeof(BLEHead)+sizeof(BLETail)+sizeof(GPSInf_s);
	frameHead->type=GETGPS;
	GPSInf->PositionStatus=GetGPS(&(GPSInf->latitude),&(GPSInf->longitude),&(GPSInf->altitude));
	frameTail->CrcVerify=CRC_Cal((unsigned char *)&(frameHead->size),sizeof(BLEHead)-sizeof(uint16_t)+sizeof(GPSInf_s));
	frameTail->Tail=FRAMETAIL;
//	UartPrintf("BEL :");
//	for(int n=0;n<sizeof(Inf);n++)
//	{
//		UartPrintf("0x%02X ", Inf[n]);
//	}
//	UartPrintf("\r\n");
//	UartPrintf("BEL CurLatitude %0.11f CurLongitude %0.11f CurAtitude %0.11f Status %d\r\n",GPSInf->latitude,GPSInf->longitude,GPSInf->altitude,GPSInf->PositionStatus);
	BLESend((unsigned char*)Inf,sizeof(Inf));	
}
void BELGetDevInf()
{
	char Inf[sizeof(BLEHead)+sizeof(BLETail)+sizeof(SysStatusBack_s)];
	BLEHead * frameHead=(BLEHead *)Inf;
	BLETail * frameTail=(BLETail *) (Inf+sizeof(BLEHead)+sizeof(SysStatusBack_s));
	SysStatusBack_s * SysInf=(SysStatusBack_s *)(Inf+sizeof(BLEHead));
	frameHead->head=FRAMEHEAD;
	frameHead->size=sizeof(BLEHead)+sizeof(BLETail)+sizeof(SysStatusBack_s);
	frameHead->type=GETDEVINF;
	SysInf->GPSMode=SysStatus.GPSMode;
	SysInf->RTKSource=SysStatus.RTKSource;
	SysInf->fixPos=SysStatus.FixPos;
	SysInf->latitude=SysStatus.Latitude;
	SysInf->longitude=SysStatus.Longitude;
	SysInf->altitude=SysStatus.Altitude;
	SysInf->ave=SysStatus.AveTime;
	SysInf->ServerPort=SysStatus.ServerPort;
	SysInf->RadioSpeed=SysStatus.RadioSpeed;
	SysInf->RadioChan=SysStatus.RadioChan;
	memcpy(SysInf->ServerIP,SysStatus.ServerIP,20);
	
	
	frameTail->CrcVerify=CRC_Cal((unsigned char *)&(frameHead->size),sizeof(BLEHead)-sizeof(uint16_t)+sizeof(SysStatusBack_s));
	frameTail->Tail=FRAMETAIL;

	BLESend(Inf,sizeof(Inf));	
}
void replyInf(BLECmd_e cmd,CmdReply_e reply)
{
	char Inf[sizeof(BLEHead)+sizeof(BLETail)+sizeof(CmdReply_e)];
	BLEHead * frameHead=(BLEHead *)Inf;
	BLETail * frameTail=(BLETail *) (Inf+sizeof(BLEHead)+sizeof(CmdReply_e));
	CmdReply_e * replyInf=(CmdReply_e *)(Inf+sizeof(BLEHead));
	frameHead->head=FRAMEHEAD;
	frameHead->size=sizeof(BLEHead)+sizeof(BLETail)+sizeof(CmdReply_e);
	frameHead->type=cmd;
	* replyInf=reply;
	frameTail->CrcVerify=CRC_Cal((unsigned char *)&(frameHead->size),sizeof(BLEHead)-sizeof(uint16_t)+sizeof(CmdReply_e));
	frameTail->Tail=FRAMETAIL;
	BLESend(Inf,sizeof(Inf));		
}
