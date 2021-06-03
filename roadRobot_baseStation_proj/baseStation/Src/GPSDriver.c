#include "GPSDriver.h"
#include "cmsis_os.h"
#include "main.h"
#include "UartDriver.h"
#include "Radio.h"
#include "BLEDriver.h"
#include "UartDriver.h"
#include "TaskList.h"
#include "BLEInterface.h"
#include "M4GDrive.h"
bool Test4GFlag =false;
typedef struct 
{
	unsigned char Preamble;
	uint16_t Length;
}RTCMHEAD;

int table[]={
	0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
	0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272,
	0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E, 0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646,
	0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7, 0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3,
	0x15E205, 0x93AEFE, 0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
	0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077, 0x681E59, 0xEE52A2,
	0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E, 0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24,
	0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5, 0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1,
	0xA11107, 0x275DFC, 0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
	0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375, 0x15723B, 0x933EC0,
	0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C, 0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4,
	0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15, 0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8,
	0xC90F5E, 0x4F43A5, 0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
	0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3, 0x92C69D, 0x148A66,
	0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A, 0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB,
	0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A, 0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E,
	0x4EBBF8, 0xC8F703, 0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
	0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863, 0xFAD8C4, 0x7C943F,
	0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B,
	0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA, 0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C,
	0x33D79A, 0xB59B61, 0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
	0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8, 0x4E2BC6, 0xC8673D,
	0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1, 0x26359F, 0xA07964, 0xACE092, 0x2AAC69,
	0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88, 0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC,
	0x9E874A, 0x18CBB1, 0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
	0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538};

#define BUFSIZE (1500)
#define CMDSIZE (40)
#define GPSREPLYSIZE (1000)
char RTKBuf[BUFSIZE];
char RTKChkBuf[BUFSIZE];
char RTKSendBuf[BUFSIZE*3];
int RTKSendBufLen;
int RTKSendBufCouter;
static char BaseCmd[][CMDSIZE]={"freset\r\n",
"SERIALCONFIG COM2 115200 N 8 1 N OFF\r\n",
"INTERFACEMODE COM2 NONE RTCMV3 OFF\r\n",
"LOG COM2 RTCM1005 ONTIME 10\r\n",
"LOG COM2 RTCM1033 ONTIME 10\r\n",
"LOG COM2 RTCM1075 ONTIME 1\r\n",
"LOG COM2 RTCM1085 ONTIME 1\r\n",
"LOG COM2 RTCM1125 ONTIME 1\r\n",
"POSAVE ON 0.01\r\n",
};
static char MobileCmd[][CMDSIZE]={"freset\r\n",
"SERIALCONFIG COM2 115200 N 8 1 N OFF\r\n",	
"INTERFACEMODE COM2 RTCMV3 NOVATEL OFF\r\n",
"LOG COM2 GPGGA ONTIME 0.2\r\n",
"LOG COM2 GPRMC ONTIME 0.2\r\n",
"LOG COM2 GPHDT ONCHANGED\r\n",
"log COM2 gphdtdualantenna ontime 0.2\r\n",
};
static char ResetCmd[][CMDSIZE]={"freset\r\n"};

static char StatusCmd[][CMDSIZE]={"log bestpos\r\n"
};
static char SaveConfigCmd[][CMDSIZE]={"SAVECONFIG\r\n"};

char GPSReplyStr[GPSREPLYSIZE];
int  GPSReplyLen;
uint16_t RTKBufLen;
uint16_t RTKChkBufLen;
	
uint8_t  CorrectCheck;
uint8_t  IncorrectCheck;
uint8_t PosStatus=0;
OnRecvFun OnGPSRecvEx=NULL;
OnRecvFun OnRTKRecvEx=NULL;
OnRecvFun OnGPSSetRecvEx=NULL;
double CurLatitude,CurLongitude ,CurAltitude,CurTime;
int CurRTK;
static SemaphoreHandle_t  RTKGetSemaphore = NULL;
static SemaphoreHandle_t  CmdWaitSemaphore = NULL;
//static SemaphoreHandle_t  GPSPort1Semaphore = NULL;
static TaskHandle_t HandleRTKCheckTaskIF = NULL;
static void RTKCheckTask(void * argument);
static TaskHandle_t HandleCorrectTaskIF = NULL;
static void CorrectTask(void * argument);
static TaskHandle_t HandleSetGPSParaTaskIF = NULL;
static void SetGPSParaTask(void * argument);
static TaskHandle_t HandleGetGPSStatusTaskIF = NULL;
static void GetGPSStatusTask(void * argument);
static TaskHandle_t HandleGPSMainTaskIF = NULL;
static void GPSMainTask(void * argument);
static void ParseGPSTask(void * Para);
double ConvertBL(double data);
unsigned int CalculateBlockCRC24(
unsigned int ulCount, /* Number of bytes in the data block */
unsigned char *ucBuffer ); /* Data block */
static const TickType_t xMaxBlockTime = pdMS_TO_TICKS(10000);
uint8_t RTKReady;
//TEST
int8_t RTKCheck_ResFlag=0; //1 success; 2 false; 0 默认
uint32_t RTKData_counter =0;
uint32_t RTKCorrect_counter =0;
uint32_t RTKInCorrect_counter =0;
uint32_t RTKChecTime =0;
uint32_t RTKTest_Fullcounter =0;




void IniGPS(void)
{
  RTKGetSemaphore= xSemaphoreCreateBinary();
	CmdWaitSemaphore=xSemaphoreCreateBinary();
	//GPSPort1Semaphore=xSemaphoreCreateMutex();

	ComPorts[Uart_GPS1].OnUartRecv=OnGPS1Recv;
	if(SysStatus.GPSMode==BaseS)
	{
		ComPorts[Uart_GPS2].OnUartRecv=OnRTKRecv;
	}
	if(SysStatus.GPSMode==MobileS)
	{
		ComPorts[Uart_GPS2].OnUartRecv=OnGPSRecv;
	}
	
//	OnGPSRecvEx=NULL;
//	//OnRTKRecvEx=NULL;
//	OnGPSSetRecvEx=NULL;
	RTKBufLen=0;
	CorrectCheck=0;
	IncorrectCheck=0;
	GPSReplyLen=0;
	RTKReady=0;
	RTKSendBufLen = 0;
	RTKSendBufCouter = 0;
	CurLatitude=CurLongitude=CurAltitude=0;
	//HAL_GPIO_WritePin(GPSRES_GPIO_Port,GPSRES_Pin,GPIO_PIN_RESET);
	//vTaskDelay(500);
	UartSetParameter(&(ComPorts[Uart_GPS1]),9600);
	HAL_GPIO_WritePin(GPSRES_GPIO_Port,GPSRES_Pin,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(POWGPS_GPIO_Port,POWGPS_Pin,GPIO_PIN_RESET);
	//xSemaphoreGive(RTKGetSemaphore);
	xTaskCreate( RTKCheckTask,   	/* 任务函数  */
							 "RTKCheckTask",     	/* 任务名    */
							 128,               	/* 任务栈大小，单位word，也就是4字节 */
							 NULL,              	/* 任务参数  */
							 1,                 	/* 任务优先级*/
							 &HandleRTKCheckTaskIF );  /* 任务句柄  */ 
	xTaskCreate( GPSMainTask,   	/* 任务函数  */
							 "GPSMainTask",     	/* 任务名    */
							 64,               	/* 任务栈大小，单位word，也就是4字节 */
							 NULL,              	/* 任务参数  */
							 1,                 	/* 任务优先级*/
							 &HandleGPSMainTaskIF );  /* 任务句柄  */ 	
							 	
}
void RestartGPSTask(void * p)
{
		userReplyInf("Restart GPS\r\n");
	  PosStatus=0;
		CorrectCheck=0;
		IncorrectCheck=0;
		RTKChkBufLen=0;
	  RTKReady=0;
		HAL_GPIO_WritePin(GPSRES_GPIO_Port,GPSRES_Pin,GPIO_PIN_RESET);
		vTaskDelay(200);
		HAL_GPIO_WritePin(GPSRES_GPIO_Port,GPSRES_Pin,GPIO_PIN_SET);
}

void SetGPSBaseModeTask(void * p)
{
		GPSReplyLen=0;
	  xSemaphoreTake(CmdWaitSemaphore, 0);//clean semaphore;
		UartSendData(&(ComPorts[Uart_GPS1]),BaseCmd[0],strlen(BaseCmd[0]));
		xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
		vTaskDelay(200);
		xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
		for(int n=1;n<(sizeof(BaseCmd)/CMDSIZE);n++)
		{
			UartSendData(&(ComPorts[Uart_GPS1]),BaseCmd[n],strlen(BaseCmd[n]));
			xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
			vTaskDelay(200);
		}
		UartSendData(&(ComPorts[Uart_GPS1]),SaveConfigCmd[0],strlen(SaveConfigCmd[0]));
		xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
		vTaskDelay(200);
		SysStatus.GPSMode=BaseS;
		SysRamStatus.SwitchMode=BaseS;
		SysStatus.AveTime=0.01;
		if(WriteStatus()==HAL_OK)
		{
			replyInf(SETGPSMODE,CMDOK);
			userReplyInf("Set GPS to Base\r\n");
		}
		else
		{
			replyInf(SETGPSMODE,WRITEERROR);
			userReplyInf("Save Data Error\r\n");
		}

		replyInf(SETGPSMODE,CMDOK);
}

void SetGPSMobileModeTask(void * p)
{
		
	  GPSReplyLen=0;
		SysStatus.GPSMode=MobileS;
		SysRamStatus.SwitchMode=MobileS;
		SysStatus.FixPos=0;
		SysStatus.AveTime=0.1;
	xSemaphoreTake(CmdWaitSemaphore, 0);//clean semaphore;
		UartSendData(&(ComPorts[Uart_GPS1]),MobileCmd[0],strlen(MobileCmd[0]));
		xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
		vTaskDelay(200);
		xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
		for(int n=1;n<(sizeof(MobileCmd)/CMDSIZE);n++)
		{
			UartSendData(&(ComPorts[Uart_GPS1]),MobileCmd[n],strlen(MobileCmd[n]));
			xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
			vTaskDelay(200);
		}	
		UartSendData(&(ComPorts[Uart_GPS1]),SaveConfigCmd[0],strlen(SaveConfigCmd[0]));
		xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
		vTaskDelay(200);
		
		if(WriteStatus()==HAL_OK)
		{
			replyInf(SETGPSMODE,CMDOK);
		  userReplyInf("Set GPS to Mobile\r\n");
		}
		else
		{
			replyInf(SETGPSMODE,WRITEERROR);
			userReplyInf("Save Data Error\r\n");
		}

		replyInf(SETGPSMODE,CMDOK);
		
}
void SetRTKSourceTask(void* Para)
{
	if(Para==NULL)
		return;
	RTKSource_m * Source=(RTKSource_m *)Para;
	SysStatus.RTKSource =*Source;
	WriteStatus();
	replyInf(SETRTKSOURCE,CMDOK);
	userReplyInf("Set RTKSource to %d\r\n",*Source);		
}

void SetFixPosTask(void* Para)
{
	char cmd[60];
	double * BLH;
	if(Para==NULL)
		return;
	BLH=(double *)Para;
	for(int n=0;n<2;n++)
	{
		if((BLH[n]>180)||(BLH[n]<-180))
		{
			replyInf(SETBASEPOS,PARAERROR);
			IllegalInstruction();
			return;
		}
	}
	if((BLH[2]>10000)||(BLH[2]<0))
	{
		replyInf(SETBASEPOS,PARAERROR);
		IllegalInstruction();
		return;
	}
	xSemaphoreTake(CmdWaitSemaphore, 0);//clean semaphore;
	sprintf(cmd ,"FIX POSITION %0.11f %0.11f %0.5f\r\n",BLH[0],BLH[1],BLH[2]);
	UartSendData(&(ComPorts[Uart_GPS1]),cmd,strlen(cmd));
	xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
	vTaskDelay(200);
	UartSendData(&(ComPorts[Uart_GPS1]),SaveConfigCmd[0],strlen(SaveConfigCmd[0]));
	xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime);
	vTaskDelay(200);
	
	SysStatus.FixPos =1;
	SysStatus.Latitude=BLH[0];
	SysStatus.Longitude=BLH[1];
	SysStatus.Altitude=BLH[2];
	if(WriteStatus()==HAL_OK)
	{
		replyInf(SETBASEPOS,CMDOK);
		userReplyInf("Set Fix to %0.11f %0.11f %0.5f\r\n",BLH[0],BLH[1],BLH[2]);		
	}
	else
	{
		replyInf(SETBASEPOS,WRITEERROR);
		userReplyInf("Save Data Error\r\n");
	}

}
void SetAvePosTask(void* Para)
{
	char cmd[20];
	double * AveTime;
	if(Para==NULL)
		return;
	AveTime=(double *)Para;
	if((*AveTime>99)||(*AveTime<0.001))
	{
		replyInf(SETAVETIME,PARAERROR);
		IllegalInstruction();
		return;
	}
	xSemaphoreTake(CmdWaitSemaphore, 0);//clean semaphore;
	sprintf(cmd ,"POSAVE ON %0.2f\r\n",*AveTime);
	UartSendData(&(ComPorts[Uart_GPS1]),cmd,strlen(cmd));
	if(pdTRUE==xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime))
	{
		userReplyInf("Set SetAvePos Done\r\n");
	}
	else
	{
		userReplyInf("Set SetAvePos Time Out\r\n");
		replyInf(SETAVETIME,OUTTIME);
		return;
	}
	vTaskDelay(200);
	UartSendData(&(ComPorts[Uart_GPS1]),SaveConfigCmd[0],strlen(SaveConfigCmd[0]));
	if(pdTRUE==xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime))
	{
		userReplyInf("Set SaveConfigCmd Done\r\n");
	}
	else
	{
		userReplyInf("Set SaveConfigCmd Time Out\r\n");
		replyInf(SETAVETIME,OUTTIME);
		return;
	}
	vTaskDelay(200);
	
		
	SysStatus.FixPos =0;
	SysStatus.AveTime=*AveTime;
	if(WriteStatus()==HAL_OK)
	{
			replyInf(SETAVETIME,CMDOK);
		userReplyInf("Set AveTime to  %0.2f\r\n",*AveTime);	
	}
	else
	{
		replyInf(SETAVETIME,WRITEERROR);
		userReplyInf("Save Data Error\r\n");
	}	
}
void ParseGPSTask(void * Para)
{
	int Flag1,Flag2,Flag3;
	char * data =(char*)Para;
	int len=strlen(data);
	Flag1=0;
	if(len>0)
	{
		//UartPrintf("DATA %s\r\n",data);
		while(1)
		{
			for(;Flag1<len;Flag1++)
			{
				if(data[Flag1]=='$')
					break;
			}
			if(Flag1==len)
				return;
			for(Flag2=Flag1+1;Flag2<len;Flag2++)
			{
				if(data[Flag2]==',')
					break;
			}		
			if(Flag2==len)
				return;
			for(Flag3=Flag2+1;Flag3<len;Flag3++)
			{
				if(data[Flag3]=='*')
					break;
			}		
			if(Flag3==len)
				return;
			data[Flag2]='\0';
			data[Flag3]='\0';
			if(strcmp(data+Flag1+1,"GPGGA")==0)
			{
				double h2;
				char * temp=data+Flag2+1;
				CurRTK=0;
				sscanf(temp,"%*f,%*f,N,%*f,E,%d,%*d,%*f,%lf,M,%lf,",&CurRTK,&CurAltitude,&h2);
				//UartPrintf("RTK %d Altitude %0.5f\r\n",CurRTK,CurAltitude);
				CurAltitude=CurAltitude+h2;
				if(CurRTK==0)
				{
					//UartPrintf("%s\r\n",temp);
				}
			}
			
			else if(strcmp(data+Flag1+1,"GPRMC")==0)
			{
				sscanf(data+Flag2+1,"%lf,A,%lf,N,%lf,",&CurTime,&CurLatitude,&CurLongitude);
				CurLatitude=ConvertBL(CurLatitude);
				CurLongitude=ConvertBL(CurLongitude);
				BELGetGPS();
				//UartPrintf("RAW %s\r\n",data+Flag2+1);
				//UartPrintf("CurTime %f CurLatitude %0.11f CurLongitude %0.11f\r\n",CurTime,CurLatitude,CurLongitude);
			}
			Flag1=Flag3+1;
		}
		
	}
}
double ConvertBL(double data)
{
	double temp,ans;
	int    tempi;
	tempi=(int)(data/100);
	temp=data-tempi*100;
	return(tempi+temp/60.0);
}
int RestartGPS()
{
	AddTaskFromISR(ExePerformer,RestartGPSTask,NULL);
	return 0;
}
int SetBaseMode()
{
	ComPorts[Uart_GPS2].OnUartRecv=OnRTKRecv;	
	AddTaskFromISR(ExePerformer,SetGPSBaseModeTask,NULL);
	return 0;
}

int SetMobileMode()
{
	ComPorts[Uart_GPS2].OnUartRecv=OnGPSRecv;	
	AddTaskFromISR(ExePerformer,SetGPSMobileModeTask,NULL);
	return 0;
}
int SetRTKSource(RTKSource_m Source)
{
	static RTKSource_m Source_t;
	if(SysStatus.GPSMode!=MobileS)
	{
		replyInf(SETRTKSOURCE,CMDILLEGAL);
		return 1;
	}
	Source_t=Source;
	AddTaskFromISR(ExePerformer,SetRTKSourceTask,&Source_t);
	return 0;	
}

int SetFixPos(double BLH[3])
{
	static double BLH_t[3];
	if(SysStatus.GPSMode!=BaseS)
	{
		replyInf(SETBASEPOS,CMDILLEGAL);
		return 1;
	}
	BLH_t[0]=BLH[0];
	BLH_t[1]=BLH[1];
	BLH_t[2]=BLH[2];
	AddTaskFromISR(ExePerformer,SetFixPosTask,BLH_t);
	return 0;		
}

int SetAveTime(double Time)
{
	static double AveTime;
	if(SysStatus.GPSMode!=BaseS)
	{
		replyInf(SETAVETIME,CMDILLEGAL);
		return 1;
	}
	AveTime=Time;
	AddTaskFromISR(ExePerformer,SetAvePosTask,&AveTime);
	return 0;		
}


void OnRTKRecv(unsigned char * buf ,int len)
{

	//UartPrintf("OnRTK\r\n");
	GetRTK(buf,len); //更新RTK数据
}
int  GPS1Send(unsigned char * buf ,int len)
{
		UartSendData(&(ComPorts[Uart_GPS1]),buf,len);

}
int  GPS2Send(unsigned char * buf ,int len)
{
		UartSendData(&(ComPorts[Uart_GPS2]),buf,len);

}

int  GetRTK(unsigned char * buf ,int len)
{
	//GPS2Send(buf,len);
	
	if(BUFSIZE<(len))
	{
		return 0;
	}
	memcpy(RTKBuf,buf,len);
	
#ifdef BASE_4G_TEST_SEND
	
	static uint8_t recCount=0;
	++recCount;
	if(recCount>5){
		setRTKData(buf,len);
		recCount=0;
	}		
#endif
	RTKBufLen=len;
	static portBASE_TYPE xTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(RTKGetSemaphore,&xTaskWoken );
	return len;
}
void OnGPSRecv(unsigned char * buf ,int len)
{
	static char gpsdata[1024];

	if(SysRamStatus.SwitchMode==MobileS)
	{
		//printf("----len: %d\r\n",len);
		memcpy(gpsdata,buf,len);
		gpsdata[len]='\0';
		AddTaskFromISR(ExePerformer,ParseGPSTask,gpsdata);
	}
	
	if(OnGPSRecvEx!=NULL)
		OnGPSRecvEx(buf,len);
}

void OnGPS1Recv(unsigned char * buf ,int len)
{
	if(OnGPSSetRecvEx!=NULL)
		OnGPSSetRecvEx(buf,len);
	if((len+GPSReplyLen)<GPSREPLYSIZE)
	{
		static portBASE_TYPE xTaskWoken = pdFALSE;
		memcpy(GPSReplyStr + GPSReplyLen,buf,len);
		GPSReplyLen+=len;	
		xSemaphoreGiveFromISR(CmdWaitSemaphore,&xTaskWoken );				
	}
	else
		GPSReplyLen=0;
	//UartPrintf("OnGPS1Recv\r\n");

}

static void RTKCheckTask(void * argument)
{
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000000);
	while(1)
	{
		 uint16_t index;
		//printf("RTKGetSemaphore:%p\r\n",RTKGetSemaphore);
		 //xSemaphoreTake(RTKGetSemaphore, (TickType_t)xMaxBlockTime);//
		if(RTKGetSemaphore==NULL){
			continue;
		}
		xSemaphoreTake(RTKGetSemaphore, portMAX_DELAY);
		//osDelay(500);
			static int testxx =0;
		//printf("RTKCheck:%d\r\n",testxx++);
		
		 if(RTKBufLen>0)
		 {
			 //UartPrintf("Recv Lend %d\r\n",RTKBufLen);
			 if(BUFSIZE<(RTKBufLen+RTKChkBufLen))
			 {
				 UartPrintf("Buf Full time:%d\r\n",HAL_GetTick);
				 ++RTKTest_Fullcounter;
				 RTKChkBufLen=0;
			 }
			 memcpy(RTKChkBuf+RTKChkBufLen,RTKBuf,RTKBufLen);
			 RTKChkBufLen+=RTKBufLen;
			 
			 
			 RTKBufLen=0;
			 index=0;
			 //UartPrintf("Time: %d  INT RTK Len %d  index %d\r\n",xTaskGetTickCount(),RTKChkBufLen,index);
			 for(index=0;index<RTKChkBufLen;index++)
			 {
				 ///
				 if(RTKChkBuf[index]==0xD3)
				 {
							if((index+3)>RTKChkBufLen)
								break;
							uint16_t Len=0;
							uint8_t * LenBuf=(uint8_t *)&Len;
							LenBuf[0]=RTKChkBuf[index+2];
							LenBuf[1]=RTKChkBuf[index+1];		
							if(Len<1024)
							{
								uint16_t framelen=Len+6;
								if((framelen+index)<=RTKChkBufLen)
								{
									++RTKData_counter;
									unsigned long CRCAns=CalculateBlockCRC24(framelen,RTKChkBuf+index);
									if(CRCAns==0)
									{
										if(SysStatus.GPSMode==BaseS)
										{
											//UartPrintf("Get RTK Size %d \r\n",framelen);	
											/*if(framelen>100)
											{
												memcpy(RTKSendBuf+RTKSendBufLen,RTKChkBuf+index,framelen);
												RTKSendBufCouter++;
												RTKSendBufLen+=framelen;
												UartPrintf("Check RTK Size %d \r\n",framelen);
												if(RTKSendBufCouter>=3)
												{
													if(OnRTKRecvEx!=NULL)
													{
															OnRTKRecvEx(RTKSendBuf,RTKSendBufLen);
														UartPrintf("Get RTK Size %d \r\n",RTKSendBufLen);
													}														
													RTKSendBufLen = 0;
													RTKSendBufCouter = 0;
												}													
											}
											else
											{
												if(OnRTKRecvEx!=NULL)
														OnRTKRecvEx(RTKChkBuf+index,framelen);												
											}*/
											if(OnRTKRecvEx!=NULL)
														OnRTKRecvEx(RTKChkBuf+index,framelen);	
											
										}
										else if(SysStatus.GPSMode==MobileS)
										{
											//UartPrintf("Get RTK Size %d \r\n",framelen);	
												GPS2Send(RTKChkBuf+index,framelen);  
										}
										
										//UartPrintf("CRC Done!index %d framelen %d\r\n",index,framelen);
										index+=(framelen-1);
										CorrectCheck++;
										++RTKCorrect_counter;
										RTKCheck_ResFlag =1;
										RTKChecTime = HAL_GetTick();
										UartPrintf("\r\n%4.3f, %d, %d, %d, %d,%d\r\n",BASE_4G_TEST_HEAD,RTKInCorrect_counter,RTKCorrect_counter,RTKData_counter
													,RTKChecTime,RTKTest_Fullcounter);
										//UartPrintf("\r\n[RTKCheck] correct!\r\n");
									}
									else
									{
										IncorrectCheck++;
										int head=framelen+index;
										//UartPrintf("\r\n[RTKCheck] Incorrect!\r\n");
										++RTKInCorrect_counter;
										RTKCheck_ResFlag =2;
										//RTKChecTime = HAL_GetTick();
//										for(int i=0;i<framelen+5;i++)
//										{
//											UartPrintf("%02X ",(uint8_t)RTKChkBuf[index+i]);
//											if((i+1)%10==0)
//												UartPrintf("\r\n");
//										}
//										UartPrintf("\r\n");
										//UartPrintf("CRC Incorrect!index %d  framelen %d remain %d\r\n",index,framelen,RTKChkBufLen-framelen-index);
										
									}
								}
								else
								{
										///UartPrintf("No Tail len %d index %d\r\n",framelen,index);
										break;
								}
							}
				 }
			 }
			 //UartPrintf("OUT RTK Len %d  index %d\r\n",RTKChkBufLen,index);
			 if(index==0)
			 {
				 
			 }
			 else if(index<RTKChkBufLen)
			 {
				 RTKChkBufLen=RTKChkBufLen-index;
				 memmove(RTKChkBuf,RTKChkBuf+index,RTKChkBufLen);
			 }		 
			 else
			 {
				 RTKChkBufLen=0;
			 }
		 }
	}
}
static void CorrectTask(void * argument)//检查是否有RTK输入
{
		int Counter=IncorrectCheck+CorrectCheck;
		if(Counter!=0)
		{
			float CorrectP=((float)CorrectCheck)/(IncorrectCheck+CorrectCheck)*100;
			if(CorrectCheck>5)
				RTKReady=1;
			else
			{
				PosStatus=NOPOSITION;
				RTKReady=0;
			}
			UartPrintf("Correct %.2f%% \tSum %d  \tCorrect %d \tIncorrect %d\r\n",CorrectP,Counter,CorrectCheck,IncorrectCheck);
			CorrectCheck=0;
			IncorrectCheck=0;
		}
		else
		{
			UartPrintf("No RTK Data\r\n");			
		}
}
unsigned int CalculateBlockCRC24(
unsigned int ulCount, /* Number of bytes in the data block */
unsigned char *ucBuffer ) /* Data block */
{
	unsigned int crc = 0,i;
	int buf[3];
	for(i = 0; i< ulCount; i++)
	{
		crc = ((crc << 8) & 0xFFFFFF) ^ table[(crc >> 16) ^ (ucBuffer[i] & 0xff)];
	}
	return crc;
	//printf("%x,%x,%x\r\n",buf[0],buf[1],buf[2]);
}

void GPSPPSPlus()
{
		TickType_t GPSPPSTime;
		GPSPPSTime = xTaskGetTickCount();
		//UartPrintf("GPS PPS: %d\r\n",GPSPPSTime);
}
void GetGPSStatusTask(void * argument)
{
		int TimeCounter=0;
		char PosStatusStr[15];

		const TickType_t xMaxBlockTime = pdMS_TO_TICKS(2000);
		 int statuslen,statusindex;
		 GPSReplyLen=0;
		 xSemaphoreTake(CmdWaitSemaphore, 0);//clean semaphore;
		 UartSendData(&(ComPorts[Uart_GPS1]),StatusCmd[0],strlen(StatusCmd[0]));
	   //UartPrintf("GPS:1\r\n");
		 if(pdTRUE==xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime))
		 {
			 double Latitude,Longitude ,Altitude;
			 //vTaskDelay(100);			 
			 //GPSReplyLen=0;
			 //UartPrintf("GPS:2\r\n");
			 if(pdTRUE==xSemaphoreTake(CmdWaitSemaphore, (TickType_t)xMaxBlockTime))
			 {
				 //UartPrintf("GPS:3\r\n");
				 if(GPSReplyLen>20)
				 {
					 //vTaskDelay(200);		
					 GPSReplyStr[GPSReplyLen]='\0';
					 
					 statuslen=GPSReplyLen;
					 for(statusindex=50;statusindex<statuslen;statusindex++)
					 {
						 if(GPSReplyStr[statusindex]=='<')
							 break;
					 }
					 sscanf(GPSReplyStr+statusindex,"%*s %*s %s %lf %lf %lf",PosStatusStr,&Latitude,&Longitude ,&Altitude );
					 //UartPrintf("GPS_M:  %s %0.11f %0.11f %0.5f %d\r\n",PosStatusStr,Latitude,Longitude ,Altitude,CurRTK);
					 CurAltitude=Altitude;
					 CurLatitude=Latitude;
					 CurLongitude=Longitude;
					 
					 if(strcmp(PosStatusStr,"NONE")==0)
					 {
						 PosStatus=NOPOSITION;
					 }
					 else if(((strcmp(PosStatusStr,"FIXEDPOS")==0)&&(RTKReady==1))||((strcmp(PosStatusStr,"NARROW_INT")==0)&&(CurRTK!=0)))
					 {
						 PosStatus=RTKPOSITION;
					 }
					 else if((SysStatus.GPSMode!=BaseS)&&(CurRTK==0))
					 {
						 PosStatus=NOPOSITION;
					 }
					 else
					 {
						 PosStatus=POSITION;
					 }
				 }
				 else
				 {
					 PosStatus=NOPOSITION;
				 }
			}
//			else
//			{
//				 UartPrintf("GPS_M: flaut1 %d \r\n",xTaskGetTickCount());
//				 PosStatus=0;
//			}
		 }
		 else
		 {
				 UartPrintf("GPS_M: flaut2 %d \r\n",xTaskGetTickCount());
				 PosStatus=NOPOSITION;
		 }	 
}

static void GPSMainTask(void * argument)
{
	int TimeCounter=0;
	 while(1)
	 {
		 
		 vTaskDelay(500);

		 if((TimeCounter%10)==0)
		 {
			 if(SysRamStatus.SwitchMode!=LinkS)
					AddTask(ExePerformer,GetGPSStatusTask,NULL);
		 }	
		 if((TimeCounter%20)==0)
		 {
			 //if(SysStatus.GPSMode==MobileS)
					AddTask(ExePerformer,CorrectTask,NULL);
		 }	
		 
		 switch (PosStatus)
		 {
			 case NOPOSITION:
						HAL_GPIO_WritePin(GPSS_GPIO_Port,GPSS_Pin,GPIO_PIN_RESET);
				 break;
			 case POSITION:
						HAL_GPIO_TogglePin(GPSS_GPIO_Port,GPSS_Pin);
				 break;
			 case RTKPOSITION:
						HAL_GPIO_WritePin(GPSS_GPIO_Port,GPSS_Pin,GPIO_PIN_SET);
				 break;
		 }
		 TimeCounter++;
		 	 
	 }	
}

PositionStatus_e GetGPS(double *B,double *L,double *H)
{
	*B=0;
	*L=0;
	*H=0;
	if((B==NULL)||(L==NULL)||(H==NULL))
		return NOPOSITION;
	if(PosStatus==NOPOSITION)
		return NOPOSITION;
	*B=CurLatitude;
	*L=CurLongitude ;
	*H=CurAltitude;
	return PosStatus;
}