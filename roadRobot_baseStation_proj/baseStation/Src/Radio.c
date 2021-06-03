#include "Radio.h"
#include "cmsis_os.h"
#include "main.h"
#include "GPSDriver.h"
#include "TaskList.h"
#include "StatusManager.h"
#include "BLEInterface.h"
//#define	 PackSize (58)
#include "CRC.h"
#define	 PackSize (150)
#define	 BufSize (10)
#define  ImportantPackSize (100)
#define  ImportantBufSize (3)
#define TESTRADIO_BUFFSIZE (1024)
#define Uart_WDR_RATE (115200)
uint8_t SetCmdSpeed(uint8_t speed);
void SetComSpeed(uint8_t speed);
static TaskHandle_t HandleRadioTestTaskIF = NULL;
static void RadioTestTask(void * argument);

static TaskHandle_t HandleRadioSendTaskIF = NULL;
static void RadioSendTask(void * argument);
void SetWaitTime(uint8_t radioSpeed);
void RadioReadConfig();
//char RadioBuf[1024];
int  BufLen=0;
char radiotest=0;
char Cmd[10]={0xc0,0x0, 0x2, 0x3d, 0x17, 0x04};
char ConfigBuf[6];
//char Cmd[10]={0xc0,0x0, 0x2, 0x3a, 0x17, 0x40};
uint16_t WaitTime;

uint8_t RadioSeting=0;
typedef struct 
{
	char buf[PackSize];
	unsigned short len;
}RadioBuf_s;
//int8_t airmode;
typedef struct 
{
	char buf[ImportantPackSize];
  uint16_t len;
}ImportantBuf_s;
RadioBuf_s RadioBufs[BufSize];
ImportantBuf_s ImportantBufs[ImportantBufSize];
unsigned IndexHead;
unsigned IndexEnd;
uint8_t BufFull;

unsigned ImportantIndexHead;
unsigned ImportantIndexEnd;
uint8_t ImportantBufFull;


static SemaphoreHandle_t  SendSemaphore = NULL;
static SemaphoreHandle_t   SetSemaphore =NULL;
OnRecvFun OnRadioRecvEx=NULL;
uint8_t RadioInitialized=0;
void checkImportantBuf();
void IniRadio(void)
{
	RadioInitialized=0;
	radiotest=0;
	BufFull=0;
	RadioSeting=0;
	//WaitTime=110;
	IndexHead = IndexEnd=0;
	
	ImportantIndexHead=ImportantIndexEnd=0;
  ImportantBufFull=0;

	SendSemaphore = xSemaphoreCreateBinary();
	ComPorts[Uart_WDR].OnUartRecv=OnRadioRecv;
	//airmode=-1;
	//HAL_GPIO_WritePin(RADIOM0_GPIO_Port, RADIOM0_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(RADIOM0_GPIO_Port, RADIOM0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //复位引脚为高，才能正常工作	
//	HAL_GPIO_WritePin(POWRADIO_GPIO_Port, POWRADIO_Pin, GPIO_PIN_RESET);	

	//SetComSpeed(SysStatus.RadioSpeed);
	//OnRadioRecvEx=NULL;
	//RadioReadConfig();
	SetWaitTime(SysStatus.RadioSpeed);
	RadioInitialized=1;
	xTaskCreate( RadioSendTask,   	/* 任务函数  */
							 "RadioSendTask",     	/* 任务名    */
							 128,               	/* 任务栈大小，单位word，也就是4字节 */
							 NULL,              	/* 任务参数  */
							 1,                 	/* 任务优先级*/
							 &HandleRadioSendTaskIF );  /* 任务句柄  */
	
	//xTaskCreate( RadioTestTask,   
	//						 "RadioTestTask",     	
	//						 128,               	
	//						 NULL,              	
	//						 1,                 	
	//						 &HandleRadioTestTaskIF );  
	
}
void OnRadioRecv(unsigned char * buf ,int len)
{
	if(RadioInitialized==1)
	{
		if(RadioSeting==1)
		{
			if(memcmp(ConfigBuf,buf,6)==0)
				RadioSeting=0;
		}
		if(OnRadioRecvEx!=NULL)
			OnRadioRecvEx(buf,len);
	}
}

static void RadioSendTask(void * argument)
{
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(10000); /* 设置最大等待时间为10000ms */
	/*Cmd[0]=0xc0;
	Cmd[1]=0x00;
	Cmd[2]=0x02;
	Cmd[3]=0x3d;
	Cmd[4]=0x17;
	Cmd[5]=0x40;*/	
	//vTaskDelay(2000);

	while(1)
	{
		while((IndexHead!=IndexEnd)||(BufFull==1))
		{
			
			//UartPrintf("Send IndexHead %d IndexEnd %d Size %d \r\n",IndexHead,IndexEnd,RadioBufs[IndexHead].len);	
			UartSendData(&(ComPorts[Uart_WDR]),(unsigned char*)RadioBufs[IndexHead].buf,RadioBufs[IndexHead].len);
			
			#ifdef  BASE_TEST
			static unsigned IndexHead_test=0;
			IndexHead_test = IndexHead;
			UartSendData(&(ComPorts[Uart_232]),(unsigned char*)RadioBufs[IndexHead_test].buf,RadioBufs[IndexHead_test].len);
			#endif
			IndexHead++;
			if(IndexHead>=BufSize)
				IndexHead=0;
			BufFull=0;
			//checkImportantBuf();
			//vTaskDelay(20);
			//UartPrintf("WaitTime %d\r\n",WaitTime);
			//vTaskDelay(WaitTime);//如果使用非连串模式，需要使用该语句
			//vTaskDelay(WaitTime);
		}
		xSemaphoreTake(SendSemaphore, (TickType_t)xMaxBlockTime);		
	}
}
int getRemainBuf()
{
	int bufsize;
		if(IndexHead==IndexEnd)
	{
		if(BufFull==1)
			bufsize=0;
		else
			bufsize=BufSize*PackSize;	
	}else
	{
		bufsize=(BufSize+IndexHead-IndexEnd)%BufSize*PackSize;
	}
	return bufsize;
}
void checkImportantBuf()
{
	if((ImportantBufFull!=0)||(ImportantIndexHead!=ImportantIndexEnd))
	{
		int bufsize;
		bufsize=getRemainBuf();
		if(bufsize>ImportantBufs[ImportantIndexHead].len)
		{
			//RadioSend(ImportantBufs[ImportantIndexHead].buf,ImportantBufs[ImportantIndexHead].len);
			ImportantIndexHead=(ImportantIndexHead+1)%ImportantBufSize;
			ImportantBufFull=0;
		}
	}
}

int  RadioSend(unsigned char * buf ,int len)
{
	int bufsize;
	
	
	if(RadioInitialized==0)
		return 0;

	if(buf==NULL)
		return 0;
	if(len<0)
		return 0;
	
	//UartSendData(ComPorts[Uart_WDR],buf,len);
	//return 0;
	
	bufsize=getRemainBuf();
	if(bufsize<len)
	{
		UartPrintf("Radio Buf Full, remain %d need %d \r\n",bufsize,len);	
		return 0;
	}
	/*if(bufsize<len)
	{
		//UartPrintf("Send Radio Full bufsize %d len %d\r\n",bufsize,len);	
		
		if(ImportantPackSize>len)
		{
			if(ImportantBufFull==0)
			{
				memcpy(ImportantBufs[ImportantIndexEnd].buf,buf,len);
				ImportantIndexEnd=(ImportantIndexEnd+1)%ImportantBufSize;
				if(ImportantIndexHead==ImportantIndexEnd)
					ImportantBufFull=1;
			}
		}
		else
		{
			UartPrintf("Send Radio Full len %d\r\n",len);	
		}
		return 0;
	}*/
	//UartPrintf("Send bufsize %d len %d IndexHead %d IndexEnd %d\r\n",bufsize,len,IndexHead,IndexEnd);	
	for(int index=0;index<len;index+=PackSize)
	{
		int RemainSize=len-index;
		if((RemainSize)>PackSize)
		{
			memcpy(RadioBufs[IndexEnd].buf,buf+index,PackSize);
			RadioBufs[IndexEnd].len=PackSize;			
		}
		else
		{
			memcpy(RadioBufs[IndexEnd].buf,buf+index,RemainSize);
			RadioBufs[IndexEnd].len=RemainSize;
			//UartPrintf("RadioBufs index %d len %d \r\n",IndexEnd,RemainSize);
		}
		//UartPrintf("Send IndexEnd %d index %d lastsize %d len %d\r\n",IndexEnd,index,lastsize,len);
		IndexEnd++;
		if(IndexEnd>=BufSize)
				IndexEnd=0;		
	}
	if(IndexEnd==IndexHead)
		BufFull=1;
static portBASE_TYPE xTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(SendSemaphore,&xTaskWoken );
}
static void RadioTestTask(void * argument)
{
	static char SendBuf[]={0xD3,0x00,0xB2,0x43,0x30,0x00,0x2B,0x10,0xB8,0x22,0x00,0x20,0x24,0x05,\
		0x0C,0x84,0x00,0x00,0x00,0x00,0x20,0x20,0x00,0x00,0x7F,0xFF,0x28,0x26,0x23,0x21,0x27,0x26,0x27,\
		0x23,0x80,0x00,0x00,0x00,0x23,0xD2,0xDC,0xD1,0x24,0x1C,0xCE,0x6A,0xE9,0x4C,0xFB,0xB5,0xF3,0x7F,\
		0xCF,0x1F,0xEF,0x03,0x4E,0x0F,0x08,0x4F,0x3F,0x41,0xA0,0x24,0x3F,0x07,0xCA,0x77,0x93,0x16,0xF8,\
		0xFD,0xE6,0x20,0x91,0x40,0xFD,0x83,0xA0,0x06,0xE9,0xAC,0x83,0x55,0x27,0xD0,0xEF,0xB4,0x3F,0x9C,\
		0x41,0x73,0xA2,0x06,0x01,0x67,0xF4,0x8B,0x2F,0x65,0x84,0x41,0xEC,0x9C,0xEC,0xCF,0xF8,0x07,0x7C,\
		0xB0,0x8B,0xF7,0x43,0xB0,0xD3,0xF7,0x93,0x7B,0xFB,0xD7,0xAE,0xB7,0x1F,0x81,0x74,0x6B,0x05,0x79,\
		0x9B,0xCC,0x99,0x9F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD,0xD9,0x00,0x01,0x75,0x1A,0x57,0xA6,0x3C,0xED,\
		0x75,0x7A,0x5B,0x5D,0x72,0x5D,0xF2,0xBB,0xD9,0xEC,0x63,0xD8,0x9F,0xA5,0xEF,0x4B,0xB8,0x3D,0x70,\
		0x7A,0xA4,0x3E,0x48,0x7E,0x8E,0xE5,0x1D,0xE0,0x57,0xA0,0xB0,0x5A,0x65,0x00,0x24,0x56,0xEB,0xD3,\
		0x00,0x89,0x43,0xD0,0x00,0x44,0x6F,0xEC,0xE2,0x00,0x20,0x06,0x02,0x38,0x00,0x00,0x00,0x00,0x00,\
		0x20,0x40,0x00,0x00,0x77,0xFA,0x12,0x62,0x7A,0x82,0x1A,0x14,0x18,0x25,0x4B,0xC6,0x1B,0xC6,0xCD,\
		0x9A,0x90,0x93,0xFF,0x82,0x0E,0x40,0x2F,0x1E,0x64,0x7B,0xD6,0x0A,0x3E,0x21,0xAC,0x57,0x1E,0x9F,\
		0x8A,0xE8,0x17,0x58,0xDF,0xDF,0xC3,0x74,0x55,0x50,0xAD,0xB1,0xF2,0xE4,0x17,0x7A,0xFE,0xB2,0xFD,\
		0xF8,0xB3,0xEB,0x81,0x40,0xA6,0x71,0x03,0xB0,0x44,0x00,0xBB,0x2F,0xBC,0x28,0x90,0xBF,0x7B,0xFE,\
		0xCB,0x15,0xFF,0x43,0x88,0x63,0x4C,0xDF,0xFE,0xDC,0xFC,0xFF,0xFF,0x00,0x1A,0xE3,0x75,0x34,0xC4,\
		0xE5,0x76,0xE3,0x8F,0x97,0x1F,0x2E,0x82,0xCE,0x03,0x36,0x86,0x68,0xEC,0xFF,0xD9,0xAF,0x5C,0xC6,\
		0xB9,0x74,0x69,0x08,0xD2,0x40,0xF4,0xAB,0xB5,0xD3,0x01,0x22,0x46,0x50,0x00,0x2B,0x0F,0xDD,0x60,\
		0x00,0x20,0x7A,0xD6,0x84,0xA8,0x00,0x00,0x00,0x00,0x20,0x02,0x00,0x00,0x77,0xFF,0xFF,0x54,0xFC,\
		0xF6,0xF5,0x08,0xF0,0xF2,0xF4,0x94,0xEE,0x92,0xF0,0xA6,0xB0,0xA6,0xA2,0x00,0x00,0x00,0x00,0x00,\
		0x00,0x00,0x16,0xE0,0xAD,0xA6,0x94,0x27,0xBA,0xAE,0xD8,0xD2,0x68,0xC5,0xA9,0x79,0x67,0x2D,0x26,\
		0x6C,0x37,0x7F,0xF8,0x00,0x07,0xFF,0x7F,0xFE,0x7F,0xFF,0xFF,0x58,0x0C,0xBF,0x40,0x80,0x76,0x01,\
		0x40,0x02,0xBF,0x28,0x83,0xAC,0x0B,0x18,0x36,0x19,0x9D,0x32,0xA0,0x9A,0x96,0x07,0xA6,0x0C,0x7D,\
		0x01,0x81,0xF9,0xAD,0x17,0x1A,0x02,0x3B,0xDB,0x76,0xDB,0x33,0x0A,0x65,0x7B,0x87,0xAF,0x0D,0x01,\
		0xF8,0xC4,0x28,0x73,0x84,0x66,0x99,0x17,0xB2,0x31,0x64,0x4D,0x1F,0x7C,0xCD,0x53,0xE6,0x04,0x3F,\
		0x15,0x83,0xFC,0x50,0xD0,0x04,0x8B,0xCF,0x55,0xF2,0x7D,0x56,0x61,0x0D,0x8F,0xB8,0x35,0x38,0x3E,\
		0x74,0xD5,0x7E,0xB6,0xA4,0xFE,0x16,0x48,0x23,0xEE,0x41,0x52,0x5B,0x45,0x4A,0x5B,0xEA,0x54,0xF7,\
		0xEA,0x3F,0x30,0x00,0x9B,0x04,0x45,0xFF,0xEA,0xF0,0xC3,0xAB,0x12,0x60,0x30,0x2E,0x41,0xA7,0xD6,\
		0x0A,0x24,0xC0,0x18,0x03,0xBF,0xFD,0xFE,0x43,0x0D,0x01,0xFF,0x6F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,\
		0xFF,0xFF,0xFF,0xFF,0xF0,0x00,0x00,0x05,0x97,0xCB,0x86,0x34,0xD9,0x9E,0x78,0xE5,0x45,0x59,0xEB,\
		0xA6,0x7A,0xEB,0xA6,0x56,0xDD,0x7E,0x37,0xE3,0x6F,0x4B,0x78,0x9B,0xD6,0x07,0xAC,0xC3,0xA0,0x3F,\
		0x42,0xDF,0x79,0x3F,0x12,0xB0,0x7A,0x60,0x28,0x20,0xB6,0x3E,0x6B,0xE4,0xB7,0xD8,0xF0,0x8F,0x61,\
		0x20,0x80,0x0C,0x80,0x5C,0x3A,0x16,0x76,0x93,0xF4,0x21,0x06,0x11,0x26,0x03,0x0A,0xC0,0xD8,0x17,0x11,\
	  0xD3,0xF7,0x93,0x7B,0xFB,0xD7,0xAE,0xB7,0x1F,0x81,0x74,0x6B,0x05,0x79,\
		0x9B,0xCC,0x99,0x9F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD,0xD9,0x00,0x01,0x75,0x1A,0x57,0xA6,0x3C,0xED,\
		0x75,0x7A,0x5B,0x5D,0x72,0x5D,0xF2,0xBB,0xD9,0xEC,0x63,0xD8,0x9F,0xA5,0xEF,0x4B,0xB8,0x3D,0x70,\
		0x7A,0xA4,0x3E,0x48,0x7E,0x8E,0xE5,0x1D,0xE0,0x57,0xA0,0xB0,0x5A,0x65,0x00,0x24,0x56,0xEB};
	/*for(int i=0;i<623;i++)
{
	SendBuf[i]=i;
}
for(int i=0;i<300;i++)
		{
			SendBuf[i]=i;
		}*/
int size =sizeof(SendBuf);
while(1)
	{
		
		RadioSend(SendBuf,size);
		//RadioSend(SendBuf,300);
		//if((radiotest==1))
			//UartSendData(ComPorts[Uart_WDR],SendBuf,300);
		vTaskDelay(900);
	}
}

char RadioConfig(unsigned char cmd[6])
{
	int Counter=0;
	UartPrintf("RadioConfig %02X %02X %02X %02X %02X %02X\r\n",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5]);	
	memcpy(ConfigBuf,cmd,6);
	RadioSeting=1;
	vTaskDelay(200);
	HAL_GPIO_WritePin(RADIOM0_GPIO_Port, RADIOM0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RADIOM1_GPIO_Port, RADIOM1_Pin, GPIO_PIN_SET);
	UartSetParameter(&(ComPorts[Uart_WDR]),Uart_WDR_RATE);
	vTaskDelay(100);
	while(RadioSeting==1)
	{
		UartSendData(&(ComPorts[Uart_WDR]),ConfigBuf,6);
		vTaskDelay(150);
		Counter++;
		if(Counter>5)
		{
			replyInf(SETRADIO,OUTTIME);
			userReplyInf("Set Radio Outtime\r\n");
			break;			
		}
	}
	vTaskDelay(100);
	HAL_GPIO_WritePin(RADIOM0_GPIO_Port, RADIOM0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RADIOM1_GPIO_Port, RADIOM1_Pin, GPIO_PIN_RESET);	
	//UartSetParameter(ComPorts[Uart_WDR],Uart_WDR_RATE);
	//SetComSpeed(SysStatus.RadioSpeed);
	if(Counter>5)
		return 1;
	else
	{
		userReplyInf("Set Radio Success\r\n");	
		return 0;
	}
}

void RadioReadConfig()
{
	char readbuf[3];
	readbuf[0]=readbuf[1]=readbuf[2]=0xc1;
	vTaskDelay(300);
	HAL_GPIO_WritePin(RADIOM0_GPIO_Port, RADIOM0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RADIOM1_GPIO_Port, RADIOM1_Pin, GPIO_PIN_SET);
	UartSetParameter(&(ComPorts[Uart_WDR]),Uart_WDR_RATE);
	vTaskDelay(100);

	UartSendData(&(ComPorts[Uart_WDR]),readbuf,3);
	vTaskDelay(500);
	HAL_GPIO_WritePin(RADIOM0_GPIO_Port, RADIOM0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RADIOM1_GPIO_Port, RADIOM1_Pin, GPIO_PIN_RESET);	
	vTaskDelay(300);
	//SetComSpeed(SysStatus.RadioSpeed);
	UartSetParameter(&(ComPorts[Uart_WDR]),Uart_WDR_RATE);
}

static void RadioSetTask(void * argument)
{	
	IndexHead=IndexEnd=0;
	BufFull=0;
	SetWaitTime(SysStatus.RadioSpeed);
	//Cmd[3]=0x38 | SysStatus.RadioSpeed;
	Cmd[3]=SetCmdSpeed(SysStatus.RadioSpeed);
	Cmd[4]=SysStatus.RadioChan;
	if(RadioConfig(Cmd)==0)
	{
		if(WriteStatus()==HAL_OK)
			replyInf(SETRADIO,CMDOK);
		else
			replyInf(SETRADIO,WRITEERROR);
	}		
	//airmode=-1;			
}
uint8_t SetCmdSpeed(uint8_t speed)
{
	switch(speed)
	{
		case 0:
			return 0x3d;
			break;
		case 1:
			return 0x3d;
			break;
		case 2:
			return 0x3d;
			break;
		case 3:
			return 0x3c;
			break;
		case 4:
			return 0x3b;
			break;
		case 5:
			return 0x3a;
			break;
		case 6:
			return 0x39;
			break;
		case 7:
			return 0x38;
			break;
	}
}
void SetComSpeed(uint8_t speed)
{
	UartSetParameter(&(ComPorts[Uart_WDR]),115200);
	return;
	switch(speed)
	{
		case 0:
			UartSetParameter(&(ComPorts[Uart_WDR]),1200);
			break;
		case 1:
			UartSetParameter(&(ComPorts[Uart_WDR]),2400);
			break;
		case 2:
			UartSetParameter(&(ComPorts[Uart_WDR]),4800);
			break;
		case 3:
			UartSetParameter(&(ComPorts[Uart_WDR]),9600);
			break;
		case 4:
			UartSetParameter(&(ComPorts[Uart_WDR]),19200);
			break;
		case 5:
			UartSetParameter(&(ComPorts[Uart_WDR]),38400);
			break;
		case 6:
			UartSetParameter(&(ComPorts[Uart_WDR]),57600);
			break;
		case 7:
			UartSetParameter(&(ComPorts[Uart_WDR]),115200);
			break;
	}	
}
void SetWaitTime(uint8_t radioSpeed)
{
		/*
	0:0.3k 2.86
	1:1.2k 801ms
	2:2.4k 392 ms
	3:4.8k 180ms
	4:9.6k 136ms
	5:19.2k 107ms
	*/
	
		switch(7-radioSpeed)
		{
			case 0:
				WaitTime=1920;
				break;
			case 1:
				WaitTime=960;
				break;
			case 2:
				WaitTime=480;
				break;
			case 3:
				WaitTime=240;
				break;
			case 4:
				WaitTime=120;
				break;
			case 5:
				WaitTime=60;
				break;
			case 6:
				WaitTime=40;
				break;
			case 7:
				WaitTime=20;
				break;
		}
}
/*char SetAirSpeed(char mode)
{

	if(mode < 6)
	{
		static portBASE_TYPE xTaskWoken = pdFALSE;
		airmode=mode;		
		//xSemaphoreGiveFromISR(SetSemaphore,&xTaskWoken );	
		
	  AddTaskFromISR(ExePerformer,RadioSetTask,NULL);
		return 0;
	}
	return 1;
}*/
char SetRadioConfig(uint8_t speed,uint8_t chan)
{
	if(speed > 7)
	{
		replyInf(SETRADIO,PARAERROR);
		return 1;
	}
	if(chan > 0x1F)
	{
		replyInf(SETRADIO,PARAERROR);
		return 1;
	}
		SysStatus.RadioSpeed=speed;
		SysStatus.RadioChan=chan;
		static portBASE_TYPE xTaskWoken = pdFALSE;	  
		//xSemaphoreGiveFromISR(SetSemaphore,&xTaskWoken );			
	  AddTaskFromISR(ExePerformer,RadioSetTask,NULL);
		return 0;	
}

void radio_test(unsigned char * buf ,int len)
{
	unsigned char crcData[4]={0} ;
	unsigned short crcData_;
	unsigned short crcResult;
	static unsigned char recDataBuff[TESTRADIO_BUFFSIZE]={0};
	crcData[0]= buf[len-1]; //换行A
	crcData[1]= buf[len-2]; //回车D

	static uint16_t index=0;
	if(len<TESTRADIO_BUFFSIZE-index){
		memcpy(&recDataBuff[index],buf,len);
		index += len;
	}
	if(crcData[0]==0xa && crcData[1]==0xd){
		crcData[2]= recDataBuff[index-3];
		crcData[3]= recDataBuff[index-4];
		crcData_=(crcData[3]<<8) | (crcData[2]);
		crcResult=CRC_Cal(recDataBuff,index-4);
		if(crcData_ != crcResult){
			static int counter=0;
			counter++;
			UartPrintf("\r\n [radio crc error! %d]\r\n",counter);
//			UartPrintf("\r\n crc recived len:%d, crcData_:%x, crcResult:%x, \r\n",
//				index,crcData_,crcResult);
		}else{
			UartPrintf("\r\n [radio crc ok! ]\r\n");
		}	

		index =0;
	} 

}
