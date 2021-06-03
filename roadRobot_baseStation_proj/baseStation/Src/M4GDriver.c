#include "M4GDrive.h"
#include "cmsis_os.h"
#include "main.h"
#include "BLEDriver.h"
#include "StatusManager.h"
#include "TaskList.h"
#include "BLEInterface.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define ACKLEN (256)
#define RECVCUT (6)
#define RESTART_TIME (90*2)
#define M4G_BUFFSIZE (1024)
#define Uart_4G_RATE (115200)
static SemaphoreHandle_t  AckWaitSemaphore = NULL;
static TaskHandle_t HandleM4GMainTaskIF = NULL;
static void M4GMainTask(void * argument);
uint16_t SumCheckFun(uint8_t * data , uint16_t len);
char M4GInitialized=0;
OnRecvFun OnM4GRecvEx=NULL;
NetState  M4GState;
char AckData[ACKLEN];
uint16_t AckLen;
static const TickType_t xMaxBlockTime = pdMS_TO_TICKS(2000);
char RecvDataCounter;
static char *tempPort="0000";
static unsigned char *testRTKDataBuf = NULL;
static unsigned char TestRTKDataBuff[]={0xD3, 0x00, 0xBA, 0x43, 0x30, 0x00, 0x6F, 0x27, 0x62, 0x22, 0x00, 0x20, 0x22, 0x08, 0x50, 0x40, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0x01, 0x00, 0x67, 0xF6, 0x67, 0xA4, 0x24, 0x23, 0xA7, 0x24, 0x27, 0x00, 0x00, 0x00, 0x27, 0xD6, 0x57, 0x7C, 0x54, 0xDA, 0xEC, 0x68, 0x02, 0xFF, 0x7C, 0xFF, 0xCD, 0xF0, 0xF7, 0xCC, 0x40, 0xA5, 0xE2, 0xBD, 0xC4, 0xDF, 0xC9, 0xF3, 0x99, 0xD7, 0x37, 0x0E, 0x54, 0xA3, 0x34, 0x06, 0x78, 0x0D, 0x17, 0x14, 0x4E, 0x23, 0xFC, 0x7A, 0x50, 0xF4, 0xE0, 0xEA, 0x42, 0x2E, 0x44, 0x6C, 0x07, 0xEC, 0xEA, 0x1E, 0x43, 0xE6, 0xE2, 0xCF, 0xAA, 0x80, 0xBB, 0x75, 0xF5, 0xEA, 0x58, 0x9F, 0xBC, 0x3F, 0xA0, 0xA8, 0xD5, 0x44, 0x1B, 0xA6, 0x13, 0x94, 0xF0, 0x2D, 0x2A, 0x9F, 0x59, 0xF3, 0x05, 0x0F, 0x22, 0x15, 0xAC, 0xA4, 0x45, 0x77, 0x4F, 0xD4, 0x30, 0xFF, 0xCD, 0x81, 0x0A, 0xD7, 0x6F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFA, 0x7F, 0xFB, 0xBA, 0x00, 0x00, 0x1A, 0x69, 0x9E, 0x7A, 0xDF, 0xA6, 0xB9, 0x4B, 0x26, 0x79, 0xCB, 0x24, 0xF0, 0x13, 0x2B, 0x26, 0x5B, 0xCB, 0xD7, 0x97, 0xC7, 0x2F, 0x8E, 0x60, 0x59, 0xF9, 0xF3, 0xF3, 0xE7, 0xE4, 0xFE, 0x21, 0xFC, 0x1B, 0xC9, 0x1F, 0x92, 0x30, 0xC8, 0xC1, 0x91, 0x43, 0x25, 0x08, 0xD7, 0x00, 0xA7, 0x30, 0x3C, 0xD3, 0x00, 0xD5, 0x43, 0xD0, 0x00, 0xAA, 0xBA, 0x46, 0xE2, 0x00, 0x20, 0x1C, 0x40, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xC0, 0x00, 0x00, 0x7F, 0x7F, 0xFD, 0x0D, 0x0D, 0x39, 0x2D, 0x39, 0x15, 0x17, 0x36, 0x16, 0xD2, 0xB5, 0x61, 0xB2, 0xFC, 0x47, 0x40, 0x31, 0xE3, 0xB7, 0xFC, 0x28, 0x1F, 0xA0, 0xD1, 0xFF, 0x03, 0xF9, 0xD7, 0xFA, 0xD0, 0x48, 0x38, 0x23, 0x70, 0xBE, 0xE1, 0x6F, 0xA7, 0xCB, 0x55, 0x2E, 0xA9, 0xD2, 0xC5, 0xE6, 0x43, 0x7B, 0xF5, 0x79, 0x24, 0xF2, 0x3B, 0xE7, 0x13, 0xD4, 0xCF, 0xA8, 0x50, 0x53, 0xC0, 0xF0, 0xC1, 0xE5, 0x0F, 0x4E, 0x20, 0x62, 0x40, 0x7C, 0x0F, 0x67, 0xFF, 0x5F, 0x7C, 0xFD, 0x8D, 0x0E, 0x00, 0x13, 0xD8, 0x09, 0x31, 0x90, 0x29, 0xD0, 0x46, 0x08, 0xB5, 0x0E, 0xA9, 0x60, 0x13, 0x7F, 0x8F, 0xEF, 0x37, 0xFF, 0xC0, 0x38, 0xF2, 0x1F, 0xF3, 0xE3, 0x3A, 0x1F, 0x73, 0xB8, 0x43, 0xE9, 0x66, 0x0C, 0xE6, 0xE4, 0x37, 0x9B, 0xB1, 0x39, 0xC9, 0x3E, 0xC2, 0xB3, 0xFB, 0x0A, 0xDB, 0xFF, 0xFF, 0xFF, 0xA3, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x34, 0xCF, 0x3D, 0x74, 0xCE, 0x66, 0xEC, 0xB2, 0xB8, 0xA0, 0x77, 0x0D, 0x33, 0x92, 0xBA, 0xF0, 0x74, 0xE0, 0xF2, 0xC1, 0xE0, 0x5C, 0x88, 0xB9, 0x41, 0x72, 0x62, 0x9B, 0xA5, 0x3D, 0x3A, 0x22, 0xF4, 0x4F, 0xE8, 0x8F, 0xD3, 0xAB, 0xA6, 0x7F, 0x4D, 0xBE, 0x5C, 0x9C, 0xB9, 0xB9, 0x72, 0x9E, 0x73, 0x3C, 0xEC, 0x79, 0xDC, 0x9F, 0x6B, 0xB4, 0xD3, 0x00, 0xE7, 0x46, 0x50, 0x00, 0x6F, 0x26, 0x87, 0x60, 0x00, 0x20, 0x1E, 0xB6, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00, 0x77, 0xFF, 0xFC, 0xF5, 0x07, 0x01, 0x0A, 0xF6, 0xF4, 0x9A, 0xF2, 0x91, 0x0E, 0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x7C, 0x19, 0xB6, 0x88, 0xB6, 0x25, 0x15, 0xC1, 0xFE, 0xEB, 0xDA, 0x98, 0x1F, 0x80, 0x01, 0xFF, 0xF0, 0x00, 0x80, 0x95, 0xFF, 0x79, 0xF8, 0x97, 0xE1, 0x3F, 0xE8, 0x7F, 0xF0, 0x09, 0xC0, 0x2B, 0x39, 0xFC, 0x73, 0xBC, 0x6A, 0x5B, 0xF1, 0x73, 0xE3, 0x22, 0xFD, 0xD6, 0x0A, 0x62, 0x70, 0xA4, 0xBB, 0xF3, 0xAA, 0xE7, 0xA2, 0x07, 0xFA, 0x10, 0xA3, 0xDF, 0xB7, 0xD1, 0x4C, 0x6A, 0x98, 0xCB, 0x31, 0x49, 0x64, 0xB6, 0x1B, 0x61, 0xD4, 0x4A, 0x87, 0x50, 0xC7, 0x80, 0x8D, 0x7B, 0x03, 0x3A, 0xF2, 0x0E, 0x19, 0x57, 0x49, 0x36, 0xDC, 0xD3, 0xB1, 0x06, 0xC1, 0x8E, 0x06, 0x67, 0x18, 0x19, 0x58, 0xC0, 0x37, 0x1E, 0x02, 0x22, 0x49, 0xF6, 0xE4, 0x57, 0xF7, 0x84, 0x3F, 0x87, 0x05, 0x75, 0x9F, 0x77, 0xC6, 0xD5, 0x18, 0x03, 0xF5, 0x5D, 0xF4, 0x16, 0x81, 0x57, 0xF3, 0xFF, 0x5F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x00, 0x00, 0x19, 0x63, 0x2D, 0xB6, 0x51, 0x36, 0x18, 0xDF, 0x86, 0x38, 0xE3, 0x86, 0x79, 0xCC, 0xF6, 0x46, 0xBB, 0x4D, 0x69, 0x5D, 0x2C, 0x2F, 0x8C, 0x60, 0x00, 0xF9, 0xEA, 0x12, 0x6D, 0xB8, 0xDB, 0x8D, 0x2F, 0x19, 0xDE, 0xF4, 0xFD, 0x59, 0xFC, 0xBF, 0x0B, 0xEE, 0x35, 0x5B, 0x53, 0xF6, 0xA1, 0x8B, 0x6B, 0x9C, 0x66, 0x3A, 0x04, 0xA2, 0x54, 0x24};
static uint16_t testRTKDataLen = 0;
static void m4G_Send_test();
void Ini4G(void)
{
	//OnM4GRecvEx=NULL;
	M4GState=NotNet;
	M4GState=ActServer;
	AckWaitSemaphore=xSemaphoreCreateBinary();
	RecvDataCounter=0;
	ComPorts[Uart_4G].OnUartRecv=On4GRecv;
//	HAL_GPIO_WritePin(POW4G_GPIO_Port,POW4G_Pin,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(T4GRES_GPIO_Port,T4GRES_Pin,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //复位引脚为高，才能正常工作
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); 
	UartSetParameter(&(ComPorts[Uart_4G]),Uart_4G_RATE);
	xTaskCreate( M4GMainTask,   
							 "M4GMainTask",     	
							 64,               	
							 NULL,              	
							 1,                 	
							 &HandleM4GMainTaskIF );
	
	M4GInitialized=1;
}
void On4GRecv(unsigned char * buf ,int len)
{
	if(M4GInitialized!=0)
	{
		if(OnM4GRecvEx!=NULL)
			OnM4GRecvEx(buf,len);		
		if(SysStatus.GPSMode==MobileS)
		{
				RecvDataCounter=RECVCUT;
		}
	}
	else
	{
		if(len<ACKLEN)
		{
			static portBASE_TYPE xTaskWoken = pdFALSE;
			memcpy(AckData,buf,len);
			AckLen=len;	
			xSemaphoreGiveFromISR(AckWaitSemaphore,&xTaskWoken );						
		}
	
	}
}
int  M4GSend(unsigned char * buf ,int len)
{
	if(M4GInitialized==0)
		return 0;
	UartSendData(&(ComPorts[Uart_4G]),buf,len);	 
}

void M4GLEDTask(void * argument)
{
	static int notActServerCounter=0;
	if(M4GState!=NotNet)
	{
		if((SysStatus.GPSMode!=BaseS))
		{
			if((RecvDataCounter>0))
			{
				RecvDataCounter--;
				M4GState=ActServer;
			}
			else
			{
				M4GState=ActNet;
			}
		}
	}


			
		switch (M4GState)
		{
			 case NotNet:
				 HAL_GPIO_WritePin(T4GCS_GPIO_Port,T4GCS_Pin,GPIO_PIN_RESET);
				 break;
			 case ActNet:
				 HAL_GPIO_TogglePin(T4GCS_GPIO_Port,T4GCS_Pin);
				 break;
			 case ActServer:
				 HAL_GPIO_WritePin(T4GCS_GPIO_Port,T4GCS_Pin,GPIO_PIN_SET);
				 break;
		}	
		
		
		if(M4GState!=ActServer)
		{
			notActServerCounter++;
			if(notActServerCounter>RESTART_TIME)
			{
				notActServerCounter=0;
				Restart4G();
			}
		}
}

void CheckM4Task(void * argument)
{
	static int notRecvCounter=0;
	
}
void M4GMainTask(void * argument)
{
	int timer=0;
	while(1)
	{
		vTaskDelay(10);
#ifdef BASE_4G_TEST_SEND
		if (Test4GFlag){
			if(timer%30==0){
				m4G_Send_test();
			}
		}
#endif
		if((timer%50)==0)
		{
			//AddTaskFromISR(ExePerformer,M4GLEDTask,NULL);
			AddTask(ExePerformer,M4GLEDTask,NULL);
		}

		timer++;
	}	
}
void RestartM4GTask(void * argument)
{	 
		UartPrintf("Restart 4G\r\n");
		HAL_GPIO_WritePin(T4GRES_GPIO_Port,T4GRES_Pin,GPIO_PIN_RESET);
		vTaskDelay(100);
		HAL_GPIO_WritePin(T4GRES_GPIO_Port,T4GRES_Pin,GPIO_PIN_SET);
}
/********YeeCom*********/
void SetServerIPTask(void * argument)
{	 
	
	static uint8_t cmd_arr[30];
	static char *cmdYee ="AT*SERVER1=0,";
	char cmdYeeTail[10]={0};
	strcat(cmdYeeTail,",");
	strcat(cmdYeeTail,tempPort);
	strcat(cmdYeeTail,"#");
	uint16_t cmdLen = strlen(cmdYee);
	uint16_t cmdTailLen = strlen(cmdYeeTail);
	for(uint8_t i=0;i<cmdLen;i++){
		cmd_arr[i]=cmdYee[i];
	}
	  uint16_t * check;
		char * IP = (char * )argument;
	

		uint16_t IPLen=strlen(IP);
		if(IPLen==0||IPLen>18)
		{
			replyInf(SETSERVERIP,PARAERROR);
			IllegalInstruction();
			return;
		}
		M4GInitialized=0;
		UartPrintf("Set Server IP To %s\r\n",IP);
		
		for(uint16_t n=0;n<IPLen;n++)
		{
			cmd_arr[cmdLen+n] = IP[n]; 
		}
		for(uint8_t i=0;i<cmdLen;i++){
			cmd_arr[cmdLen+IPLen+i]=cmdYeeTail[i];
		}
		UartSendData(&(ComPorts[Uart_232]),cmd_arr,cmdLen+IPLen+cmdTailLen);
		//UartSendData(&(ComPorts[Uart_4G]),cmd_arr,cmdLen+IPLen+cmdTailLen);
		if(pdTRUE==xSemaphoreTake(AckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{	
				strcpy(SysStatus.ServerIP,IP);
			int16_t porti;
				sscanf(tempPort,"%d",&porti);
				SysStatus.ServerPort=porti;
				if(AckData[5]==0xF0)
				{
					
					if(WriteStatus()==HAL_OK)
					{
						replyInf(SETSERVERIP,CMDOK);
						userReplyInf("Set IP Done\r\n");
					}
					else
					{
						replyInf(SETSERVERIP,WRITEERROR);
						userReplyInf("Save Data Error\r\n");
					}
						
					
					RestartM4GTask(NULL);

				}					
				else
				{
					replyInf(SETSERVERIP,REPLYERROR);
					userReplyInf("Set IP Fail\r\n");	
				}
		}
		else
		{
				replyInf(SETSERVERIP,OUTTIME);
			 userReplyInf("Set IP Out Time\r\n");
		}
		M4GInitialized=1;
}
//void SetServerIPTask(void * argument)
//{	 
//	/********YeeCom*********/
//	static uint8_t cmd_arr[30];
//	static char *cmdYee ="AT*SERVER1=0,";
//	static char *cmdYeeTail =",60003#";
//	uint16_t cmdLen = strlen(cmdYee);
//	uint16_t cmdTailLen = strlen(cmdYeeTail);
//	for(uint8_t i=0;i<cmdLen;i++){
//		cmd_arr[i]=cmdYee[i];
//	}
//	/********YeeCom*********/
//		
//		uint8_t cmd[30];
//	  uint16_t * check;
//		char * IP = (char * )argument;
//	
//		cmd[0]=0xAA;
//		cmd[1]=0x55;
//		cmd[2]=0x00;
//		cmd[4]=0x00;
//		cmd[5]=0x41;
//		uint16_t IPLen=strlen(IP);
//		if(IPLen==0||IPLen>18)
//		{
//			replyInf(SETSERVERIP,PARAERROR);
//			IllegalInstruction();
//			return;
//		}
//		M4GInitialized=0;
//		UartPrintf("Set Server IP To %s\r\n",IP);
//		cmd[3]=IPLen+4;
//		for(uint16_t n=0;n<IPLen;n++)
//		{
//			//cmd[6+n]=IP[n];
//			cmd_arr[cmdLen+n] = IP[n]; 
//		}
//		for(uint8_t i=0;i<cmdLen;i++){
//			cmd_arr[cmdLen+IPLen+i]=cmdYeeTail[i];
//		}
//		 
//		check=(uint16_t *)(cmd+IPLen+6);
//		*check=SumCheckFun(cmd+2,IPLen+4);		
//		//UartSendData(&(ComPorts[Uart_4G]),cmd,IPLen+8);
//		UartSendData(&(ComPorts[Uart_232]),cmd_arr,cmdLen+IPLen+cmdTailLen);
//		UartSendData(&(ComPorts[Uart_4G]),cmd_arr,cmdLen+IPLen+cmdTailLen);
//		if(pdTRUE==xSemaphoreTake(AckWaitSemaphore, (TickType_t)xMaxBlockTime))
//		{
//				if(AckData[5]==0xF0)
//				{
//					strcpy(SysStatus.ServerIP,IP);
//					if(WriteStatus()==HAL_OK)
//					{
//						replyInf(SETSERVERIP,CMDOK);
//						userReplyInf("Set IP Done\r\n");
//					}
//					else
//					{
//						replyInf(SETSERVERIP,WRITEERROR);
//						userReplyInf("Save Data Error\r\n");
//					}
//						
//					
//					RestartM4GTask(NULL);

//				}					
//				else
//				{
//					replyInf(SETSERVERIP,REPLYERROR);
//					userReplyInf("Set IP Fail\r\n");	
//				}
//		}
//		else
//		{
//				replyInf(SETSERVERIP,OUTTIME);
//			 userReplyInf("Set IP Out Time\r\n");
//		}
//		M4GInitialized=1;
//}

void SetServerPortTask(void * argument)
{	 
	tempPort = (char * )argument;
	
	
		uint8_t cmd[20];
	  uint16_t * check;
	  int porti;
 		char * Port = (char * )argument;
		cmd[0]=0xAA;
		cmd[1]=0x55;
		cmd[2]=0x00;
		cmd[4]=0x00;
		cmd[5]=0x42;
		uint16_t PortPLen=strlen(Port);
		if(PortPLen==0||PortPLen>18)
		{
			IllegalInstruction();
			replyInf(SETSERVERPORT,PARAERROR);
			return;
		}
		sscanf(Port,"%d",&porti);
		if((porti<=0)||(porti>65536))
		{
			replyInf(SETSERVERPORT,PARAERROR);
			IllegalInstruction();
			return;			
		}
		M4GInitialized=0;
		UartPrintf("Set Server Port To %s\r\n",Port);
		cmd[3]=PortPLen+4;
		for(uint16_t n=0;n<PortPLen;n++)
		{
			cmd[6+n]=Port[n];
		}
		check=(uint16_t *)(cmd+PortPLen+6);
		*check=SumCheckFun(cmd+2,PortPLen+4);		
		UartSendData(&(ComPorts[Uart_4G]),cmd,PortPLen+8);
		if(pdTRUE==xSemaphoreTake(AckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{
				if(AckData[5]==0xF0)
				{
					SysStatus.ServerPort=porti;
					if(WriteStatus()==HAL_OK)
					{
						replyInf(SETSERVERPORT,CMDOK);
						userReplyInf("Set Port Done\r\n");
					}
					else
					{
						replyInf(SETSERVERPORT,WRITEERROR);
						userReplyInf("Save Data Error\r\n");
					}

					RestartM4GTask(NULL);
				}
				else
				{
					replyInf(SETSERVERPORT,REPLYERROR);
					userReplyInf("Set Port Fail\r\n");
				}
		}
		else
		{
			 replyInf(SETSERVERPORT,OUTTIME);
			 userReplyInf("Set Port Out Time\r\n");
		}
		M4GInitialized=1;
}

void IniDevTask(void * argument)
{	 
		uint8_t cmd[25];
	  uint16_t * check;
	  int porti;
 		char * Port = (char * )argument;
		UartSetParameter(&(ComPorts[Uart_4G]),Uart_4G_RATE);
		M4GInitialized=0;
	//bps
		cmd[0]=0xAA;
		cmd[1]=0x55;
		cmd[2]=0x00;
		cmd[3]=0x0A;
		cmd[4]=0x00;
		cmd[5]=0x45;
		cmd[6]=0x31;
		cmd[7]=0x31;
		cmd[8]=0x35;
		cmd[9]=0x32;
		cmd[10]=0x30;
		cmd[11]=0x30;
		cmd[12]=0x01;
		cmd[13]=0x78;           
		UartSendData(&(ComPorts[Uart_4G]),cmd,14);
		if(pdTRUE==xSemaphoreTake(AckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{
				if(AckData[5]==0xF0)
				{
					
				}
				else
				{
					userReplyInf("Ini 4G BPS Fail\r\n");
					return;
				}
		}
		else
		{
					userReplyInf("Ini 4G BPS Fail\r\n");
					return;
		}
		//wulian
		cmd[0]=0xAA;
		cmd[1]=0x55;
		cmd[2]=0x00;
		cmd[3]=0x05;
		cmd[4]=0x00;
		cmd[5]=0x90;
		cmd[6]=0x00;
		cmd[7]=0x00;
		cmd[8]=0x95;          
		UartSendData(&(ComPorts[Uart_4G]),cmd,9);
		if(pdTRUE==xSemaphoreTake(AckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{
				if(AckData[5]==0xF0)
				{
					
				}
				else
				{
					userReplyInf("Ini 4G SERVER Fail\r\n");
					return;
				}
		}
		else
		{
					userReplyInf("Ini 4G SERVER Fail\r\n");
					return;
		}
		//communication
		cmd[0]=0xAA;
		cmd[1]=0x55;
		cmd[2]=0x00;
		cmd[3]=0x05;
		cmd[4]=0x00;
		cmd[5]=0x40;
		cmd[6]=0x02;
		cmd[7]=0x00;
		cmd[8]=0x47;          
		UartSendData(&(ComPorts[Uart_4G]),cmd,9);
		if(pdTRUE==xSemaphoreTake(AckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{
				if(AckData[5]==0xF0)
				{
					
				}
				else
				{
					userReplyInf("Ini 4G Client Fail\r\n");
					return;
				}
		}
		else
		{
					userReplyInf("Ini 4G Client Fail\r\n");
					return;
		}
		//IP
		cmd[0]=0xAA;
		cmd[1]=0x55;
		cmd[2]=0x00;
		cmd[3]=0x12;
		cmd[4]=0x00;
		cmd[5]=0x41;
		cmd[6]=0x31;
		cmd[7]=0x31;
		cmd[8]=0x38;
		cmd[9]=0x2E;
		cmd[10]=0x31;
		cmd[11]=0x37;
		cmd[12]=0x38;
		cmd[13]=0x2E;
		cmd[14]=0x31;
		cmd[15]=0x33;
		cmd[16]=0x39;
		cmd[17]=0x2E;
		cmd[18]=0x39;
		cmd[19]=0x32;
		cmd[20]=0x03;
		cmd[21]=0x1F;
		
		UartSendData(&(ComPorts[Uart_4G]),cmd,22);
		if(pdTRUE==xSemaphoreTake(AckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{
				if(AckData[5]==0xF0)
				{
					
				}
				else
				{
					userReplyInf("Ini 4G IP Fail\r\n");
					return;
				}
		}
		else
		{
					userReplyInf("Ini 4G IP Fail\r\n");
					return;
		}
		
				//PORT
		cmd[0]=0xAA;
		cmd[1]=0x55;
		cmd[2]=0x00;
		cmd[3]=0x09;
		cmd[4]=0x00;
		cmd[5]=0x42;
		cmd[6]=0x36;
		cmd[7]=0x30;
		cmd[8]=0x30;
		cmd[9]=0x30;
		cmd[10]=0x32;
		cmd[11]=0x01;
		cmd[12]=0x43;
		
		UartSendData(&(ComPorts[Uart_4G]),cmd,13);
		if(pdTRUE==xSemaphoreTake(AckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{
				if(AckData[5]==0xF0)
				{
					
				}
				else
				{
					userReplyInf("Ini 4G Port Fail\r\n");
					return;
				}
		}
		else
		{
					userReplyInf("Ini 4G Port Fail\r\n");
					return;
		}
		
		UartSetParameter(&(ComPorts[Uart_4G]),Uart_4G_RATE);
		strcpy(SysStatus.ServerIP,"118.178.139.92");
		SysStatus.ServerPort=porti;
		if(WriteStatus()==HAL_OK)
		{
			userReplyInf("Ini Dev\r\n");
		}
		else
		{
			userReplyInf("Save Data Error\r\n");
		}
						

		RestartM4GTask(NULL);
		M4GInitialized=1;
}
int SetServerPort(int Port)
{
	static char Port_t[20];
	if((Port>0)&&(Port<65536))
	{
		sprintf(Port_t,"%d",Port);
		AddTaskFromISR(ExePerformer,SetServerPortTask,&Port_t);
	}
	else
	{
		replyInf(SETSERVERPORT,PARAERROR);
		IllegalInstruction();
	}
}

void Ini4GCommand()
{
		AddTaskFromISR(ExePerformer,IniDevTask,NULL);
}

int SetServerIP(char  * IP)
{
	static char IP_t[20];
	if(strlen(IP)<19)
	{
		strcpy(IP_t,IP);
		AddTaskFromISR(ExePerformer,SetServerIPTask,&IP_t);
	}
	else
	{	  
			replyInf(SETSERVERIP,PARAERROR);
			IllegalInstruction();
	}
}
void Restart4G()
{
		AddTaskFromISR(ExePerformer,RestartM4GTask,NULL);		
}	

void M4GStatePuls()
{
		static TickType_t M4GStatusTimeLast1,M4GStatusTimeLast2;
		char   CheckTime;
		TickType_t M4GStatusTime;
		M4GStatusTime = xTaskGetTickCount();
		
		CheckTime=((M4GStatusTime-M4GStatusTimeLast2+configTICK_RATE_HZ/10)/configTICK_RATE_HZ);
		//UartPrintf("CheckTime : %d \r\n",CheckTime);
		if(CheckTime==2)
		{
			M4GState=NotNet;
			UartPrintf("NotNet\r\n");			
		}
		else if (CheckTime==1)
		{
			M4GState=ActNet;
			UartPrintf("ActNet\r\n");	
		}
		else if((CheckTime==10)||(CheckTime==9)||(CheckTime==11))
		{
			if(SysStatus.GPSMode==BaseS)
			{
					M4GState=ActServer;
					UartPrintf("ActServer\r\n");	
			}
		}
		M4GState=ActServer;
		M4GStatusTimeLast2=M4GStatusTimeLast1;		
		M4GStatusTimeLast1=M4GStatusTime;
}

uint16_t SumCheckFun(uint8_t * data , uint16_t len)
{
	uint16_t sum=0;
	for(uint16_t n=0;n<len;n++)
	{
		sum+=data[n];
	}
	sum=(sum>>8&0xff)|(sum<<8&0xff00);
	return sum;
}
//void M4G_CRC_test(unsigned char * buf ,int len)
//{
//	unsigned char crcData[4]={0} ;
//	unsigned short crcData_;
//	unsigned short crcResult;
//	static unsigned char recDataBuff[M4G_BUFFSIZE]={0};
//	crcData[0]= buf[len-1]; //换行A
//	crcData[1]= buf[len-2]; //回车D

//	static uint16_t index=0;
//	if(len<M4G_BUFFSIZE-index){
//		memcpy(&recDataBuff[index],buf,len);
//		index += len;
//	}
//	if(crcData[0]==0xa && crcData[1]==0xd){
//		crcData[2]= recDataBuff[index-3];
//		crcData[3]= recDataBuff[index-4];
//		crcData_=(crcData[3]<<8) | (crcData[2]);
//		crcResult=CRC_Cal(recDataBuff,index-4);
//		if(crcData_ != crcResult){
//			static int counter=0;
//			counter++;
//			UartPrintf("\r\n [M4G] crc error! %d\r\n",counter);
//		}	
//		UartPrintf("\r\n [M4G] crc recived len:%d, crcData_:%x, crcResult:%x, \r\n",
//				index,crcData_,crcResult);
//		index =0;
//	} else{
//		UartPrintf("\r\n [M4G] not find \\r\\n");
//	}

//}
//************test*************/
static void m4G_Send_test(){
	
	//M4GSend(testRTKDataBuf,testRTKDataLen);
	M4GSend(TestRTKDataBuff,sizeof(TestRTKDataBuff));
	//UartSendData(&(ComPorts[Uart_232]),testRTKDataBuf,testRTKDataLen);

}
void setRTKData(unsigned char * buf ,int len){
	static bool onceFlag =false;
	if(!onceFlag){
		if(testRTKDataBuf!=NULL){
			free(testRTKDataBuf);
		}
		testRTKDataBuf = (unsigned char*)malloc(len);
		memcpy(testRTKDataBuf,buf,len);	
		testRTKDataLen = len;
		onceFlag = true;
	}
		

}