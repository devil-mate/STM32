#include "BLEDriver.h"
#include "cmsis_os.h"
#include "main.h"
#include "Radio.h"
#include "TaskList.h"
#define Uart_BLE_RATE (115200)
OnRecvFun OnBLERecvEx=NULL;
void IniCmd();
static char IniCmd1[]="+++a\r\n";
static char IniCmd2[]="AT+UART=115200,8,0,0\r\n";
static char IniCmd3[]="AT+NAME=GPSM\r\n";
static char IniCmd4[]="AT+ENTM\r\n";
static char IniCmd5[]="AT+UART?\r\n";
static char IniCmd6[]="AT+TPL?\r\n";
static char IniCmd7[]="AT+TPL=8\r\n";
static TaskHandle_t HandleBLETaskIF = NULL;
static void BLETask(void * argument);
char BLEInitialized=0;
static SemaphoreHandle_t  BLEAckWaitSemaphore = NULL;

void IniBLE(void)
{
	ComPorts[Uart_BLE].OnUartRecv=OnBLERecv;
	//OnBLERecvEx=NULL;
	BLEAckWaitSemaphore=xSemaphoreCreateBinary();
	//IniCmd();
	
	//HAL_GPIO_WritePin(POWBLU_GPIO_Port,POWBLU_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLURES_GPIO_Port,BLURES_Pin, GPIO_PIN_SET);
	BLEInitialized=1;
	xTaskCreate( BLETask,   
							 "BLETaskIF",     	
							 64,               	
							 NULL,              	
							 1,                 	
							 &HandleBLETaskIF );
	
	
}
static void BLETask(void * argument)
{
	while(1)
	{
		if(BLEInitialized==0)
			IniCmd();
		vTaskDelay(300);
	}
}
void IniCmd()
{
	BLEInitialized=0;
	UartSetParameter(&(ComPorts[Uart_BLE]),115200);
	vTaskDelay(100);
	UartSendData(&ComPorts[Uart_BLE],IniCmd1,strlen(IniCmd1));
	vTaskDelay(300);
	UartSendData(&ComPorts[Uart_BLE],IniCmd2,strlen(IniCmd2));
	vTaskDelay(300);
//	/*UartSendData(ComPorts[Uart_BLE],IniCmd5,strlen(IniCmd3));
//	vTaskDelay(300);
//	UartSendData(ComPorts[Uart_BLE],IniCmd6,strlen(IniCmd3));
//	vTaskDelay(300);
//	UartSendData(ComPorts[Uart_BLE],IniCmd6,strlen(IniCmd3));
//	vTaskDelay(300);
//	UartSendData(ComPorts[Uart_BLE],IniCmd7,strlen(IniCmd3));
//	vTaskDelay(300);*/
	UartSendData(&ComPorts[Uart_BLE],IniCmd4,strlen(IniCmd4));
	vTaskDelay(300);	
	UartSetParameter(&(ComPorts[Uart_BLE]),Uart_BLE_RATE);
	BLEInitialized=1;
}
void OnBLERecv(unsigned char * buf ,int len)
{
	if(BLEInitialized!=0)
	{
		if(OnBLERecvEx!=NULL){
			OnBLERecvEx(buf,len);
		}
	}
	else
	{
		//printf("xxxxx===\r\n");
		if(len>2)
		{
			buf[2]='\0';
			if(strcmp((char*)buf,"OK")==0)
			{
					static portBASE_TYPE xTaskWoken = pdFALSE;
					xSemaphoreGiveFromISR(BLEAckWaitSemaphore,&xTaskWoken );	
			}		
		}
	}
	

}
int  BLESend(unsigned char * buf ,int len)
{
	if(BLEInitialized==0)
		return 0;
	UartSendData(&(ComPorts[Uart_BLE]),buf,len);


}
void setBlueToothNameTask(void * argument)
{	 
	  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);
		uint8_t cmd[30];
	  uint16_t * check;
		char * name = (char * )argument;
	
  	sprintf(cmd,"AT+NAME%s",name);
		BLEInitialized=0;
		UartSendData(&(ComPorts[Uart_BLE]),cmd,strlen(cmd));
		if(pdTRUE==xSemaphoreTake(BLEAckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{
			//HAL_GPIO_WritePin(BLURES_GPIO_Port,BLURES_Pin, GPIO_PIN_RESET);
			//vTaskDelay(10);
			//HAL_GPIO_WritePin(BLURES_GPIO_Port,BLURES_Pin, GPIO_PIN_SET);
				userReplyInf("Set BlueTooth Done\r\n");
		}
		else
		{
			 userReplyInf("Set BlueTooth Time Out\r\n");
		}
		vTaskDelay(100);
  	sprintf(cmd,"AT+IMMB1");
		BLEInitialized=0;
		UartSendData(&(ComPorts[Uart_BLE]),cmd,strlen(cmd));
		if(pdTRUE==xSemaphoreTake(BLEAckWaitSemaphore, (TickType_t)xMaxBlockTime))
		{
			HAL_GPIO_WritePin(BLURES_GPIO_Port,BLURES_Pin, GPIO_PIN_RESET);
			vTaskDelay(10);
			HAL_GPIO_WritePin(BLURES_GPIO_Port,BLURES_Pin, GPIO_PIN_SET);
				userReplyInf("Close BLE Done\r\n");
		}
		else
		{
			 userReplyInf("Close BLE Time Out\r\n");
		}		
		
		BLEInitialized=1;
}
void setBlueToothName(char * name)
{
			AddTaskFromISR(ExePerformer,setBlueToothNameTask,name);
}