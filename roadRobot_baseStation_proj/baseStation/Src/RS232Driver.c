#include "RS232Driver.h"
#include "cmsis_os.h"
#include "main.h"
#include "BLEDriver.h"
static TaskHandle_t HandleRS232TaskIF = NULL;
static void RS232Task(void * argument);
OnRecvFun OnRS232RecvEx=NULL;
void IniRS232(void)
{
	//OnRS232RecvEx=NULL;
			ComPorts[Uart_232].OnUartRecv=OnRS23Recv;
/*	xTaskCreate( RS232Task,   
							 "RS232TaskIF",     	
							 512,               	
							 NULL,              	
							 1,                 	
							 &HandleRS232TaskIF );*/
}
void OnRS23Recv(unsigned char * buf ,int len)
{
	if(OnRS232RecvEx!=NULL)
		OnRS232RecvEx(buf,len);
}
int  RS232Send(unsigned char * buf ,int len)
{
	UartSendData(&(ComPorts[Uart_232]),buf,len);	
}

static void RS232Task(void * argument)
{
/*	char Cmd[]="Hello world!";
	 while(1)
	 {
		 RS232Send(Cmd,strlen(Cmd));
		 vTaskDelay(300);
	 }*/
}