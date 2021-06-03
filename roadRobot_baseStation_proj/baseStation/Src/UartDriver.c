#include "UartDriver.h"
#include <FreeRTOS.h>
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#define FULLCLEAR (3)
#define PRINT_TX_BUF_SIZE (512)
static char print_buf[PRINT_TX_BUF_SIZE];

#define COM_PORT_MSG_SIZE (2048)

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

//ComPort_t ComPorts[Uart_NULL] __attribute__ ((at(0x2000E800)));
 ComPort_t ComPorts[Uart_NULL];

static UartDMABuf_t* dma_buf_create(int size);
static PortId_e getUartId(UART_HandleTypeDef *huart);
uint8_t PrintPort;
uint8_t UartIni=0;
//static SemaphoreHandle_t  SendBuffMutex = NULL;
void InitUart()
{
		//PrintPort=0xff;
	//SendBuffMutex = xSemaphoreCreateMutex();
	  PrintPort=Uart_232;
	  for (int i = 0; i < Uart_NULL; i++)
    {
        memset(ComPorts+i, 0, sizeof(ComPort_t));
    }
		
		ComPorts[Uart_WDR].huart = &huart1;
    ComPorts[Uart_WDR].desc = ENUM_STR(Uart_WDR);
		ComPorts[Uart_WDR].CTR485=NULL;

		ComPorts[Uart_GPS1].huart = &huart2;
    ComPorts[Uart_GPS1].desc = ENUM_STR(Uart_GPS1);
		ComPorts[Uart_GPS1].CTR485=NULL;
		
		ComPorts[Uart_GPS2].huart = &huart3;
    ComPorts[Uart_GPS2].desc = ENUM_STR(Uart_GPS2);
		ComPorts[Uart_GPS2].CTR485=NULL;
		

		ComPorts[Uart_232].huart = &huart5;
    ComPorts[Uart_232].desc = ENUM_STR(Uart_232);
		ComPorts[Uart_232].CTR485=NULL;
		
		ComPorts[Uart_4G].huart = &huart4;
    ComPorts[Uart_4G].desc = ENUM_STR(Uart_4G);
		ComPorts[Uart_4G].CTR485=NULL;

		ComPorts[Uart_BLE].huart = &huart6;
    ComPorts[Uart_BLE].desc = ENUM_STR(Uart_BLE);
		ComPorts[Uart_BLE].CTR485=NULL;


    for (int i = 0; i < Uart_NULL; i++)
    {
			  ComPorts[i].OnUartRecv =NULL;
				ComPorts[i].SendHead=0;
				ComPorts[i].SendPre=0;//本次发送完成后的位置
				ComPorts[i].SendTail=0;	
				ComPorts[i].SendFull=0;
				ComPorts[i].RxBufIndex=0;
				ComPorts[i].FullCounter=0;
				ComPorts[i].IdeaActve=0;
				if(ComPorts[i].CTR485!=NULL)
				{
					HAL_GPIO_WritePin(ComPorts[i].CTR485,ComPorts[i].PIN485,GPIO_PIN_RESET);
				}
				HAL_UART_Receive_DMA(ComPorts[i].huart,ComPorts[i].dmaRxBuf[ComPorts[i].RxBufIndex].buf,DMA_BUF_SIZE);
				__HAL_UART_ENABLE_IT(ComPorts[i].huart,UART_IT_IDLE);
    }
	UartSetParameter(&(ComPorts[Uart_232]),115200);
		UartIni=1;
}

static UartDMABuf_t* dma_buf_create(int size)
{
    /*UartDMABuf_t *dmaBuf;
    dmaBuf = (UartDMABuf_t *)pvPortMalloc(sizeof(UartDMABuf_t));
    memset(dmaBuf, 0, sizeof(UartDMABuf_t));

    if (!dmaBuf)
    {
        return NULL;
    }
    size = (size == 0) ? DMA_BUF_SIZE : size;
    dmaBuf->buf = (uint8_t *)pvPortMalloc(size);
    if (!dmaBuf->buf)
    {
        vPortFree(dmaBuf);
        return NULL;
    }
    return dmaBuf;*/
		return NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    PortId_e id = getUartId(huart);

    if (id != Uart_NULL)
    {
			
			  uint16_t RxLen;
			  uint8_t  RxBufIndexPre=(ComPorts[id].RxBufIndex+1) % READBUFSIZE;
			  ComPorts[id].IdeaActve=0;
				RxLen = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
			  //UartPrintf("[UART]RxCplt : %s %d\r\n ",ComPorts[id].desc,RxLen);
				HAL_UART_Receive_DMA(huart, ComPorts[id].dmaRxBuf[RxBufIndexPre].buf, DMA_BUF_SIZE);
			  if(ComPorts[id].OnUartRecv!=NULL)
				{		
						ComPorts[id].OnUartRecv(ComPorts[id].dmaRxBuf[ComPorts[id].RxBufIndex].buf,RxLen);
				}				
				
				ComPorts[id].RxBufIndex=RxBufIndexPre;
			//USBPrintf("[UART]RxCplt Out: %s",ComPorts[id].desc);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_RxCpltCallback(huart);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(UartIni==0)
		return;
	  PortId_e id = getUartId(huart);
    if (id != Uart_NULL)
    {
			
			ComPorts[id].SendHead =ComPorts[id].SendPre;
			ComPorts[id].SendFull=0;
			if(ComPorts[id].SendHead!=ComPorts[id].SendTail)
			{
				int16_t BufRemainLen=ComPorts[id].SendTail-ComPorts[id].SendHead;
				if(BufRemainLen>0)
				{
					HAL_UART_Transmit_DMA(ComPorts[id].huart, ComPorts[id].SendBuf+ComPorts[id].SendHead, BufRemainLen);
					ComPorts[id].SendPre=ComPorts[id].SendTail;
				}
				else
				{
					HAL_UART_Transmit_DMA(ComPorts[id].huart, ComPorts[id].SendBuf+ComPorts[id].SendHead, SEND_BUF_SIZE-ComPorts[id].SendHead);
					ComPorts[id].SendPre=0;			
				}				
			}
			else
			{
						if(ComPorts[id].CTR485!=NULL)
						{
							HAL_GPIO_WritePin(ComPorts[id].CTR485,ComPorts[id].PIN485,GPIO_PIN_RESET);
						}
			}
			
    }	 
}
void UART_IDLECallBack(UART_HandleTypeDef *huart)
{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);
	  PortId_e i=getUartId(huart);
		if(ComPorts[i].IdeaActve==1)
		{
			HAL_UART_RxCpltCallback(huart);
			//UartPrintf("[UART]IdleActve Error: %s\r\n",ComPorts[i].desc);

			//USBPrintf("[UART]IdeaActve Error: %s",ComPorts[i].desc);
		}
		else
			ComPorts[i].IdeaActve=1;
}

static PortId_e getUartId(UART_HandleTypeDef *huart)
{
    uint8_t i;
    for (i = 0; i < Uart_NULL; i++)
    {
        if (huart == ComPorts[i].huart)
            return (PortId_e)i;
    }

    return (PortId_e)Uart_NULL;
}
int UartSendData(ComPort_t *ComPort,unsigned char * buf ,int len)
{
	int SendLen;
	if(UartIni==0)
		return 0;
	if(ComPort==NULL)
		return 0;
	if(buf==NULL)
		return 0;
	if(len<=0)
		return 0;
	if(ComPort->SendFull!=0)
	{
		ComPort->FullCounter++;
		if(ComPort->FullCounter>FULLCLEAR)
		{
			ComPort->FullCounter=0;
			ComPort->SendFull=0;
			ComPort->huart->gState = HAL_UART_STATE_READY;
		}
		else
			return 0;
	}
	else
		ComPort->FullCounter=0;
		
	
	if(ComPort->SendTail>=ComPort->SendHead)
	{
		uint16_t BufRemainLen;
		BufRemainLen=SEND_BUF_SIZE-ComPort->SendTail;
		if(BufRemainLen>=len)
		{
			memcpy(ComPort->SendBuf+ComPort->SendTail,buf,len);
			ComPort->SendTail+=len;
			if(ComPort->SendTail==SEND_BUF_SIZE)
				ComPort->SendTail=0;
			SendLen=len;
		}
		else
		{
			uint16_t DataRemainLen=len-BufRemainLen;
			memcpy(ComPort->SendBuf+ComPort->SendTail,buf,BufRemainLen);
			SendLen=BufRemainLen;
			
			if(DataRemainLen<=ComPort->SendHead)
			{
				memcpy(ComPort->SendBuf,buf+SendLen,DataRemainLen);
				ComPort->SendTail=DataRemainLen;
				SendLen+=DataRemainLen;
			}
			else
			{
				memcpy(ComPort->SendBuf,buf+SendLen,ComPort->SendHead);
				ComPort->SendTail=ComPort->SendHead;
				SendLen+=ComPort->SendHead;					
			}
		}
	}
	else
	{
		 uint16_t BufRemainLen=ComPort->SendHead-ComPort->SendTail;
		 if(BufRemainLen>=len)
		 {
				memcpy(ComPort->SendBuf+ComPort->SendTail,buf,len);
				ComPort->SendTail+=len;
				SendLen=len;
		 }
		 else
		 {
				memcpy(ComPort->SendBuf+ComPort->SendTail,buf,BufRemainLen);
				ComPort->SendTail=ComPort->SendHead;
				SendLen=BufRemainLen;			 
		 }
	}
	if(ComPort->SendTail==ComPort->SendHead)
		ComPort->SendFull=1;
	if(ComPort->huart->gState == HAL_UART_STATE_READY)
	{
		int16_t BufRemainLen=ComPort->SendTail-ComPort->SendHead;
		if(ComPort->CTR485!=NULL)
		{
			HAL_GPIO_WritePin(ComPort->CTR485,ComPort->PIN485,GPIO_PIN_SET);
		}
		if(BufRemainLen>0)
		{
			HAL_UART_Transmit_DMA(ComPort->huart, ComPort->SendBuf+ComPort->SendHead, BufRemainLen);
			ComPort->SendPre=ComPort->SendTail;
		}
		else
		{
			HAL_UART_Transmit_DMA(ComPort->huart, ComPort->SendBuf+ComPort->SendHead, SEND_BUF_SIZE-ComPort->SendHead);
			ComPort->SendPre=0;			
		}
	}
	return SendLen;
}

char  UartSetParameter(ComPort_t *ComPort,int rate)
{
	if(ComPort==NULL)
		return 0;
  ComPort->huart->Init.BaudRate = rate;
  if (HAL_UART_Init(ComPort->huart) != HAL_OK)
  {
    //Error_Handler();
		return 0;
  }
	else
		return 1;
}



int UartPrintf(const char *format, ...)
{
    va_list args;
    uint32_t length;
 		if(PrintPort<Uart_NULL)
		{
			va_start(args, format);
			length = vsnprintf((char *)print_buf, PRINT_TX_BUF_SIZE, (char *)format, args);
			va_end(args);
			UartSendData(ComPorts+PrintPort,(uint8_t*)print_buf, length);
		}
    return length;
}
char SetPrintPort(uint8_t port)
{
	if(port<Uart_NULL)
	{
		PrintPort=port;
		return port;
	}
	else
	{
		PrintPort=0xff;
		return -1;
	}
	
}