#include "USBDriver.h"
#include "usbd_def.h"
extern USBD_HandleTypeDef hUsbDeviceHS;

#define HEAD (0x3487)
#define TAIL (0xA3BC)

#define USB_PRINT_TX_BUF_SIZE (500)
#define USB_RX_BUF_SIZE (2048)

typedef  struct
{
    unsigned short head;
    unsigned short size;
    unsigned char  code;
    unsigned char  check;
    unsigned char  data;
}USBDataHead;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
static char print_usb_buf[USB_PRINT_TX_BUF_SIZE];
const unsigned char CELL_SIZE[]={20,10,2};
const unsigned short CELL_BUF_SIZE[]={128,128,20};
//const unsigned char USB_CHANNEL_SIZE  __attribute__ ((at(0x08010000))) = sizeof(CELL_SIZE)/sizeof(unsigned char);
const unsigned char USB_CHANNEL_SIZE = sizeof(CELL_SIZE)/sizeof(unsigned char);
unsigned char usbRxBuf[USB_RX_BUF_SIZE];
OnUSBRecvFunc onUSBRecvFuncGroup[USB_DOWN_SIZE];
USBChannel usbBuf[USB_CHANNEL_SIZE];
unsigned char USB_Ready=0;

unsigned short frameHeadPos,frameTailPos, frameEndPos,frameIndex;
unsigned char frameHeadFound,frameTailFound;

int assembleFrame(unsigned char * send_buf,unsigned char channel,unsigned char code,unsigned short 	len,unsigned char 	* data);
void iniUSBDriver()
{
	frameHeadPos=frameTailPos=frameEndPos=frameIndex=0;
	frameHeadFound=frameTailFound=0;
	memset(usbBuf,0,sizeof(USBChannel)*USB_CHANNEL_SIZE);
	for(int n=0;n<USB_CHANNEL_SIZE;n++)
	{
		usbBuf[n].cellSize=CELL_SIZE[n];		
		usbBuf[n].cells=(USBBufCell*)malloc(sizeof(USBBufCell)*usbBuf[n].cellSize);
		memset(usbBuf[n].cells,0,sizeof(USBBufCell)*usbBuf[n].cellSize);
		usbBuf[n].sendBuf=malloc(CELL_BUF_SIZE[n]);
		
		for(int i=0;i<usbBuf[n].cellSize;i++)
		{
			usbBuf[n].cells[i].bufSize=CELL_BUF_SIZE[n];
			usbBuf[n].cells[i].data=malloc(CELL_BUF_SIZE[n]);
		}		
	}
	for(int n=0;n<USB_DOWN_SIZE;n++)
	{
		onUSBRecvFuncGroup[n]=NULL;
	}
  HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}
char sendUSBData(unsigned char channel,unsigned short 	len,unsigned char 	* data)
{
	if(hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
		return 7;
	if(USB_Ready==0)
		return 6;
	unsigned char i;
	unsigned char statue;
	if(channel>=USB_CHANNEL_SIZE)
		return 1;
	if(len<=0)
		return 2;
	if(len>CELL_BUF_SIZE[channel])
		return 3;
	
	i=usbBuf[channel].cellIn;
	usbBuf[channel].cellIn=(usbBuf[channel].cellIn+1)%(usbBuf[channel].cellSize);	
	memcpy(usbBuf[channel].cells[i].data,data,len);
	usbBuf[channel].cells[i].len=len;
	usbBuf[channel].cells[i].fresh=1;
	return 0;
}
void sendUSBBuf()
{
	int channel;
	unsigned char statue;
	extern USBD_HandleTypeDef hUsbDeviceHS;
	if(USB_Ready==0)
		return ;
	if(hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
		return ;
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
	if (hcdc->TxState != 0)
		return;
	for(int channel=0;channel<USB_CHANNEL_SIZE;channel++)
	{
		
		int i=usbBuf[channel].cellOut;
		if(i==usbBuf[channel].cellIn)
			continue;
		if(usbBuf[channel].cells[i].fresh==1)
		{
			{

				#ifndef TEST_USB
				usbBuf[channel].cells[i].fresh=0;
				#endif
				usbBuf[channel].cellOut=(usbBuf[channel].cellOut+1)%(usbBuf[channel].cellSize);	
			}			
			statue=CDC_Transmit_HS(usbBuf[channel].cells[i].data,usbBuf[channel].cells[i].len);
			return;
		}
		else
			continue;
	}
}
char sendChannelData(unsigned char channel,unsigned char code,unsigned short 	len,unsigned char 	* data)
{
	unsigned short sent,assembleLen;
	unsigned char  result;
	if(hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
		return 7;
	sent=0;
	
	while(sent<len)
	{
		if((len-sent)<(CELL_BUF_SIZE[channel]-8))
		{
			//printf("sendChannelData %d %d %d\r\n",channel,code,len-sent);
			assembleLen=assembleFrame(usbBuf[channel].sendBuf,channel,code,len-sent,data+sent);
			sent=len;
		}
		else
		{
			
			assembleLen=assembleFrame(usbBuf[channel].sendBuf,channel,code,CELL_BUF_SIZE[channel]-8,data+sent);
			sent+=CELL_BUF_SIZE[channel]-8;
		}
		result=sendUSBData(channel,assembleLen,usbBuf[channel].sendBuf);
		if(result!=0)
			return 1;
	}
	return 0;
}
int assembleFrame(unsigned char * send_buf,unsigned char channel,unsigned char code,unsigned short 	len,unsigned char 	* data)
{
	unsigned short * head;
	unsigned short * tail;
	unsigned short * size;
	unsigned char  * frameCode;
	unsigned char  * check;
	if(len>(CELL_BUF_SIZE[channel]-8))
		return 0;
	head = (unsigned short *)send_buf;
	*head = HEAD;
	size=head+1;
	*size=len;
	frameCode = (unsigned char  *)(size+1);
	*frameCode=code;

	check=frameCode+1;
	*check=0;
	for(unsigned short i=0;i<len;i++)
	{
			*check+=*(((unsigned char  *)data)+i);
			*(check+1+i)=*(data+i);
	}
	//memcpy(check+1,data,len);
	tail=(unsigned short *)(check+1+len);
	*tail=TAIL;
	return len+8;
}
void sendLiveFrame(int index)
{
	sendChannelData(2,USB_LIVE, sizeof(index),(uint8_t*)&index);
}
int USBPrintf(const char *format, ...)
{
    va_list args;
    uint32_t length;
		va_start(args, format);
		if(USB_Ready==0)
			return 0;
		//xSemaphoreTake( USBPrintfMutex, portMAX_DELAY );
		length = vsnprintf((char *)print_usb_buf, USB_PRINT_TX_BUF_SIZE, (char *)format, args);
		va_end(args);
		sendChannelData(1,USB_LOG, length,(uint8_t*)print_usb_buf);
		//xSemaphoreGive( USBPrintfMutex);
    return length;
}
void setUSBCallBackFunc(USB_Down_Code code,OnUSBRecvFunc func)
{
	if(code<USB_DOWN_SIZE)
		onUSBRecvFuncGroup[code]=func;
}
void frameParsing()
{
	  while(frameIndex < frameEndPos)
    {
        if(frameHeadFound==0)
        {
            unsigned short * head =(unsigned short *)(usbRxBuf+frameIndex);
            if(*head==HEAD)
            {
                frameHeadFound=1;
                frameHeadPos=frameIndex;
							  //USBPrintf("[BOOT] get head %u",frameIndex );
            }
            frameIndex++;
        }
        else
        {
            if((frameEndPos-frameHeadPos)>sizeof(USBDataHead))
            {
                USBDataHead * dataHead=(USBDataHead *)(usbRxBuf+frameHeadPos);

                if(dataHead->size+8+frameHeadPos<=frameEndPos)
                {
                    unsigned short * tail=(unsigned short *)(usbRxBuf+dataHead->size+6+frameHeadPos);
                    if(*tail==TAIL)
                    {
											 
                        frameTailPos=dataHead->size+8+frameHeadPos;//??????
                        frameTailFound=1;
											  //USBPrintf("[BOOT] get tail  %u %u",frameHeadPos,dataHead->size+6+frameHeadPos );
                    }
                    else
                    {
                        frameHeadFound=0;
                    }
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }
        }
        if(frameTailFound)
        {
            USBDataHead * dataHead=(USBDataHead *)(usbRxBuf+frameHeadPos);
            unsigned char  check;
            unsigned char * data=&(dataHead->data);
            check=0;
            for(int n=0;n<dataHead->size;n++)
            {
                check+=*(data+n);
            }
						//USBPrintf("[BOOT] check  %u %u",check,dataHead->check );
            if(check==dataHead->check)
            {
							if(dataHead->code<USB_DOWN_SIZE)
							{
								//USBPrintf("[BOOT] onUSBRecvFuncGroup %u %u",dataHead->code,dataHead->size );
								if(onUSBRecvFuncGroup[dataHead->code]!=NULL)
								{
									//USBPrintf("[BOOT] call onUSBRecvFuncGroup %u %u",dataHead->code,dataHead->size );
									onUSBRecvFuncGroup[dataHead->code]((unsigned char * )data,dataHead->size);
								}
							}
							//onUSBDataRecv(dataHead->code,dataHead->size,(unsigned char * )data);
                /*if(dataHead->code < CHANNEL_SIZE)
                {
                    if(answers[dataHead->code] != NULL)
                    {
                        answers[dataHead->code]->OnDataReceive((char * )data, dataHead->size);
                    }
                    else
                    {
                        //TODO ??????
                    }
                }
                else
                {
                    //TODO ??????
                }*/
            }
            else
            {
                //TODO ?????
            }

            int remain=frameEndPos-frameTailPos;
            if(remain>0)
            {
                memmove(usbRxBuf,usbRxBuf+frameTailPos,remain);
                frameEndPos=remain;
            }
            else
            {
                frameEndPos=0;
            }
            frameHeadPos= frameTailPos= frameIndex=0;
            frameHeadFound=frameTailFound=0;
        }
    }
}
void getUSBData(uint8_t* buf, uint32_t *len)
{
    if(buf==NULL)
        return;
    if(*len==0)
        return;
		//USBPrintf("[BOOT] recv data len %u %u",*len,*len + frameEndPos );
    if(*len + frameEndPos > USB_RX_BUF_SIZE)
    {
			frameHeadPos=frameTailPos=frameEndPos=frameIndex=0;
			frameHeadFound=frameTailFound=0;
      return;
    }
    memcpy(usbRxBuf+frameEndPos,buf,*len);
    frameEndPos+=*len;
    frameParsing();
}
void onUSBDataRecv(unsigned char code,unsigned short 	len,unsigned char 	* data)
{ 
	switch(code)
	{
		case USB_FLY_CMD:
			  break;			
	}

}

void StatusWatcherTask(void * argument)
{
	static uint8_t old_usb_status = USBD_STATE_SUSPENDED;
	static uint8_t idelCouter = 0;
		if(hUsbDeviceHS.dev_state != old_usb_status)
    {
        if(hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED)
				{						
					EnableAllIRQ();
					HAL_NVIC_SetPriority(OTG_HS_IRQn, 7, 0);
					USB_Ready=1;
				}
        else
				{
					USB_Ready=0;
					DisableAllIRQ();
					HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);
					
					//UartPrintf("USBD_STATE_SUSPENDED\r\n");
				}
        old_usb_status = hUsbDeviceHS.dev_state;
    }  		
		if(hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
		{
			idelCouter++;
			if(idelCouter>=50)
			{
				//HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);
				MX_USB_DEVICE_Init();
				idelCouter=0;
			}
		} 
}

void EnableAllIRQ()
{
}
void DisableAllIRQ()
{
	}