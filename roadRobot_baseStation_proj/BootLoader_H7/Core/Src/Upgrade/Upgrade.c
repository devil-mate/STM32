#include "Upgrade.h"
#include "USBDriver.h"
#include "FlashOperation.h"


#pragma pack(push)
#pragma pack(1)
typedef struct
{
	uint32_t dataLength;
	uint64_t ver;
	uint32_t needUpgrade;
}UpgradeFlag;
typedef struct
{
	uint32_t workMode;
	uint32_t received;
	uint32_t total;
	uint64_t ver;
}RequireHead;
typedef struct
{
	uint32_t index;
	uint32_t total;
	uint64_t ver;
	uint8_t  data;
}ReplyHead;
#pragma pack(pop)
#define NEED_UPGRADE (0x0000AABB)
#define USB_BUF_SIZE (512)
#define USB_BUF_LIST_SIZE (10)
#define FLASH_BUF_SIZE CPU_FLASH_SECTOR_SIZE
//#define FLASH_BUF_SIZE (sizeof(int64_t)*4)
uint32_t dataReceived;
UpgradeFlag * upgradeFlag;
uint64_t * devID;
uint64_t frameID=0;
RequireHead requireHead;
uint8_t dataBuf[FLASH_BUF_SIZE];
uint32_t bufSize;
uint32_t flashWritePos;
iapfun jump2app; 
uint8_t USBBuf[USB_BUF_LIST_SIZE][USB_BUF_SIZE];
uint16_t USBBufSize[USB_BUF_LIST_SIZE];
uint8_t USBBufHead,USBBufTail;

uint32_t burned;
uint8_t USBBufActive;
unsigned char sendFlyStreamFunc(unsigned char * buf ,unsigned int len)
{
	return sendChannelData(0,USB_FLY_DATA,len,buf);
}

unsigned char writeData(uint32_t _ulFlashAddr, uint8_t *_ucpSrc, uint32_t _ulSize)
{
	uint8_t done;
	int result;
	result=bsp_EraseCpuFlash(_ulFlashAddr);
	done=1;
	do
	{
		result=bsp_WriteCpuFlash(_ulFlashAddr,_ucpSrc,_ulSize);
		uint8_t *  check=((uint8_t*)_ulFlashAddr);
		for(uint32_t n = 0 ;n<_ulSize;n++)
		{
			if(*(check+n)!=*(_ucpSrc+n))
			{
				done=0;
				result=bsp_EraseCpuFlash(_ulFlashAddr);
				break;
			}
		}		
	}while(done==0);
	return 0;
}
__asm void MSR_MSP(unsigned long addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}
//跳转到应用程序段
//appxaddr:用户代码起始地址.
void iap_load_app(unsigned long appxaddr)
{
//	for (int i = 0; i < 8; i++)
//{
//        NVIC->ICER[i]=0xFFFFFFFF;
//        NVIC->ICPR[i]=0xFFFFFFFF;
//} 
	extern TIM_HandleTypeDef htim13;
  extern TIM_HandleTypeDef htim14;
  HAL_TIM_Base_Stop(&htim14);
	HAL_TIM_Base_Stop(&htim13);
	//if(((*(volatile unsigned long*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法. 
		jump2app=(iapfun)*(volatile unsigned long*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(volatile unsigned long*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		jump2app();									//跳转到APP.

}
void upgradeFinish()
{
//	unsigned long appxaddr = PORM_ADDR;
//	//if(((*(volatile unsigned long*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法.
//	{ 
//		jump2app=(iapfun)*(volatile unsigned long*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
//		MSR_MSP(*(volatile unsigned long*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
//		jump2app();									//跳转到APP.
//	}
	extern TIM_HandleTypeDef htim13;
  extern TIM_HandleTypeDef htim14;
	HAL_TIM_Base_Stop(&htim14);
	HAL_TIM_Base_Stop(&htim13);
	iap_load_app(PORM_ADDR);
}
unsigned char onUpgradeFrame(ProtocolHead * head , unsigned char * data,uint16_t dataLength)
{
	ReplyHead * replyHead=(ReplyHead *)data;
	uint32_t copySize;
	int result;
	uint32_t frameDataSize;
	uint32_t remainDataSize;
	uint32_t copiedDataSize;
	if(dataLength==0)
		return 0;
	requireHead.total=replyHead->total;
	if(dataLength<=(sizeof(ReplyHead)-sizeof(uint8_t)))
		return 0;
	frameDataSize=dataLength-(sizeof(ReplyHead)-sizeof(uint8_t));
	int32_t bufOffset;
	bufOffset=((int32_t)replyHead->index)-burned;
	USBPrintf("[BOOT] Upgrade id %llu index %u len %u ",head->frame_id, replyHead->index,frameDataSize);
	if(bufOffset<0)
		return 0;
	
	remainDataSize=frameDataSize;
	copiedDataSize=0;
	while(remainDataSize+bufOffset>=FLASH_BUF_SIZE)
	{
		copySize=FLASH_BUF_SIZE-bufOffset;
		memcpy(dataBuf+bufOffset,&(replyHead->data)+copiedDataSize,copySize);
		copiedDataSize+=copySize;
		remainDataSize-=copySize;
		result=writeData(PORM_ADDR+burned,dataBuf,FLASH_BUF_SIZE);
		burned+=FLASH_BUF_SIZE;
		//bufOffset=((int32_t)replyHead->index)-burned;
		bufOffset=0;
		bufSize=0;	
	}
	if(remainDataSize>0)
	{
		memcpy(dataBuf+bufOffset,&(replyHead->data)+copiedDataSize,remainDataSize);
		bufOffset=remainDataSize+bufOffset;
		//bufSize=remainDataSize+bufOffset;		
	}
	
	requireHead.received=bufOffset+burned;
	if(requireHead.received==requireHead.total)
	{
		sendFlyProtocolFunction(CLOUD_CMD_RELAY_HEAD,flyProtocolID,FLY_FIRMWARE,&requireHead,sizeof(requireHead));
		USBPrintf("[BOOT] Upgrade Finish Ver %llu Total %lu Received %lu ",requireHead.ver ,requireHead.total ,requireHead.received);
		HAL_Delay(150U);	
		result=writeData(PORM_ADDR+burned,dataBuf,bufOffset);
		upgradeFinish();
	}
	else{
		sendFlyProtocolFunction(CLOUD_CMD_RELAY_HEAD,flyProtocolID,FLY_FIRMWARE,&requireHead,sizeof(requireHead));
	}
}


	
unsigned char USBUpgradeBack(unsigned char * buf ,int len)
{
	USBPrintf("[BOOT] USBUpgradeBack %u",len);
	if((len<USB_BUF_SIZE))
	{
		memcpy(USBBuf[USBBufHead],buf,len);
		USBBufSize[USBBufHead]=len;
		USBBufHead=(USBBufHead+1)%USB_BUF_LIST_SIZE;
		if(USBBufHead==USBBufTail)
		{
			USBPrintf("[BOOT] usb buf full");
		}
		USBBufActive=1;		
	}
}
void iniUpgrade()
{
	GPIO_PinState upBotton;
	GPIO_PinState needUpBotton;
	USBBufHead=USBBufTail=0;
	requireHead.workMode=10;
	requireHead.received=0;
	requireHead.total=0;
	requireHead.ver=0;
	bufSize=0;
	dataReceived=0;
	flashWritePos=0;
	//USBBufSize=0;
  USBBufActive=0;
	burned=0;
	upgradeFlag = (UpgradeFlag*)FLAG_ADDR;
	flyProtocolID=*((uint64_t*)ID_ADDR);
	frameID=10;
	setFlyProtocolSendFunc(sendFlyStreamFunc);
	SetFlyProtocolID(flyProtocolID);
	//SetFlyProtocolID(824604997699117056);
	setUSBCallBackFunc(USB_FLY_CMD,USBUpgradeBack);
	SetUpgradeFrameFunc(onUpgradeFrame);
	upBotton=HAL_GPIO_ReadPin(DI1_GPIO_Port,DI1_Pin);
	needUpBotton=HAL_GPIO_ReadPin(DI2_GPIO_Port,DI2_Pin);
	if(((upgradeFlag->needUpgrade == NEED_UPGRADE)&&(upBotton==GPIO_PIN_SET))||(needUpBotton==GPIO_PIN_RESET))
	{
		extern USBD_HandleTypeDef hUsbDeviceHS;
		requireHead.ver=upgradeFlag->ver;
		//requireHead.total=upgradeFlag->dataLength;
		//bsp_EraseCpuFlash(PORM_ADDR);
		while(hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
		{
			HAL_Delay(100U);
		}
		
		sendFlyProtocolFunction(CLOUD_CMD_RELAY_HEAD,flyProtocolID,FLY_FIRMWARE,&requireHead,sizeof(requireHead));
	}
	else
	{
		upgradeFinish();
	}
}
void idleUpgradeTask()
{
	static int counter =0 ;
	if(USBBufActive==1)
	{
		while(USBBufHead!=USBBufTail)
		{
			onFlyProtocolDataRecved(USBBuf[USBBufTail],USBBufSize[USBBufTail]);
			USBBufTail=(USBBufTail+1)%USB_BUF_LIST_SIZE;
		}
		
		USBBufActive=0;
	}
	counter++;
}