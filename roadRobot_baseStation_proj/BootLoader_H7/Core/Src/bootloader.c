#include "bootloader.h"
#include "FlashOperation.h"
#include "stm32h7xx_hal.h"
typedef  void (*iapfun)(void);				//定义一个函数类型的参数.

#define PORM_ADDR (0x08020000)
#define UPDATE_ADDR (0x08120000)
#define HEAD_FLAG 0x12345678
//#define HEAD_FLAG 0x1A3E5678
typedef struct
{
	uint32_t flag;
	uint32_t size;
}UpdataHead;
UpdataHead * head;
iapfun jump2app; 
void update();
void iap_load_app(unsigned long appxaddr);
void bootLoader()
{
		uint32_t delayCnt=0;
		head = (UpdataHead*)UPDATE_ADDR;
		if(head->flag==HEAD_FLAG)
		{
			update();
		}
		delayCnt++;
    while(delayCnt%50000000!=0)
    {delayCnt++;}
		//iap_load_app(PORM_ADDR);
}
void update()
{
	char *src;
	int result;
	for(int n=0;n<((head->size)/0x20000+1);n++)
	{
		result=bsp_EraseCpuFlash(PORM_ADDR+n*0x20000);
		//if(result !=)
	}
	result=bsp_WriteCpuFlash(PORM_ADDR,UPDATE_ADDR+FLASH_CELL_SIZE,head->size);
	result=bsp_EraseCpuFlash(UPDATE_ADDR);
	return;
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
	//if(((*(volatile unsigned long*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法.
	{ 
		jump2app=(iapfun)*(volatile unsigned long*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(volatile unsigned long*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		jump2app();									//跳转到APP.
	}
}