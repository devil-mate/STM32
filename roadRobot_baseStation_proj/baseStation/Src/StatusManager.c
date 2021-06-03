#include "StatusManager.h"
#include "cmsis_os.h"
#include "FlashOperation.h"

/* Private define ------------------------------------------------------------*/
#define FLASH_USER_START_ADDR   0x080A0000          /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (FLASH_USER_START_ADDR + sizeof(SysStatus_s))  /* End @ of user Flash area */



SysStatus_s SysStatus;
SysRamStatus_s SysRamStatus;
uint32_t GetSector(uint32_t Address);
void IniStatusManager(void)
{
	ReadStatus();
	if(SysStatus.GPSMode>LinkS)
		ResetStatus();
	SysRamStatus.LinkPort=Uart_NULL;
	SysRamStatus.ReadPort=Uart_NULL;
	SysRamStatus.SwitchMode=SysStatus.GPSMode;
	
}
void ResetStatus()
{

	SysStatus.GPSMode=MobileS;
	SysStatus.RTKSource=0;
	SysStatus.FixPos=0;
	SysStatus.Latitude=0;
	SysStatus.Longitude=0;
	SysStatus.Altitude=0;
	SysStatus.AveTime=0.01;
  SysStatus.ServerIP[0]='\0';
	SysStatus.RadioSpeed=0;
	SysStatus.RadioChan=0;
	SysStatus.InterfacePort=Uart_232;
	WriteStatus();
	userReplyInf("Reset Config Done!\r\n");
}

HAL_StatusTypeDef WriteStatus()
{		
	return bsp_WriteCpuFlash(FLASH_USER_START_ADDR,(uint8_t*)&SysStatus,sizeof(SysStatus));
}

void ReadStatus()
{
	uint32_t Address = 0;
	uint32_t* data=(uint32_t* )&SysStatus;
	Address = FLASH_USER_START_ADDR;
  while (Address < FLASH_USER_END_ADDR)
  {
    *data = *(__IO uint32_t*)Address;   
    Address = Address + 4;
		data++;
  }  
}




