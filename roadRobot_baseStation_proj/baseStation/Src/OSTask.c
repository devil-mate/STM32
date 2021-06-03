#include "OSTask.h"
#include "Radio.h"
//#include "UartTest.h"
#include "main.h"
#include "UartDriver.h" 
#include "GPSDriver.h"
#include "BLEDriver.h"
#include "M4GDrive.h"
#include "RS232Driver.h"
#include "SwitchTask.h"
#include "TaskList.h"
#include "StatusManager.h"
#include "UserInterface.h"
//#include "Laser.h"
//#include "forward_control.h"

//#include "MODBUSTask.h"

//static void AppTaskCreate (void);
extern IWDG_HandleTypeDef hiwdg;

char Test[]="Test RS232\r\n";
void ShowTaskInf(void *arg);
void print_reboot_flag(void);
void clear_reboot_flag(void);
static void TestListTask(void * argument);
void User_Init()
{
	int index =0;
//portCONFIGURE_TIMER_FOR_RUN_TIME_STATS();
	//portGET_RUN_TIME_COUNTER_VALUE() ;
	//printf("\r\n==start\r\n");
	
	
	IniStatusManager();
	IniTaskPerformer();
	InitUart();
	IniSwitchTask();
	IniRadio();
	IniGPS();
	IniBLE();
	Ini4G();
	IniRS232();
		
	IniUserInterface();
	//IniLaser();
	UartPrintf("System Start\r\n");
	print_reboot_flag();
	clear_reboot_flag();

	while(1)
	{
//		static size_t freeHeapSize; 
//	freeHeapSize =xPortGetFreeHeapSize();
//	UartPrintf("init_____ freeHeapSize: %d\r\n",freeHeapSize);
		
		HAL_GPIO_WritePin(STM32S_GPIO_Port, STM32S_Pin, GPIO_PIN_RESET);	
		vTaskDelay(300);
		//AddTask(ExePerformer,TestListTask,NULL);
		HAL_GPIO_WritePin(STM32S_GPIO_Port, STM32S_Pin, GPIO_PIN_SET);
		vTaskDelay(300);
		//UartPrintf("System LED\r\n");	
		//ShowTaskInf(NULL);
#ifdef ENABLE_WATCHDOG
  HAL_IWDG_Refresh(&hiwdg);
#endif
		//
		//RS232Send(Test,strlen(Test));
		//ShowTaskInf(NULL);
		//HAL_IWDG_Refresh(&hiwdg);
	}
	 
}
static void TestListTask(void * argument)
{
		static int Counter=0;
		Counter++;
}
void ShowTaskInf(void *arg)
{
    static char pWriteBuffer[2048];
	  size_t FreeHeapSize;
		vTaskList((char *)&pWriteBuffer);
		UartPrintf("\r\ntask_name task_state priority stack tasK_num\r\n");
		UartPrintf("%s\r\n", pWriteBuffer);   
	  FreeHeapSize=xPortGetFreeHeapSize();
		UartPrintf("Free Heap Size %d\r\n", FreeHeapSize);   
	  FreeHeapSize=xPortGetMinimumEverFreeHeapSize();
	  UartPrintf("Minimum Free Heap Size %d\r\n\r\n", FreeHeapSize); 
	
    return;
}
void print_reboot_flag(void)

{

UartPrintf("reboot flag :0x%04X\r\n",RCC->CSR); //

/*?????????*/

if(RCC->CSR & 1<<31){

UartPrintf("reboot error:Low-power!\r\n"); //?????

}else if(RCC->CSR & 1<<30){

UartPrintf("reboot error:Window watchdog!\r\n"); //?????

}else if(RCC->CSR & 1<<29){

UartPrintf("reboot error:Independent watchdog!\r\n"); //?????

}else if(RCC->CSR & 1<<28){

UartPrintf("reboot error:Software!\r\n"); //????

}else if(RCC->CSR & 1<<27){

UartPrintf("reboot error:POR/PDR!\r\n"); //por/pdr??

}else if(RCC->CSR & 1<<26){

UartPrintf("reboot error:PIN!\r\n"); //NRST????

}else if(RCC->CSR & 1<<25){

UartPrintf("reboot error:BOR!\r\n"); //????RMVF???

}

}

/*???????*/

void clear_reboot_flag(void)

{

RCC->CSR |= 1<<24; //??

}
