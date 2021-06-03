#include "SwitchTask.h"
#include "Radio.h"
//#include "UartTest.h"
#include "main.h"
#include "UartDriver.h" 
#include "GPSDriver.h"
#include "BLEDriver.h"
#include "M4GDrive.h"
#include "RS232Driver.h"
#include "BLEInterface.h"
#include <stdbool.h>
 void OnCmdRecv(unsigned char * buf ,int len);
 void OnRS232RecvC(unsigned char * buf ,int len);
 void OnBLERecvC(unsigned char * buf ,int len);
void OnGPSRecvC(unsigned char * buf ,int len);
void OnRTKRecvC(unsigned char * buf ,int len);
void OnRadioRecvC(unsigned char * buf ,int len);
void OnM4GRecvC(unsigned char * buf ,int len);
void OnGPSSetRecvC(unsigned char * buf ,int len);

#ifdef BASE_TEST
#include <stdbool.h>

#endif
 
static void RTKCheck_test();
extern int8_t RTKCheck_ResFlag;
extern uint32_t RTKData_counter;
extern uint32_t RTKCorrect_counter ;
extern uint32_t RTKInCorrect_counter;
extern uint32_t RTKChecTime;
//enum 
//{
//	MobileS=0,
//	BaseS,
//	SetS,
//} SwitchMode ;
//enum 
//{
//	  S_4G = 0,
//    S_GPS1, //GPS2
//    S_GPS2, //GPS3
//    S_WDR,
//    S_BLE,
//    S_NULL = 0xFF //nonexist uart
//} LinkPort,LogPort;
//enum
//{
//	RTK4G=0,
//	RTKWDR,
//} RTKSource;
char BLEIsAT=0;
void IniSwitchTask(void)
{
	OnRadioRecvEx=OnRadioRecvC;
	OnRS232RecvEx=OnRS232RecvC;
	OnBLERecvEx=OnBLERecvC;
	OnGPSRecvEx=OnGPSRecvC;
	OnRTKRecvEx=OnRTKRecvC;
	OnM4GRecvEx=OnM4GRecvC;
	OnGPSSetRecvEx=OnGPSSetRecvC;

	BLEIsAT=0;
  //SetPrintPort(Uart_232);
		
//		SysStatus.SwitchMode=MobileS;
//		SysStatus.RTKSource=RTK4G;
//		SetMobileMode();
}

void OnCmdRecv(unsigned char * buf ,int len)
{
	//printf("------------lllllen=%d \r\n",len);
	BLEMessage(buf , len);
}


void OnRS232RecvC(unsigned char * buf ,int len)
{
	
	if(SysRamStatus.SwitchMode==LinkS)
	{
		switch(SysRamStatus.LinkPort)
		{
			case Uart_4G:
				M4GSend(buf,len);
				break;
			case Uart_GPS1:
				GPS1Send(buf,len);
				break;
			case Uart_GPS2:
				GPS2Send(buf,len);
				break;
			case Uart_WDR:
				RadioSend(buf,len);
				break;
			case Uart_232:
				RS232Send(buf,len);	
				break;
			case Uart_BLE:
				if(memcmp(buf,"+++a",4)==0)
					BLEIsAT=1;
				if(memcmp(buf,"AT+ENTM",7)==0)
					BLEIsAT=0;
				BLESend(buf,len);
				break;
		}
	}
	if(SysStatus.InterfacePort==Uart_232)
		CmdMessage(buf,len);
	if(SysRamStatus.ReadPort==Uart_232){
		//RS232Send(buf,len);
	}

		

}
void OnBLERecvC(unsigned char * buf ,int len)
{
	if(BLEIsAT==0){
		OnCmdRecv(buf,len);
	}
	
	
	if(SysRamStatus.SwitchMode==LinkS)
	{
		if(SysRamStatus.LinkPort==Uart_BLE)
			RS232Send(buf,len);
	}
	if(SysStatus.InterfacePort==Uart_BLE)
		CmdMessage(buf,len);
	if(SysRamStatus.ReadPort==Uart_BLE)
		RS232Send(buf,len);
}

void OnGPSSetRecvC(unsigned char * buf ,int len)
{
	if(SysRamStatus.SwitchMode==LinkS)
	{
		if(SysRamStatus.LinkPort==Uart_GPS1)
			RS232Send(buf,len);
	}
	if(SysStatus.InterfacePort==Uart_GPS1)
		CmdMessage(buf,len);
	if(SysRamStatus.ReadPort==Uart_GPS1)
		RS232Send(buf,len);
}
void OnGPSRecvC(unsigned char * buf ,int len)
{
	if(SysRamStatus.SwitchMode==LinkS)
	{
		if(SysRamStatus.LinkPort==Uart_GPS2)
			RS232Send(buf,len);
	}
	if(SysStatus.InterfacePort==Uart_GPS2)
		CmdMessage(buf,len);
	if(SysRamStatus.ReadPort==Uart_GPS2)
		RS232Send(buf,len);
}
void OnRTKRecvC(unsigned char * buf ,int len)
{
	if(SysRamStatus.SwitchMode==LinkS)
	{
		if(SysRamStatus.LinkPort==Uart_GPS2){
			RS232Send(buf,len);	
			
		}
	}
	if(BaseS==SysRamStatus.SwitchMode)
	{

		RadioSend(buf,len);
		M4GSend(buf,len);
//		static int xxx=0;
//		xxx++;
		//printf("onRTKRecvc\r\n");
		//UartPrintf("onRTKRecvc\r\n");
	}
	if(SysStatus.InterfacePort==Uart_GPS2)
		CmdMessage(buf,len);
	if(SysRamStatus.ReadPort==Uart_GPS2)
		RS232Send(buf,len);


}
void OnRadioRecvC(unsigned char * buf ,int len)
{
	if(SysRamStatus.SwitchMode==LinkS)
	{
		if(SysRamStatus.LinkPort==Uart_WDR){
			RS232Send(buf,len);	
			radio_test(buf,len);
		}
	}
	if(MobileS==SysRamStatus.SwitchMode)
	{
		if(SysStatus.RTKSource==RTKWDR)
			GetRTK(buf,len);
		
	}	
	if(SysStatus.InterfacePort==Uart_WDR)
		CmdMessage(buf,len);
	if(SysRamStatus.ReadPort==Uart_WDR){
		RS232Send(buf,len);
		radio_test(buf,len);
	}
}
void OnM4GRecvC(unsigned char * buf ,int len)
{
	if(SysRamStatus.SwitchMode==LinkS)
	{
		if(SysRamStatus.LinkPort==Uart_4G){
			RS232Send(buf,len);
		}
			
	}		
	if(MobileS==SysRamStatus.SwitchMode)
	{
		if(SysStatus.RTKSource==RTK4G){
			GetRTK(buf,len); 

		}
		
	}	
	if(SysStatus.InterfacePort==Uart_4G)
		CmdMessage(buf,len);
	if(SysRamStatus.ReadPort==Uart_4G){

#ifdef BASE_4G_TEST
		//RTKCheck_test();
#elif
		RS232Send(buf,len);
		static int16_t rateCount =0;
		++rateCount;
		if(rateCount>10){
			RTKCheck_test();
			rateCount=0;
		}
#endif

		
	}
}
 void LinkPortC(int port)
 {
	 static SysStatus_s SysStatus_old;
	 if(port>Uart_BLE)
	 {
		 if(SysRamStatus.SwitchMode==LinkS)
		 {
			 SysRamStatus.SwitchMode=SysStatus.GPSMode;
			 userReplyInf("Link Mode Out\r\n");
		 }
		 else
		 {
			 IllegalInstruction();
		 } 
	 }
	 else
	 {
			 SysRamStatus.SwitchMode=LinkS;
			 SysRamStatus.LinkPort=port;
			 userReplyInf("Link to %d\r\n",port);		 
	 }
 }
 
  void ReadPortC(int port)
 {
	 static SysStatus_s SysStatus_old;
	 if(port>Uart_BLE)
	 {
			 SysRamStatus.ReadPort=Uart_NULL;
			 userReplyInf("Read Mode Out\r\n");
	 }
	 else
	 {
			 SysRamStatus.ReadPort=port;
			 userReplyInf("Read to %d\r\n",port);		 
	 }
 }

 static void RTKCheck_test(){
//	 if(RTKCheck_ResFlag==2){
//		//UartPrintf("\r\n[RTKCheck] Incorrect!_counter[%d], total[%d]\r\n",RTKInCorrect_counter,RTKData_counter);
//			 UartPrintf("\r\n%4.3f, %d, %d, %d, %d\r\n",BASE_4G_TEST_HEAD,RTKInCorrect_counter,RTKCorrect_counter,RTKData_counter
//					,HAL_GetTick());
//		RTKCheck_ResFlag=0;
//	 }else if(RTKCheck_ResFlag ==1){
//			 UartPrintf("\r\n%4.3f, %d, %d, %d, %d\r\n",BASE_4G_TEST_HEAD,RTKInCorrect_counter,RTKCorrect_counter,RTKData_counter
//					,HAL_GetTick());
//		RTKCheck_ResFlag=0;
//	 }else{
//		//UartPrintf("\r\n[RTKCheck] no RTKCheck! total[%d]\r\n",RTKData_counter);
//	 }
	 UartPrintf("\r\n%4.3f, %d, %d, %d, %d\r\n",BASE_4G_TEST_HEAD,RTKInCorrect_counter,RTKCorrect_counter,RTKData_counter
					,RTKChecTime);

 }
 
 