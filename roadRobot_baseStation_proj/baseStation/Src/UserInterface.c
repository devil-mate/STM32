#include "UserInterface.h"
#include "Radio.h"
//#include "UartTest.h"
#include "main.h"
#include "UartDriver.h" 
#include "GPSDriver.h"
#include "BLEDriver.h"
#include "M4GDrive.h"
#include "RS232Driver.h"
#include <string.h>
#include <stdarg.h>
#include "Radio.h"
#include "cm_backtrace.h"
#include "version.h"
#include "BackConter.h"
#define   CMDDATASIZE (512)
#define IU_PRINT_TX_BUF_SIZE (TRACE_PRINT_TX_BUF_SIZE+5)
static char print_buf[IU_PRINT_TX_BUF_SIZE];
typedef enum
{
    SETPOS_C = 0,
    SETAVE_C,
    SETRTK_C,
    SETSERVERIP_C,
    SETSERVERPORT_C,
    SETMOBILEMODE_C,
    SETBASEMODE_C ,
		RESTARTGPS_C ,
		RESTART4G_C ,
		RESETCONFIG_C ,
		LINKPORT_C ,
		READPORT_C ,
		RADIO_C, 
		PRINT_C,
		GETINFO_C,
		SETBTNAME_C,
		INI4G_C,
		

		GETDUMP_C,
		UPDATE_C,
		GETVER_C,
		RESTART_C,
	  GETFLAG_C,
	  GENBUG_C,
		GENAST_C,
		TEST4GSend_C,
		FAILCOMMAND 
} CmdType_e;

TaskPerformer_s * UIPerformer;
static char CmdData[CMDDATASIZE];
static uint16_t CmdDataLen;
void ParseInstruction(void * Para);
CmdType_e InstructionNameToCode(char * Name);
void getDevInf();
void System_SoftReset(void);
void uartPrintConter();
void IniUserInterface()
{
	CmdDataLen=0;
	SysStatus.InterfacePort=Uart_232;
	UIPerformer=CreateTaskPerformer(512,5,"UIPerformer");
}

void CmdMessage(char * cmd , uint16_t len)
{
	if(CMDDATASIZE>(len+CmdDataLen))
	{
		memcpy(CmdData+CmdDataLen,cmd,len);
		CmdDataLen+=len;
		AddTaskFromISR(UIPerformer,ParseInstruction,NULL);
	}
	else
		CmdDataLen=0;
}

void ParseInstruction(void * Para)
{
	uint16_t Index;
	uint16_t SpacePos[25];
	uint8_t  SpacePosIndex=0;
	for(Index=0;Index<CmdDataLen;Index++)
	{
		if(CmdData[Index]=='\r')
		{
			break;
		}
		else if(CmdData[Index]==' ')
		{
			SpacePos[SpacePosIndex]=Index;
			SpacePosIndex++;
			if(SpacePosIndex==25)
			{
				CmdDataLen=0;
				return;
			}
		}
		else
		{
			//CmdData[Index]=downChar(CmdData[Index]);
		}
	}
	if(Index>=CmdDataLen)
		return;
  	for(int n=0;n<SpacePosIndex;n++)
	{
		CmdData[SpacePos[n]]='\0';
	}
	CmdData[Index]='\0';
	switch (InstructionNameToCode(CmdData))
	{
		case SETPOS_C:
			{
				if(SysStatus.GPSMode==BaseS)
				{
					double BLH[3];
					sscanf(CmdData+1+SpacePos[0],"%lf",BLH);
					sscanf(CmdData+1+SpacePos[1],"%lf",BLH+1);
					sscanf(CmdData+1+SpacePos[2],"%lf",BLH+2);
					SetFixPos(BLH);
				}
				else
					IllegalInstruction();
			}
			break;
    case SETAVE_C:
			{
				if(SysStatus.GPSMode==BaseS)
				{
					double time;
					sscanf(CmdData+1+SpacePos[0],"%lf",&time);
					SetAveTime(time);
				}
				else
					IllegalInstruction();
			}
			break;
    case SETRTK_C:
			{
				if(SysStatus.GPSMode==MobileS)
				{
					int source;
					source=atoi(CmdData+1+SpacePos[0]);
					if(source>RTKWDR)
						break;
					SetRTKSource(source);
				}
				else
					IllegalInstruction();
			}
			break;
    case SETSERVERIP_C:
				SetServerIP(CmdData+1+SpacePos[0]);
			break;
    case SETSERVERPORT_C:
			{
					int port;
					char * portC=CmdData+1+SpacePos[0];
					sscanf(portC,"%d",&port);
					SetServerPort(port);			
			}
			break;
    case SETMOBILEMODE_C:
			SetMobileMode();
			break;
    case SETBASEMODE_C:
			SetBaseMode();
			break;
		case RESTARTGPS_C:
			RestartGPS();
			break;
		case RESTART4G_C:
			Restart4G();
			break;
		case RESETCONFIG_C:
			ResetStatus();
			break;
		case LINKPORT_C:
			{
				int port=atoi(CmdData+1+SpacePos[0]);
				LinkPortC(port);
			}
			break;
		case READPORT_C:
			{
				int port=atoi(CmdData+1+SpacePos[0]);
				ReadPortC(port);
			}
			break;
		case RADIO_C:
			{
				uint8_t speed=atoi(CmdData+1+SpacePos[0]);
				uint8_t channel=atoi(CmdData+1+SpacePos[1]);
				if(SetRadioConfig(speed,channel)!=0)
				{
					userReplyInf("Set Radio Fail\r\n");	
				}
			}
			break;
		case PRINT_C:
			{
				uint8_t port=atoi(CmdData+1+SpacePos[0]);
				if(SetPrintPort(port)!=-1)
				{
					userReplyInf("Print to %d\r\n",port);
				}
				else
				{
					userReplyInf("Print Close\r\n");
				}
			}
			break;
		case GETINFO_C:
			{
				getDevInf();
			}
			break;
		case SETBTNAME_C:
			{
				setBlueToothName(CmdData+1+SpacePos[0]);
			}
			break;
		case INI4G_C:
			Ini4GCommand();
			break;



		case GETDUMP_C:
			{
				int len;
				char buf[IU_PRINT_TX_BUF_SIZE+1];
				do
				{
					len=get_dump(buf);
					buf[len]='\0';
					userReplyInf(buf);
					vTaskDelay(10);
				}while(len==TRACE_PRINT_TX_BUF_SIZE);
			}
			break;
		case GETVER_C:
			{
				userReplyInf("HARDWARE_VERSION:\r\n %s \r\nSOFTWARE_VERSION:\r\n %s\r\n",Hardware_Version,Software_Version);
			}
			break;	
		case RESTART_C:
			System_SoftReset();
			break;
		case GETFLAG_C:
			uartPrintConter();
			break;			
		case GENBUG_C:
		{
			double * f=(double*)0xFFFFFF;
			*f=100;			
		}
			break;
		case GENAST_C:
			configASSERT(0);
			break;
		case TEST4GSend_C:
			Test4GFlag=true;
			break;
		
		default:
			if(SysRamStatus.SwitchMode!=LinkS)
				IllegalInstruction();
			break;
	}	
	CmdDataLen=0;
}
CmdType_e InstructionNameToCode(char * Name)
{
	if(strcmp(Name,"setpos")==0)
		return SETPOS_C;	
	if(strcmp(Name,"setave")==0)
		return SETAVE_C;	
	if(strcmp(Name,"setrtk")==0)
		return SETRTK_C;	
	if(strcmp(Name,"setserverip")==0)
		return SETSERVERIP_C;
	if(strcmp(Name,"setserverport")==0)
		return SETSERVERPORT_C;
	if(strcmp(Name,"setmobilemode")==0)
		return SETMOBILEMODE_C;
	if(strcmp(Name,"setbasemode")==0)
		return SETBASEMODE_C;
	if(strcmp(Name,"restartgps")==0)
		return RESTARTGPS_C;
	if(strcmp(Name,"restart4g")==0)
		return RESTART4G_C;	
	if(strcmp(Name,"resetconfig")==0)
		return RESETCONFIG_C;	
	if(strcmp(Name,"linkport")==0)
		return LINKPORT_C;
	if(strcmp(Name,"readport")==0)
		return READPORT_C;	
	if(strcmp(Name,"setradio")==0)
		return RADIO_C;	
	if(strcmp(Name,"printport")==0)
		return PRINT_C;	
	if(strcmp(Name,"getinf")==0)
		return 		GETINFO_C;
	if(strcmp(Name,"setbtname")==0)
		return 		SETBTNAME_C;	
	if(strcmp(Name,"ini4g")==0)
		return 		INI4G_C;
	if(strcmp(Name,"test4gSend")==0)
		return 		TEST4GSend_C;
		if(strcmp(Name,"getdump")==0)
		return 		GETDUMP_C;
	if(strcmp(Name,"update")==0)	
		return 		UPDATE_C;
	if(strcmp(Name,"getver")==0)	
		return 		GETVER_C;
	if(strcmp(Name,"restart")==0)	
		return 		RESTART_C;	
	if(strcmp(Name,"getflag")==0)	
		return 		GETFLAG_C;
	if(strcmp(Name,"genbug")==0)	
		return 		GENBUG_C;
	if(strcmp(Name,"genast")==0)	
		return 		GENAST_C;	
	
	return FAILCOMMAND;
}
void getDevInf()
{
	userReplyInf("\r\n");
	userReplyInf("\r\n");
	userReplyInf("GPS Mode : %d \r\n",(int)SysStatus.GPSMode);
	userReplyInf("RTK Source : %d \r\n",(int)SysStatus.RTKSource);
	userReplyInf("fixPos : %d Latitude %f Longitude %f Altitude %f\r\n",SysStatus.FixPos,SysStatus.Latitude,SysStatus.Longitude,SysStatus.Altitude);
	userReplyInf("aveTime : %f\r\n",SysStatus.AveTime);
	userReplyInf("ServerPort : %d\r\n",SysStatus.ServerPort);
	userReplyInf("ServerIP : %s\r\n",SysStatus.ServerIP);
	userReplyInf("RadioSpeed : %d\r\n",SysStatus.RadioSpeed);
	userReplyInf("RadioChan : %d\r\n",SysStatus.RadioChan);
	userReplyInf("InterfacePort : %d\r\n",SysStatus.InterfacePort);

	userReplyInf("SwitchMode : %d\r\n",SysRamStatus.SwitchMode);
	userReplyInf("LinkPort : %d\r\n",SysRamStatus.LinkPort);
	userReplyInf("ReadPort : %d\r\n",SysRamStatus.ReadPort);
	userReplyInf("\r\n");
	userReplyInf("\r\n");


}
int userReplyInf(const char *format, ...)
{


	  va_list args;
    uint32_t length;
		if(SysStatus.InterfacePort<=Uart_BLE)
		{
			va_start(args, format);
			length = vsnprintf((char *)print_buf, IU_PRINT_TX_BUF_SIZE, (char *)format, args);
			va_end(args);
			UartSendData(&(ComPorts[SysStatus.InterfacePort]),(uint8_t*)print_buf, length);
		}
    return length;
}
int IllegalInstruction()
{
  char inf[]="Illegal instruction\r\n";
	userReplyInf(inf);
}

void uartPrintConter()
{
	for(int n =0;n<CALL_END;n++)
	{
		userReplyInf("callConter %d = %d \r\n",n,callConter[n]);
	}
}
__asm void SystemResetUI(void)
{
	MOV r0, #1           //;
	MSR FAULTMASK, r0    //; 清除FAULTMASK 禁止一切中断产生	
	LDR r0, =0xE000ED0C  //;
	LDR r1, =0x05FA0004  //;
	STR r1, [r0]         //; 系统软件复位   

deadloop
}
void System_SoftReset(void)
{
	vTaskDelay(500);
	SystemResetUI();
}