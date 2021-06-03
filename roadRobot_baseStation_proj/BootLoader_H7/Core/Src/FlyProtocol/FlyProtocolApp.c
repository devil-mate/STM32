#include "FlyProtocolApp.h"
#include "FlyProtocol.h"
#define FIRMWARE_LEN (64)
uint64_t flyProtocolFirmware;
uint8_t flyProtocolFirmwareNameLen;
char flyProtocolFirmwareName[FIRMWARE_LEN];
uint8_t firmwareLength;
uint64_t flyProtocolID;
OnRecvFrameFunc onUpgradeFrameFunc;
GetSystemTimeFunc mGetSystemTimeFunc;
OnRecvFrameFunc mHeartbeatRecvFunc;
unsigned char OnFlyRecvFrameFunc(ProtocolHead *head, unsigned char *data, uint16_t dataLength);
void iniFlyApp()
{
	iniFlyProtocol();
	firmwareLength = 0;
	onUpgradeFrameFunc = NULL;
	mGetSystemTimeFunc = NULL;
    mHeartbeatRecvFunc=NULL;
	setFlyProtocolRecvFunc(OnFlyRecvFrameFunc);
}
unsigned char getID(ProtocolHead *head)
{
	uint8_t buf[sizeof(uint64_t) * 2+FIRMWARE_LEN];
	if (head == NULL)
		return 0;
	uint64_t *id = (uint64_t *)buf;
	uint64_t *firmware = (uint64_t *)(buf + sizeof(uint64_t));
	*id=flyProtocolID;
	*firmware = flyProtocolFirmware;
	head->dev_id=flyProtocolID;
	strcpy(buf+sizeof(uint64_t) * 2,flyProtocolFirmwareName);
	return sendFlyProtocolRelpy(CLOUD_CMD_RELAY_HEAD, head->dev_id, head->frame_id, FLY_ID, buf, sizeof(uint64_t) * 2+flyProtocolFirmwareNameLen);
}
void SetFirmware(uint64_t firmwareID,char * firmwareName)
{
	if(firmwareName==NULL)
	{
		flyProtocolFirmwareName[0]='\0';
	}
	else
	{
		int nameLen=strlen(firmwareName);
		if(nameLen>=FIRMWARE_LEN-1)
		{
			memcpy(flyProtocolFirmwareName,firmwareName,FIRMWARE_LEN-1);
			flyProtocolFirmwareName[FIRMWARE_LEN-1]='\0';
			flyProtocolFirmwareNameLen=FIRMWARE_LEN-1;
		}
		else{
			strcpy(flyProtocolFirmwareName,firmwareName);
			flyProtocolFirmwareNameLen=nameLen;
		}
	}
	flyProtocolFirmware=firmwareID;
}
void SetFlyProtocolID(uint64_t id)
{
	flyProtocolID = id;
}
void SetGetSystemTimeFunc(GetSystemTimeFunc func)
{
	mGetSystemTimeFunc= func;
}
void SetHeartbeatFrameFunc(OnRecvFrameFunc func)
{
    mHeartbeatRecvFunc = func;
}
void SetUpgradeFrameFunc(OnRecvFrameFunc func)
{
	onUpgradeFrameFunc=func;
}
unsigned char  notSupportFrame(ProtocolHead *head)
{
	if (head == NULL)
		return 0;
	USBPrintf("[BOOT] notSupportFrame %08X",head->code);
	return sendFlyProtocolRelpy(CLOUD_CMD_RELAY_HEAD, head->dev_id, head->frame_id, NOT_SUPPORT_MASK | head->code, NULL,0);
}
void FlyHeartbeat()
{
	ProtocolHeartbeat mProtocolHeartbeat;
	if(mGetSystemTimeFunc!=NULL)
		mProtocolHeartbeat.time=mGetSystemTimeFunc();
	sendFlyProtocolFunctionNoRelpy(CLOUD_CMD_RELAY_HEAD,flyProtocolID,FLY_HEARTBEAT,(unsigned char *)&mProtocolHeartbeat,sizeof(mProtocolHeartbeat));
}
unsigned char OnFlyRecvFrameFunc(ProtocolHead *head, unsigned char *data, uint16_t dataLength)
{
	switch (head->code)
	{
	case FLY_ID:
		getID(head);
		break;
	case FLY_FIRMWARE:
		if(onUpgradeFrameFunc!=NULL)
			onUpgradeFrameFunc(head,data,dataLength);
		else
			notSupportFrame(head);
		break;
	case FLY_HEARTBEAT:
        if(mHeartbeatRecvFunc!=NULL)
            mHeartbeatRecvFunc(head,data,dataLength);
		break;
	//case FLY_PARAMETER_SIZE:
	//	break;
	//case FLY_PARAMETER_DES:
	//	break;
	//case FLY_PARAMETER_READ:
	//	break;
	//case FLY_PARAMETER_SET:
	//	break;
	//case FLY_PARAMETER_SAVE:
	//	break;
	//case FLY_PARAMETER_LOAD:
	//	break;
	//case FLY_VARIABLE_READ:
	//	break;
	//case FLY_VARIABLE_ENABLE:
	//	break;
	//case FLY_VARIABLE_DISENABLE:
	//	break;
	//case FLY_STREAM_OPEN:
	//	break;
	//case FLY_STREAM_CLOSE:
	//	break;
	//case FLY_WORK_START:
	//	break;
	//case FLY_WORK_STOP:
	//	break;
	//case FLY_BOOT_REPLY:
	//	break;
	//case FLY_CLEAN_ERR:
	//	break;
	default:
		notSupportFrame(head);
		break;
	}
}
