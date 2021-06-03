#ifndef __PROTOCOLAPP_H__
#define __PROTOCOLAPP_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "FlyProtocol.h"
enum FlyProtocolCode
{
      FLY_ID=0x00000000,
      FLY_FIRMWARE=0x00000001,
      FLY_PARAMETER_SIZE=0x00000010,
      FLY_PARAMETER_DES=0x00000011,
      FLY_PARAMETER_READ=0x00000012,
      FLY_PARAMETER_SET=0x00000013,
      FLY_PARAMETER_SAVE=0x00000014,
      FLY_PARAMETER_LOAD=0x00000015,
      FLY_VARIABLE_READ=0x00000020,
      FLY_VARIABLE_ENABLE=0x00000021,
      FLY_VARIABLE_DISENABLE=0x00000022,
      FLY_STREAM_OPEN=0x00000023,
      FLY_STREAM_CLOSE=0x00000024,
      FLY_WORK_START=0x00000030,
      FLY_WORK_STOP=0x00000031,
      FLY_BOOT_REPLY=0x00000035,
      FLY_CLEAN_ERR=0x00000036 ,
      FLY_HEARTBEAT=0x000000FC
};

#define NOT_SUPPORT_MASK (0x80000000)
//typedef unsigned char (* OnUpgradeFrameFunc)(ProtocolHead * head , unsigned char * data,uint16_t dataLength);
//typedef unsigned char (* OnFrameRecvFunc)(ProtocolHead * head , unsigned char * data,uint16_t dataLength);
typedef unsigned long long (* GetSystemTimeFunc)();

void iniFlyApp();
void SetFlyProtocolID(uint64_t id);
void SetFirmware(uint64_t firmwareID,char * firmwareName);
void SetUpgradeFrameFunc(OnRecvFrameFunc func);
void SetHeartbeatFrameFunc(OnRecvFrameFunc func);
void SetGetSystemTimeFunc(GetSystemTimeFunc func);
void FlyHeartbeat();
extern uint64_t flyProtocolID;

#ifdef __cplusplus
}
#endif 
#endif