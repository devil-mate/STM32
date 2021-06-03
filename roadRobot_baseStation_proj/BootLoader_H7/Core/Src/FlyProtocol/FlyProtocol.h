#ifndef __FLYPROTOCOL_H__
#define __FLYPROTOCOL_H__
#ifdef __cplusplus
extern "C" {
#endif 
#include <stdio.h>
#include "ProtocolConfig.h"
#include "ProtocolDefine.h"

typedef unsigned char (* SendStreamFunc)(unsigned char * buf ,unsigned int len);//返回0为正常，其余错误错误码后补充
typedef unsigned char (* OnRecvFrameFunc)(ProtocolHead * head , unsigned char * data, uint16_t dataLength);
typedef unsigned char (* OnErrFrameFunc)(ProtocolHead * head , uint16_t errCode);

void iniFlyProtocol();
void setFlyProtocolSendFunc(SendStreamFunc setSendFunc);
void setFlyProtocolRecvFunc(OnRecvFrameFunc setRecvFunc);
void setFlyProtocolErrFunc(OnErrFrameFunc setErrFunc);
void onFlyProtocolDataRecved(unsigned char * buf ,unsigned int len);
unsigned char sendFlyProtocolFrame(ProtocolHead * head , unsigned char * data,uint16_t dataLength,uint8_t needRelpy);
unsigned char sendFlyProtocolFunction(uint16_t head , uint64_t dev_id,uint32_t code, unsigned char * data,uint16_t dataLength);
unsigned char sendFlyProtocolFunctionNoRelpy(uint16_t head , uint64_t dev_id,uint32_t code, unsigned char * data,uint16_t dataLength);
unsigned char sendFlyProtocolRelpy(uint16_t head , uint64_t dev_id,uint64_t frame_id, uint32_t code,unsigned char * data,uint16_t dataLength);
void flyProtocolTick();//建议至少每100ms调用一次


#ifdef __cplusplus
}
#endif 
#endif