#include "FlyProtocol.h"
#include "FrameCheck.h"
#include <cstdlib>
SendStreamFunc mSendFunc;
OnRecvFrameFunc mOnRecvFrameFunc;
OnErrFrameFunc mOnErrFrameFunc;
unsigned char onRecvFrame(ProtocolHead * head , unsigned char * data );//
uint8_t recvBuf[RECV_BUF_SIZE];
uint8_t sendBuf[SEND_BUF_SIZE];

uint8_t sendBufWait[RELPY_WAIT_SIZE][SEND_BUF_SIZE];
uint16_t sendWaitTimer[RELPY_WAIT_SIZE];
uint16_t sendWaitRepeat[RELPY_WAIT_SIZE];
uint8_t  sendWaitIndex;

uint16_t flyFrameHeadPos, flyRecvBufTail;
uint8_t recvBufFull, recvSelect;
uint64_t  mProtocolFrameId;
void parsingFrame();

unsigned int pushToBuf(unsigned char *buf, unsigned int len);
void iniFlyProtocol()
{
	mSendFunc = NULL;
	mOnRecvFrameFunc= NULL;
	mOnErrFrameFunc= NULL;
	flyFrameHeadPos = flyRecvBufTail = 0;
	recvBufFull = 0;
	recvSelect = 0;
	mProtocolFrameId=0;
	memset(sendWaitTimer,0,sizeof(uint16_t)*RELPY_WAIT_SIZE);
	sendWaitIndex=0;
}
void flyProtocolTick()
{
	for(int n =0;n<RELPY_WAIT_SIZE;n++)
	{
		if(sendWaitTimer[n]>0)
		{
			sendWaitTimer[n]--;
			
			if(sendWaitTimer[n]==0)
			{
				ProtocolHead * head = (ProtocolHead * ) sendBufWait[n];
				sendWaitRepeat[n]--;
				if(sendWaitRepeat[n]==0)
				{
					if(mOnErrFrameFunc!=NULL)
					{
						mOnErrFrameFunc(head,FLY_SEND_FAIL);
					}
				}
				else
				{
					sendWaitTimer[n]=RELPY_TIME_LIMIT;
					if(mSendFunc!=NULL)
					{
						mSendFunc(sendBufWait[n],head->frameLen);
					}
				}
			}
		}
	}
}
void setFlyProtocolErrFunc(OnErrFrameFunc setErrFunc)
{
	mOnErrFrameFunc=setErrFunc;
}
void setFlyProtocolSendFunc(SendStreamFunc setSendFunc)
{
	mSendFunc = setSendFunc;
}
void setFlyProtocolRecvFunc(OnRecvFrameFunc setRecvFunc)
{
	mOnRecvFrameFunc=setRecvFunc;
}
void onFlyProtocolDataRecved(unsigned char *buf, unsigned int len)
{
	pushToBuf(buf, len);
	parsingFrame();
}
unsigned int pushToBuf(unsigned char *buf, unsigned int len)
{
	uint16_t freeSize;
	freeSize= RECV_BUF_SIZE - flyRecvBufTail;
	
	if (freeSize >= len)
	{
		memcpy(recvBuf + flyRecvBufTail, buf, len);
		flyRecvBufTail = flyRecvBufTail + len;
		return len;
	}
	else
	{
		flyRecvBufTail = 0;
		return 0;
	}
}
void parsingFrame()
{
    int16_t remainData=(int16_t)flyRecvBufTail - sizeof(ProtocolHead);
	
	for (flyFrameHeadPos = 0; (int16_t)flyFrameHeadPos < remainData; flyFrameHeadPos++)
	{
		ProtocolHead *frameHead = (ProtocolHead *)(recvBuf + flyFrameHeadPos);
		if (((frameHead->head) == CLOUD_CMD_HEAD) || ((frameHead->head) == DEV_CMD_HEAD)||
		((frameHead->head) == CLOUD_CMD_RELAY_HEAD) || ((frameHead->head) == DEV_RELAY_HEAD))
		{
			if ((frameHead->frameLen + flyFrameHeadPos) <= flyRecvBufTail)
			{
				unsigned char cs;
				cs=checkCalc((uint8_t *)frameHead, frameHead->frameLen);
				if (cs  == 0)
				{
					for(int n =0 ;n<RELPY_WAIT_SIZE;n++)
					{
						if(sendWaitTimer[n]!=0)
						{
							ProtocolHead *waitHead = (ProtocolHead *)sendBufWait[n];
							if(waitHead->frame_id==frameHead->frame_id)
							{
								sendWaitTimer[n]=0;
							}
						}
					}

					if(mOnRecvFrameFunc!=NULL)
						mOnRecvFrameFunc(frameHead,((uint8_t *)frameHead)+sizeof(ProtocolHead),frameHead->frameLen-sizeof(ProtocolHead)-sizeof(uint32_t));
					remainData=flyRecvBufTail-(flyFrameHeadPos+frameHead->frameLen);
					//assert(remainData>=0);
					memmove(recvBuf,recvBuf+flyFrameHeadPos+frameHead->frameLen,remainData);
					flyRecvBufTail=remainData;
					flyFrameHeadPos=0xFFFF;//下一个循环开始加一即为0
				}
				else
				{
					//USBPrintf("[BOOT]Check Sum Error sum %02X len %d",cs,frameHead->frameLen);
				}
			}
		}
	}
}
unsigned char sendFlyProtocolFrame(ProtocolHead * head , unsigned char * data,uint16_t dataLength,uint8_t needRelay)
{
	ProtocolHead * sendHead;
	uint32_t * check;
	if(head==NULL)
		return 1;
	if((data==NULL)&&(dataLength>0))
		return 2;
	
	if(needRelay!=0)
	{
		uint32_t * randC; 
		mProtocolFrameId++;
		randC=(uint32_t *)&(head->frame_id);
		*randC=rand();
		*(randC+1)=rand();
	}
	if(dataLength+sizeof(ProtocolHead)+sizeof(uint32_t)>SEND_BUF_SIZE)
		return 3;
	
	
	memcpy(sendBuf,head,sizeof(ProtocolHead));
	if(dataLength>0)
		memcpy(sendBuf+sizeof(ProtocolHead),data,dataLength);
	sendHead=(ProtocolHead *)sendBuf;
	check=(uint32_t * )(sendBuf+sizeof(ProtocolHead)+dataLength);
	sendHead->ver=0;
	sendHead->frameLen=sizeof(ProtocolHead)+dataLength+sizeof(uint32_t);
	*check=0-(checkCalc(sendBuf,sizeof(ProtocolHead)+dataLength));
    *check=*check&0x000000FF;
	int index= sendWaitIndex;
	if(needRelay!=0)
	{
		do
		{
			if(sendWaitTimer[index]==0)
			{
				memcpy(sendBufWait[index],sendBuf,sendHead->frameLen);
				sendWaitTimer[index]=RELPY_TIME_LIMIT;
				sendWaitRepeat[index]=RELPY_REPEAT_LIMIT;
				sendWaitIndex=(index+1)%RELPY_WAIT_SIZE;
			}
			index=(index+1)%RELPY_WAIT_SIZE;
		}while(index!=sendWaitIndex);		
	}

	
	if(mSendFunc!=NULL)
	{
		return 	mSendFunc(sendBuf,sendHead->frameLen);
	}
	return 3;
}
unsigned char sendFlyProtocolFunction(uint16_t head , uint64_t dev_id,uint32_t code, unsigned char * data,uint16_t dataLength)
{
	ProtocolHead frameHead;
	frameHead.head=head;
	frameHead.dev_id=dev_id;
	frameHead.code=code;
	return sendFlyProtocolFrame(&frameHead,data,dataLength,1);	
}
unsigned char sendFlyProtocolFunctionNoRelpy(uint16_t head , uint64_t dev_id,uint32_t code, unsigned char * data,uint16_t dataLength)
{
	ProtocolHead frameHead;
	frameHead.head=head;
	frameHead.dev_id=dev_id;
	frameHead.code=code;
	return sendFlyProtocolFrame(&frameHead,data,dataLength,0);	
}
unsigned char sendFlyProtocolRelpy(uint16_t head , uint64_t dev_id,uint64_t frame_id, uint32_t code,unsigned char * data,uint16_t dataLength)
{
	ProtocolHead frameHead;
	frameHead.head=head;
	frameHead.dev_id=dev_id;
	frameHead.code=code;
	frameHead.frame_id=frame_id;
	return sendFlyProtocolFrame(&frameHead,data,dataLength,0);	
}

