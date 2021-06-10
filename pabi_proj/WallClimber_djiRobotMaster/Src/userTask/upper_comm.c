#include "upper_comm.h"
#include <stdbool.h>
#include "protocol_type.h"
//#include "usb_type.h"
//#include "usbd_cdc_if.h"
#include "common.h"
#include "ringbuf.h"
#include "string.h"
#include "cmsis_armcc.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "pid.h"
#include "can_platform_c610Motor.h"
#include "IMUData.h"

#define USB_FB_TEST_MODE0   // USB_FB_TEST_MODE：反馈数据中加入测试数据


#define RXBUFFERSIZE   20 //缓存大小
#define MAXDIS 12000.0   //mm
uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
static int32_t usartError=0;
static HAL_UART_StateTypeDef usartSate=0;

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;

static void commReadTask(void *argument);
static void commSendTask(void *argument);
static void resolveRxBuf(void);

static void ParseFrame(uint8_t *Data, uint32_t DataLen);
static uint8_t CheckSum(uint8_t *Data, uint32_t DataLen);

static TaskHandle_t CommReadTaskHandle = NULL;
static TaskHandle_t CommSendTaskHandle = NULL;

static bool HeadFound, TailFound;
static uint32_t HeadPos, TailPos, CheckPos;

RxBuf_s RxBuffer;
//extern RxBuf_s tempRxBuff;
MoveCmd_s           MoveCmd;
PidParam_s       	PidParamCmd;
ParamConfig_s		ParamConfigs;
Flag_s 				sFlag;

RobotStateFrame_s   RobotStateFrame;
StateFeed_promptly_s   StateFeed_promptlys;


SemaphoreHandle_t BinarySem_Rx = NULL;
QueueHandle_t      QueueHandle = NULL;
extern bool g_shutdownFlag;

static RobotStateFrame_s   		LocalRobotStateFrame;
static StateFeed_promptly_s	   	LocalStateFeed_promptlys;
//int16_t errorCount_local=0;
//static uint16_t DMA_SendData(uint8_t *buf,uint16_t len);
//static uint16_t DMA_ReadData(uint8_t *buf,uint16_t len);
static uint16_t USART_SendData(uint8_t *buf,uint16_t len);
static uint16_t USART_ReadData(uint8_t *buf,uint16_t len);
static uint16_t USB_SendData(uint8_t *buf,uint16_t len);
static uint16_t readData(uint8_t *buf,uint16_t len); //从环形缓冲区读取
//static Queue_Message_s queueMessage;
static void getStateFeed_promptlys();
/**
 * @description:  上位机通讯读写任务初始化
 * @param {type} 
 * @return: 
 */
void initCommunication()
{
	memset(&sFlag, 0, sizeof(sFlag));
    memset(&RxBuffer, 0, sizeof(RxBuffer));
	//memset(&tempRxBuff, 0, sizeof(tempRxBuff));
    memset(&MoveCmd, 0, sizeof(MoveCmd));
    memset(&PidParamCmd, 0, sizeof(PidParamCmd));
	memset(&ParamConfigs, 0, sizeof(ParamConfigs));
	
	memset(&StateFeed_promptlys, 0, sizeof(StateFeed_promptlys));
	memset(&RobotStateFrame, 0, sizeof(RobotStateFrame));


	
    xTaskCreate(commReadTask,        /* 任务函数  */
                "CommReadTask",        /* 任务名    */
                256,                /* 任务栈大小，单位word，也就是4字节 */
                NULL,               /* 任务参数  */
                14,                  /* 任务优先级*/
                &CommReadTaskHandle); /* 任务句柄  */
    xTaskCreate(commSendTask,        /* 任务函数  */
                "CommSendTask",       /* 任务名    */
                256,                /* 任务栈大小，单位word，也就是4字节 */
                NULL,               /* 任务参数  */
                6,                  /* 任务优先级*/
                &CommSendTaskHandle); /* 任务句柄  */
}

static void commReadTask(void *argument)
{
	//printf("===========commReadTask================start");
	bool firstRecFlag = false;
	CheckPos=0;
	int16_t count_USB=0; 
	//BinarySem_Rx二值信号量，用于串口数据接收与解析同步
	BinarySem_Rx = xSemaphoreCreateBinary();
	if(BinarySem_Rx==NULL){
		printf("create BinarySem_Rx error!\r\n");
	}
	//开启中断接收,发送用串口，接收用usb
	HAL_UART_Receive_IT(&huart6,(uint8_t*)aRxBuffer,RXBUFFERSIZE);
	//DMA方式
	
    while (1)
    {
		//osDelay(50);
		if(BinarySem_Rx==NULL){
			BinarySem_Rx = xSemaphoreCreateBinary();
			osDelay(50);
			continue;
		}
		else{
			if(xSemaphoreTake(BinarySem_Rx,150*portTICK_RATE_MS)==pdFAIL){
				count_USB++;
				if(count_USB%100==0){
					sFlag.disconnetFlag=true;
					count_USB=0;
					printf(" ReadTask serial read error ,disconnect\r\n");
				}
				//TODO关机信号
				if(count_USB%200==0 && firstRecFlag==true && g_shutdownFlag==false){
					__set_FAULTMASK(1);// 关闭所有中断
					NVIC_SystemReset(); 
					count_USB=0;	
				}
				continue;
			}
		}
		count_USB=0;
		sFlag.disconnetFlag=false;
		firstRecFlag = true;

		if(RxBuffer.RxLen>=RX_BUF_LEN){
			RxBuffer.RxLen = 0;
		}
		uint16_t RcvedLen = readData(RxBuffer.RxBuf + RxBuffer.RxLen, RxIdleLen(RxBuffer));
		//printf("RxBuffer.RxLen==%d  =============\r\n",RxBuffer.RxLen);
		//printf("readLen==%d  disconnetFlag:%d=============\r\n",RcvedLen,sFlag.disconnetFlag);
		//////*************printf**********/////
//		printf("RxBuff:\r\n");
//		for(int i=0;i<RxBuffer.RxLen;i++){
//			printf(" %x, ",RxBuffer.RxBuf[i]);
//		}
//		printf("\r\n");
		//////*************printf**********/////
		/////TODO---

		if (RcvedLen<=0){
			continue;
		}
		else{
			RxBuffer.RxLen += RcvedLen;
			resolveRxBuf();
			count_USB=0;
		}
//		//重置433电台
//		if(getbit(MoveCmd.digit,14)){
//			//printf("resetRedio=%d, digit=%d \r\n",getbit(MoveCmd.digit,14),MoveCmd.digit);
//			//ControlDigit->resetRedio =true;
//		}
//		else{
//			//ControlDigit->resetRedio=false;
//		}
		

		
    }
}


static void resolveRxBuf(void)
{
	CheckPos= 0;
	//idex + 结尾两个字节（即检测到末尾两个字节之前）
    while (CheckPos + 2 <= RxBuffer.RxLen)
    {
        if (HeadFound==1)
        {
            if (CheckMask(&RxBuffer.RxBuf[CheckPos], PROTOCOL_TAIL))
            {
				//printf("04==tail  found\r\n");
                TailFound = true;
                TailPos = CheckPos;
            }
        }
        else
        {
            if (CheckMask(&RxBuffer.RxBuf[CheckPos], PROTOCOL_HEAD))
            {
				//printf("03==head  found\r\n");
                HeadFound = true;
                HeadPos = CheckPos;
            }
        }
        CheckPos++;

        if ((HeadFound && TailFound)==1)
        {
			//////*************printf**********/////
			//printf("head and tail found:\r\n");
//		for(int i=0;i<TailPos-HeadPos+2;i++){
//			printf(" %x, ",RxBuffer.RxBuf[HeadPos+i]);
//		}
//		printf("\r\n");
//////*************printf**********/////
			ParseFrame(RxBuffer.RxBuf + HeadPos + 2, TailPos - HeadPos - 2);
			HeadFound = TailFound = false;
            RxBuffer.RxLen = RxBuffer.RxLen - (TailPos + 2); //tailPos+2 缓存区最后/
			//TODO        if(RxBuffer.RxLen>0)
			if(RxBuffer.RxLen>0){
				memmove(RxBuffer.RxBuf, RxBuffer.RxBuf + TailPos + 2, RxBuffer.RxLen);
				CheckPos = 0;
				//printf("02==ParseFrame before\r\n");
			}
        }
    }
}

static void commSendTask(void *argument)
{
	//taskENTER_CRITICAL();

    InitProtocolFrame(RobotStateFrame);
	InitProtocolFrame(StateFeed_promptlys);

	RobotStateFrame.funcCode = Func_StateFrame;
	StateFeed_promptlys.funcCode = Func_StateFeed;
	//taskEXIT_CRITICAL();

//    memcpy(&LocalRobotPostureFrame, &RobotPostureFrame, sizeof(RobotPostureFrame));
    memcpy(&LocalRobotStateFrame, &RobotStateFrame, sizeof(RobotStateFrame));
	memcpy(&LocalStateFeed_promptlys, &StateFeed_promptlys, sizeof(StateFeed_promptlys));
	
	int tempCount=0;

    while (1)
    {
		
		int16_t send_len=0;
        osDelay(200);   
        tempCount++;
/*测试代码 ==============Begin====================*/
#if defined USB_FB_TEST_MODE
        RobotStateFrame.digit++;
        RobotStateFrame.elecInfo.batVoltage = 1.0;
        RobotStateFrame.posture.picth = 1.0;
        RobotStateFrame.tempInfo.temp[0] = 1.0;
        RobotPostureFrame.posture.posX = 1.0;
#endif
/*测试代码 ==============End=====================*/
        

        memcpy(&LocalRobotStateFrame, &RobotStateFrame, sizeof(RobotStateFrame));
		//memcpy(&LocalStateFeed_promptlys, &StateFeed_promptlys, sizeof(StateFeed_promptlys));
		getStateFeed_promptlys();
        LocalRobotStateFrame.checkSum = CheckSum((uint8_t *)&(LocalRobotStateFrame.funcCode), BodyLen(LocalRobotStateFrame));
        //LocalRobotPostureFrame.checkSum = CheckSum((uint8_t *)&(LocalRobotPostureFrame.funcCode), BodyLen(LocalRobotPostureFrame));
		LocalStateFeed_promptlys.checkSum = CheckSum((uint8_t *)&(LocalStateFeed_promptlys.funcCode), BodyLen(LocalStateFeed_promptlys));
//		int ssum = CheckSum((uint8_t *)&(LocalStateFeed_promptlys.funcCode), BodyLen(LocalStateFeed_promptlys));
//		printf("motorCurrent[0]:%3.2f motorCurrent[1]:%3.2f \r\n",
//				RobotStateFrame.sMotorState.motorCurrent[0],RobotStateFrame.sMotorState.motorCurrent[1]);
		if(tempCount>=5){
			send_len=USART_SendData((uint8_t *)&LocalRobotStateFrame, sizeof(LocalRobotStateFrame));
			osDelay(100);
			tempCount=0;
		}
		//USB send
		send_len=USART_SendData((uint8_t *)&LocalStateFeed_promptlys, sizeof(LocalStateFeed_promptlys));
		if(send_len){
			resetbit(StateFeed_promptlys.digit,10);
			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14,GPIO_PIN_RESET); //复位
		}
		//printf("send StateFeed_promptlys.digit = %d\r\n",StateFeed_promptlys.digit);
//		printf("StateFeed_promptlys send  ok==== sendlen: %d odom:   %3.2f  time %3.2f checksum:%x\r\n",
//			send_len,LocalStateFeed_promptlys.sPicOdom.odom,LocalStateFeed_promptlys.sPicOdom.time,
//			LocalStateFeed_promptlys.checkSum
//		);
		//TODO==================
    }
}

static void ParseFrame(uint8_t *Data, uint32_t DataLen)
{
    uint8_t sum = CheckSum(Data, DataLen - 1);
	
    if (Data[DataLen - 1] == sum)   //Data[DataLen - 1]最后字节为校验和，计算该字节之前的sum
    {
        uint8_t *FrameData = Data + 1; //去掉功能码
		//printf("-----------------------sucesss Data[0]==============%x\r\n",Data[0]);
		static int readcount=0;
        switch (Data[0])
        {
        case Func_MoveCmd:
//			taskENTER_CRITICAL();
            MoveCmd = *((MoveCmd_s *)(FrameData));
		//printf("moveCmd,speed[%3.2f],omega[%3.2f]\r\n",MoveCmd.speed, MoveCmd.omega);
//			taskEXIT_CRITICAL();
            break;
        case FUnc_ParamConfig:
			ParamConfigs = *(ParamConfig_s *)(FrameData);
            break;
//        case Func_LidarInfo:
//            LidarInfo = *(LidarInfo_s *)(FrameData);
//            break;
        case Func_PidParamCmd:

			PidParamCmd = *(PidParam_s *)(FrameData);

//			printf("======Func_PidParamCmd %x  Kp=%3.2f Ki=%3.2f Kd=%3.2f  IGate=%3.2f OutGate=%3.2f \r\n",
//						Func_PidParamCmd,PidParamCmd.Kp,PidParamCmd.Ki, PidParamCmd.Kd,PidParamCmd.IGate,PidParamCmd.OutGate);
            break;
        
        default:
            //log_error("Unknow func code: %x\r\n", Data[0]);
            break;
        }
		//xQueueSend(QueueHandle,&queueMessage,0);
    }
    else
    {
//////*************printf**********/////
//		printf("error:");
//		for(int i=0;i<DataLen;i++){
//			printf(" %x, ",RxBuffer.RxBuf[HeadPos+i]);
//		}
//		printf("\r\n");
//////*************printf**********/////
        //log_error("++Check sum not match, expect:%x, actual:%x\r\n", Data[DataLen - 1], sum);
		return;
		
	}
}


/**
 * @description: sum from index [0, DataLen-1]
 * @param {type} 
 * @return: 
 */
static uint8_t CheckSum(uint8_t *Data, uint32_t DataLen)
{
    uint8_t sum = 0;
	//temp TODO
	if(DataLen>=RX_BUF_LEN){
		return 0;
	}
    for (int i = 0; i < DataLen; i++)
    {
        sum += Data[i];
    }
    return sum;
}

//static uint16_t DMA_SendData(uint8_t *buf,uint16_t len)
//{
//	static uint16_t counter=0;
//	taskENTER_CRITICAL();
//	if(HAL_UART_Transmit_DMA(&huart3, buf,len)!= HAL_OK){
//		usartError = huart3.ErrorCode;
//		usartSate= huart3.gState;
//		
//            //Error_Handler();
//		
//		counter++;
//		return 0;
//	}
//	taskEXIT_CRITICAL();
//	//TODO 判断传输完成
////	while(1){
////		if(__HAL_DMA_GET_FLAG(&huart3,DMA_FLAG_TCIF3_7)){
////			__HAL_DMA_CLEAR_FLAG(&huart3,DMA_FLAG_TCIF3_7);
////			huart3.gState =HAL_UART_STATE_READY;
////			break;
////		}
////	}
//	return len;

//}

//static uint16_t DMA_ReadData(uint8_t *Buf,uint16_t IdleLen)
//{
//	//重新打开DMA接收
//	//HAL_UART_DMAStop(&huart3); //
//	uint16_t resultLen;
//	if (Buf == NULL){
//        return 0;
//	}
//	/*****使用环形缓冲区*****/
//	//resultLen=Bluetooth_Read(Buf,IdleLen,0);
//	
//	/*****不使用环形缓冲区*****/
//	//resultLen= bufferRead(Buf,IdleLen);

//	return resultLen;	

//}
static uint16_t USART_SendData(uint8_t *buf,uint16_t len)
{
	
	if(HAL_UART_Transmit(&huart6, buf,len,0xFF)!= HAL_OK){
        //Error_Handler();
		return 0;
	}
	return len;
	

}
//static uint16_t USART_ReadData(uint8_t *Buf,uint16_t IdleLen)
//{
//	uint16_t resultLen;
//	if (Buf == NULL){
//        return 0;
//	}
//	/*****使用环形缓冲区*****/
//	resultLen=Bluetooth_Read(Buf,IdleLen,0);
//	/*****不使用环形缓冲区*****/
//	//resultLen= bufferRead(Buf,IdleLen);
//	return resultLen;	

//}




static uint16_t USB_SendData(uint8_t *buf,uint16_t len)
{
	uint8_t result = USBD_OK;
	
	if(CDC_Transmit_FS( buf,len)!= USBD_OK){
		return 0;
	}
	return len;
	
}
static uint16_t readData(uint8_t *Buf,uint16_t IdleLen)
{
	uint16_t resultLen;
	if (Buf == NULL){
        return 0;
	}
	/*****使用环形缓冲区*****/
	resultLen=Bluetooth_Read(Buf,IdleLen,0);
	return resultLen;	

}
static void getStateFeed_promptlys()
{
	static float odomV,odomW;
	//getRobotV(&odomV,&odomW);
	LocalStateFeed_promptlys.digit = StateFeed_promptlys.digit;
	LocalStateFeed_promptlys.odom.wheelVelocity[0]= getWheelVelocity(0);
	LocalStateFeed_promptlys.odom.wheelVelocity[1]= getWheelVelocity(1);
	LocalStateFeed_promptlys.odom.theta= getTheta();
	
//	LocalStateFeed_promptlys.odom.wheelVelocity[0]= 0.2;
//	LocalStateFeed_promptlys.odom.wheelVelocity[1]= 0.2;
//	LocalStateFeed_promptlys.odom.theta= 0.1;
	
//	LocalStateFeed_promptlys.digit = 5;
//	LocalStateFeed_promptlys.odom.vx= 6.4;
//	LocalStateFeed_promptlys.odom.vy= 4.0;
//	LocalStateFeed_promptlys.odom.theta = 32.0;
	printf("odom: vL[%3.2f],vR[%3.2f], theta=[%3.2f]\r\n",
		LocalStateFeed_promptlys.odom.wheelVelocity[1],LocalStateFeed_promptlys.odom.wheelVelocity[1],
		LocalStateFeed_promptlys.odom.theta);
	
}

//TODO 开启IDELE中断--回调中，clear ，disnabel DMA