#include "communication_down.h"
#include <stdbool.h>
#include "protocol_type.h"
#include "usb_type.h"
//#include "usbd_cdc_if.h"
#include "common.h"
#include "bluetooth_rutrans.h"
#include "string.h"
#define USB_FB_TEST_MODE0   // USB_FB_TEST_MODE�����������м����������




//up --huart3 ����λ��ͨ��; down --huart5 ���ͻ�����ͨ��

extern UART_HandleTypeDef huart5;
static void commDownReadTask(void *argument);
static void commDownSendTask(void *argument);
static TaskHandle_t CommUpReadTaskHandle = NULL;
static TaskHandle_t CommUpSendTaskHandle = NULL;

static bool HeadFound, TailFound;
static int HeadPos, TailPos, CheckPos;

static void resolveRxBuf(void);
static void ParseFrame(uint8_t *Data, uint32_t DataLen);


StateBackToBox_s   stateBackToBoxs;
RxBuf_s RxBuffer;
Flag_s 				sFlag;
//����
ControlCmd_s controlCmds;

extern SemaphoreHandle_t BinarySem_Rx;
//SemaphoreHandle_t BinarySem_Rx = NULL;
//QueueHandle_t      QueueHandle = NULL;


//static Queue_Message_s queueMessage;

/**
 * @description:  ��λ��ͨѶ��д�����ʼ��
 * @param {type} 
 * @return: 
 */
void initCommunication_down()
{
	memset(&sFlag, 0, sizeof(sFlag));
    memset(&RxBuffer, 0, sizeof(RxBuffer));
    //memset(&RobotPostureFrame, 0, sizeof(RobotPostureFrame));
	memset(&controlCmds,0,sizeof(controlCmds));
    memset(&stateBackToBoxs, 0, sizeof(stateBackToBoxs));
	

	
    xTaskCreate(commDownReadTask,        /* ������  */
                "CommReadTask",        /* ������    */
                1024,                /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,               /* �������  */
                14,                  /* �������ȼ�*/
                &CommUpReadTaskHandle); /* ������  */
    xTaskCreate(commDownSendTask,        /* ������  */
                "CommSendTask",       /* ������    */
                1024,                /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,               /* �������  */
                6,                  /* �������ȼ�*/
                &CommUpSendTaskHandle); /* ������  */
}

static void commDownReadTask(void *argument)
{
	//printf("===========commReadTask================start");
	static int8_t count_USB=0; 
	CheckPos=0;
	//BinarySem_Rx��ֵ�ź��������ڴ������ݽ��������ͬ��
	BinarySem_Rx = xSemaphoreCreateBinary();
	//����linux���յ��Ŀ����������
	//QueueHandle=xQueueCreate(4,sizeof(queueMessage));
	
	//�����жϽ���,�����ô��ڣ�������usb
	HAL_UART_Receive_IT(&huart5,(uint8_t*)aRxBuffer,RXBUFFERSIZE);
	//DMA��ʽ
//	HAL_StatusTypeDef isOK;		
//	HAL_UART_Receive_DMA(&huart3,aRxBuffer,RXBUFFERSIZE);
	//TODO-----ֹͣ�����¿���
//	if(isOK!=HAL_OK){
//		return ;
//	}
	
    while (1)
    {
        osDelay(100);
		//usb read
        //uint16_t RcvedLen = readData(RxBuffer.RxBuf + RxBuffer.RxLen, RxIdleLen(RxBuffer));
		//���� �ж�read
		//uint16_t RcvedLen = USART_ReadData(RxBuffer.RxBuf + RxBuffer.RxLen, RxIdleLen(RxBuffer));
		//���� DMA read
		
		if(BinarySem_Rx!=NULL){
			if(xSemaphoreTake(BinarySem_Rx,50*portTICK_RATE_MS)==pdFAIL){
				count_USB++;
				if(count_USB>=100){
					sFlag.disconnetFlag=1;
					count_USB=0;
					printf(" ReadTask serial read error ,disconnect\r\n");
				}
				continue;
			}
		}
		if(RxBuffer.RxLen>=RX_BUF_LEN){
			RxBuffer.RxLen = 0;
		}
		//uint16_t RcvedLen = DMA_ReadData(tempRxBuff.RxBuf + tempRxBuff.RxLen, RxIdleLen(tempRxBuff));
		//uint16_t RcvedLen = DMA_ReadData(RxBuffer.RxBuf + RxBuffer.RxLen, RxIdleLen(RxBuffer));
		uint16_t RcvedLen = USART_ReadData(RxBuffer.RxBuf + RxBuffer.RxLen, RxIdleLen(RxBuffer));
		//printf("readLen==%d  disconnetFlag:%d=============\r\n",RcvedLen,sFlag.disconnetFlag);
		//////*************printf**********/////
//		printf("RxBuff:\r\n");
//		for(int i=0;i<RxBuffer.RxLen;i++){
//			printf(" %x, ",RxBuffer.RxBuf[i]);
//		}
//		printf("\r\n");
		//////*************printf**********/////
		/////TODO---
		if (RcvedLen==0){
			continue;
		}
		else{
            RxBuffer.RxLen += RcvedLen;
            resolveRxBuf();
			sFlag.disconnetFlag=false;
			count_USB=0;
        }
		
    }
}


static void resolveRxBuf(void)
{
	CheckPos= 0;
	//id + ��β�����ֽڣ�����⵽ĩβ�����ֽ�֮ǰ��
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
//			printf("head and tail found:");
//		for(int i=0;i<TailPos-HeadPos+2;i++){
//			printf(" %x, ",RxBuffer.RxBuf[HeadPos+i]);
//		}
//		printf("\r\n");
//////*************printf**********/////
			ParseFrame(RxBuffer.RxBuf + HeadPos + 2, TailPos - HeadPos - 2);
			//printf("01==head and tail found\r\n");
            RxBuffer.RxLen = RxBuffer.RxLen - (TailPos + 2); //tailPos+2 ���������/
			//TODO        if(RxBuffer.RxLen>0)
			if(RxBuffer.RxLen>0){
				memmove(RxBuffer.RxBuf, RxBuffer.RxBuf + TailPos + 2, RxBuffer.RxLen);
				HeadFound = TailFound = false;
				CheckPos = 0;
				//printf("02==ParseFrame before\r\n");
			}
        }
    }
}

static void commDownSendTask(void *argument)
{
    InitProtocolFrame(controlCmds);
    controlCmds.funcCode = Func_MoveCmd;


    static ControlCmd_s LocalControlCmd;

//    memcpy(&LocalRobotPostureFrame, &RobotPostureFrame, sizeof(RobotPostureFrame));
    memcpy(&LocalControlCmd, &controlCmds, sizeof(controlCmds));
	
	
	int tempCount=0;

    while (1)
    {
		
		int16_t send_len=0;
        osDelay(300);   
        tempCount++;
/*���Դ��� ==============Begin====================*/
#if defined USB_FB_TEST_MODE
        RobotStateFrame.digit++;
        RobotStateFrame.elecInfo.batVoltage = 1.0;
        RobotStateFrame.posture.picth = 1.0;
        RobotStateFrame.tempInfo.temp[0] = 1.0;
        RobotPostureFrame.posture.posX = 1.0;
#endif
/*���Դ��� ==============End=====================*/
        
        taskENTER_CRITICAL();
        memcpy(&LocalControlCmd, &controlCmds, sizeof(controlCmds));
		taskEXIT_CRITICAL();
		//printf("send controlCmds.digit:%d\r\n",controlCmds.digit);		
		//LocalControlCmd.digit = 123;
        LocalControlCmd.checkSum = CheckSum((uint8_t *)&(LocalControlCmd.funcCode), BodyLen(LocalControlCmd));

		// USB send
        //sendData((uint8_t *)&LocalRobotPostureFrame, sizeof(LocalRobotPostureFrame));
		//���� send
		send_len=USART_SendData(&huart5,(uint8_t *)&LocalControlCmd, sizeof(LocalControlCmd));
		
		//����DMA send
		//send_len=DMA_SendData((uint8_t *)&LocalControlCmd, sizeof(LocalControlCmd));
		
    }
}

static void ParseFrame(uint8_t *Data, uint32_t DataLen)
{
    uint8_t sum = CheckSum(Data, DataLen - 1);
    if (Data[DataLen - 1] == sum)   //Data[DataLen - 1]����ֽ�ΪУ��ͣ�������ֽ�֮ǰ��sum
    {
        uint8_t *FrameData = Data + 1; //ȥ��������
		//printf("-----------------------sucesss Data[0]==============%x\r\n",Data[0]);
		static int readcount=0;
        switch (Data[0]){
			default:
            //log_error("Unknow func code: %x\r\n", Data[0]);
            break;
			case Func_Feed_State:
				taskENTER_CRITICAL();
				stateBackToBoxs = *((StateBackToBox_s *)(FrameData));
				taskEXIT_CRITICAL();
	//			printf("======Func_MoveCmd %x  manualSpeed=%3.2f autoSpeed=%3.2f controlDigit:%d\r\n",
	//						Func_MoveCmd,MoveCmd.speed,MoveCmd.autoSpeed,MoveCmd.digit);
				break;
//			case Func_Feed_later:

//				taskENTER_CRITICAL();
//				stateFeedBox_later = *(StateFeedToBox_later_s *)(FrameData);
//				taskEXIT_CRITICAL();
//	//			printf("======FUnc_ParamConfig %x  picDis=%3.2f obstacleDis=%3.2f \r\n",
//	//					FUnc_ParamConfig,ParamConfigs.picDis,ParamConfigs.obstacleDis);
//				break;


        

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
        log_error("Check sum not match, expect:%x, actual:%x\r\n", Data[DataLen - 1], sum);
		
	}
}


///**
// * @description: sum from index [0, DataLen-1]
// * @param {type} 
// * @return: 
// */
//static uint8_t CheckSum(uint8_t *Data, uint32_t DataLen)
//{
//    uint8_t sum = 0;
//    for (int i = 0; i < DataLen; i++)
//    {
//        sum += Data[i];
//    }
//    return sum;
//}



