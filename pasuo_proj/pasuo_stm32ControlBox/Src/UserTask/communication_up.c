#include "communication_up.h"
#include <stdbool.h>
#include "protocol_type.h"
#include "usb_type.h"
//#include "usbd_cdc_if.h"
#include "common.h"
#include "bluetooth_rutrans.h"
#include "string.h"
#define USB_FB_TEST_MODE0   // USB_FB_TEST_MODE�����������м����������

// ����λ��ͨ��
//#define RXBUFFERSIZE   100 //�����С
//#define MAXDIS 10000.0   //mm
//uint8_t aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���

extern Flag_s sFlag;

enum {
	forwardDisState,
	backwardDisState,
	runState,
	robotForward,
	robotBackward,
	disLimitF,
	disLimitB,
	motorEn
}stateBackDigt;

extern UART_HandleTypeDef huart3;
static void commUpReadTask(void *argument);
static void commUpSendTask(void *argument);
static TaskHandle_t CommReadTaskHandle = NULL;
static TaskHandle_t CommSendTaskHandle = NULL;

//static void ParseFrame(uint8_t *Data, uint32_t DataLen);

//static void resolveRxBuf(void);
//static bool HeadFound, TailFound;
//static int HeadPos, TailPos, CheckPos;

//RxBuf_s RxBuffer;
//extern RxBuf_s tempRxBuff;

static StateBackToGUI_s StateBackToGUIs;
static StateBackToBox_s LocalstateBackToBoxs;

//static SemaphoreHandle_t BinarySem_Rx = NULL;
//QueueHandle_t      QueueHandle = NULL;


//static Queue_Message_s queueMessage;

/**
 * @description:  ��λ��ͨѶ��д�����ʼ��
 * @param {type} 
 * @return: 
 */
void initCommunication_up()
{
	
//    memset(&RxBuffer, 0, sizeof(RxBuffer));
//	memset(&tempRxBuff, 0, sizeof(tempRxBuff));
    memset(&LocalstateBackToBoxs, 0, sizeof(LocalstateBackToBoxs));



	
//    xTaskCreate(commUpReadTask,        /* ������  */
//                "CommReadTask",        /* ������    */
//                2048,                /* ����ջ��С����λword��Ҳ����4�ֽ� */
//                NULL,               /* �������  */
//                14,                  /* �������ȼ�*/
//                &CommReadTaskHandle); /* ������  */
    xTaskCreate(commUpSendTask,        /* ������  */
                "CommSendTask",       /* ������    */
                2048,                /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,               /* �������  */
                13,                  /* �������ȼ�*/
                &CommSendTaskHandle); /* ������  */
}



static void commUpSendTask(void *argument)
{
//    InitProtocolFrame(StateBackToGUIs);
//    StateBackToGUIs.funcCode = Func_Feed_State;

    memcpy(&LocalstateBackToBoxs, &stateBackToBoxs, sizeof(stateBackToBoxs));
	
	int tempCount=0;

    while (1)
    {
		
		int16_t send_len=0;
        osDelay(100);   
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
        memcpy(&LocalstateBackToBoxs, &stateBackToBoxs, sizeof(stateBackToBoxs));
		if(sFlag.disconnetFlag){
			setbit(LocalstateBackToBoxs.digit,runState);
		}else{
			resetbit(LocalstateBackToBoxs.digit,runState);
		}
		taskEXIT_CRITICAL();
		LocalstateBackToBoxs.modeDigit=123;

		StateBackToGUIs.StateBackToBoxs = LocalstateBackToBoxs;
		
        StateBackToGUIs.checkSum = CheckSum((uint8_t *)&(StateBackToGUIs.funcCode), BodyLen(StateBackToGUIs));
//        LocalRobotPostureFrame.checkSum = CheckSum((uint8_t *)&(LocalRobotPostureFrame.funcCode), BodyLen(LocalRobotPostureFrame));

		// USB send
        //sendData((uint8_t *)&LocalRobotPostureFrame, sizeof(LocalRobotPostureFrame));
		//���� send
		//send_len=USART_SendData((uint8_t *)&StateBackToGUIs, sizeof(StateBackToGUIs));
		send_len=USART_SendData(&huart3,(uint8_t *)&LocalstateBackToBoxs, sizeof(LocalstateBackToBoxs));
		//����DMA send
		//send_len=DMA_SendData((uint8_t *)&LocalRobotStateFeed, sizeof(LocalRobotStateFeed));

		//TODO==================
    }
}


