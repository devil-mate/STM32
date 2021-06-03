#include "can_handler.h"
#include "cmsis_os.h"
#include <string.h>
#include "common.h"
#include <FreeRTOS.h>
#include "ringbuf.h"
#include "can_platform_c610Motor.h"
#include "can_platform.h"

#define __CAN_TX_TEST0  //__CAN_TX_TEST���һֱ���Ͳ�������
int tempt=0;

static TaskHandle_t CanReadTaskHandle = NULL;
static TaskHandle_t CanSendTaskHandle = NULL;

static void canReadTask(void *argument);
static void canSendTask(void *argument);
static void resolveRxFrame(void);
static uint16_t popRxMsg(void);

extern CAN_HandleTypeDef hcan1;
static SemaphoreHandle_t BinarySem_CanTx = NULL;
SemaphoreHandle_t BinarySem_CanRx = NULL;
static int8_t can_filter_init();

extern bool g_motorEnFlag;
extern uint8_t chassis_can_send_data[8];
static int16_t targetVec=5000;
static float Testkp=0.35;
static float Testki=0.02;

extern WheelRPM_s wheelRpmTarget_g; //ÿ������Ŀ���ٶ�
void initCANHandler()
{
    
// TODO ���λ�������ʼ������
//	memset(&CanTxMsgBuffer, 0, sizeof(CanTxMsgBuffer));
//	memset(&CanRxMsgBuffer, 0, sizeof(CanRxMsgBuffer));
	can_filter_init();
//	//�ź���ͬ��
	BinarySem_CanTx = xSemaphoreCreateBinary();
	BinarySem_CanRx = xSemaphoreCreateBinary();
	//����
	TxRingBuffMutex = xSemaphoreCreateMutex();
	RxRingBuffMutex = xSemaphoreCreateMutex();
	//test PID
	float PID1[3]={0.35,0.02,0.0};
	float PID2[3]={0.35,0.021,0.0};
	PID_init(&sMotorVolecityPID[MotorL],PID_DELTA,PID1,5000,1000);
	PID_init(&sMotorVolecityPID[MotorR],PID_DELTA,PID2,5000,1000);

	//canRxMsgQueueHandle=xQueueCreate(4,sizeof(localCanRxBuffer));
/*���Դ��� ==============Begin====================*/
#if defined __CAN_TX_TEST
    
    CanTxBuffer.cnt = 10;
    for(int i = 0; i < CanTxBuffer.cnt; i++)
    {
        CanTxBuffer.msg[i].header.StdId = 0x000 + i;
        CanTxBuffer.msg[i].header.DLC = 8;
        CanTxBuffer.msg[i].header.ExtId = 0x00;
        CanTxBuffer.msg[i].header.IDE = CAN_ID_STD;
        CanTxBuffer.msg[i].header.RTR = CAN_RTR_DATA;
        CanTxBuffer.msg[i].data[0] = i;
        log_debug("%d\r\n", i);
    }
    log_info("%d\r\n", pushTxMsg(&(CanTxBuffer.msg[5])));
    log_info("%d\r\n", pushTxMsg(&(CanTxBuffer.msg[4])));
    log_info("%d\r\n", pushTxMsg0(&(CanTxBuffer.msg[1].header), CanTxBuffer.msg[1].data));
    for(int i = 0; i < CanTxBuffer.cnt; i++)
    {
        // ע�⣺��������Ϊ���ȼ�����ģʽʱ��ʵ�ʷ���˳�����������˳��ͬ����Ϊ��֡����ʱ���ٲ����ȼ�
        log_info("Index:%d\tID:%x\tData[0]:%x\r\n", i, CanTxBuffer.msg[i].header.StdId, CanTxBuffer.msg[i].data[0]);
    }
    
#endif
/*���Դ��� ==============End=====================*/
    
    xTaskCreate(canReadTask,         /* ������  */
                "CanReadTask",       /* ������    */
                256,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                /* �������  */
                7,                   /* �������ȼ�*/
                &CanReadTaskHandle); /* ������  */
    xTaskCreate(canSendTask,         /* ������  */
                "CanSendTask",       /* ������    */
                256,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                /* �������  */
                11,                   /* �������ȼ�*/
                &CanSendTaskHandle); /* ������  */
}

static void canReadTask(void *argument)
{	
	
	//���Ͷ�ȡ��� pushTxMsg0 ����Ҫ�����ݷŵ���������
	//TODO �ȴ�ʹ�ܺ��ȡ����
	//osDelay(5000);
	//pushTx ���������ź�
	//g_motorEnFlag Ϊtrue��֤ʹ�ܵ��֮��?
	int16_t freqCount=0;
	 
	
	
    while (1)
    {
		osDelay(20);
		//testMotorControl();
//		if(!sFlag.motorFlags.nmt_startNodeFlag){
//			//TODO ȷ���������ɹ������ʹ�ܵ��
//			startAllNode_NMT();
//			heartbeatFrame_send();
//			sFlag.motorFlags.heartbeatSendFlag = true;
////			printf("send nmt,   g_motorEnFlag = %d, freqCount=%d\r\n",
////				g_motorEnFlag,freqCount);
//		}
//		freqCount++;
//		//TODO PDO�������Զ�����
//		if(freqCount%50==0){
//			heartbeatFrame_send();
//			//printf("send heartbeat \r\n");
//			freqCount=0;
//		}

		
		//osDelay(10);

//		if(xQueueReceive(canRxMsgQueueHandle,&localCanRxBuffer,30*portTICK_RATE_MS)==pdPASS){
//			resolveRxFrame();
//			
//		}
//////		//�ź���ͬ��
		static int16_t errorCount=0;
		if(BinarySem_CanRx!=NULL){
			if(xSemaphoreTake(BinarySem_CanRx,50*portTICK_RATE_MS)==pdFAIL){
				errorCount++;
				if(errorCount>100){
					//sFlag.motorFlags.nmt_startNodeFlag = false;
					PID_clear(&sMotorVolecityPID[MotorL]);
					PID_clear(&sMotorVolecityPID[MotorR]);
					
					errorCount=0;
				}
				continue;
			}
		}else{
			osDelay(50);
		}
//		sMotorVolecityPID[MotorL].Ki=Testki;
//		sMotorVolecityPID[MotorL].Kp=Testkp;
//		sMotorVolecityPID[MotorR].Ki=Testki;
//		sMotorVolecityPID[MotorR].Kp=Testkp;
		errorCount =0;
		resolveRxFrame();
		//����������������PID����
		if(sFlag.canMotorReceiveFlag){
//			printf("targetLrpm: %3.2f, Rrpm:%3.2f\r\n",
//					wheelRpmTarget_g.LeftRPM,wheelRpmTarget_g.RightRPM);
//			printf("feedSpeedL: %3.2f, speedR%3.2f\r\n",
//					sMotorFeedData_g[MotorL].velocity,sMotorFeedData_g[MotorR].velocity);
			PID_calc(&sMotorVolecityPID[MotorL],sMotorFeedData_g[MotorL].velocity,wheelRpmTarget_g.LeftRPM);
			PID_calc(&sMotorVolecityPID[MotorR],sMotorFeedData_g[MotorR].velocity,wheelRpmTarget_g.RightRPM);
			sendControlFrame();
		}
//		sendControlFrame();

    }
}

static void canSendTask(void *argument)
{

	//��Ϣ����
	//canTxMsgQueueHandle=xQueueCreate(4,sizeof(localCanTxBuffer));
	
    while (1)
    {
		
		//osDelay(100);
		//readMotorVelocity();
		//�ź���ͬ��
		if(BinarySem_CanTx!=NULL){
			if(xSemaphoreTake(BinarySem_CanTx,30*portTICK_RATE_MS)==pdFAIL){
				continue;
			}
		}else{
			osDelay(30);
		}
		//��Ϣ����
//		if(xQueueReceive(canTxMsgQueueHandle,&localCanTxBuffer,30*portTICK_RATE_MS)!=pdPASS){
//			continue;
//		}
/*���Դ��� ==============Begin====================*/
#if defined __CAN_TX_TEST
        static uint16_t id = 0;
        if(CanTxBuffer.cnt == 0)
        {
            CanTxBuffer.cnt = 1;
            CanTxBuffer.msg[0].header.StdId = id++;
        }
#endif
/*���Դ��� ==============End=====================*/

	uint32_t freeCnt = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	while (freeCnt > 0)
	{
		osDelay(1);
		static CanTxMsg_s tempCanTxMsg;
		TxRingBuff_ReadOneMsg(&tempCanTxMsg);
		HAL_StatusTypeDef canStatus = HAL_CAN_AddTxMessage(&hcan1, &(tempCanTxMsg.header), tempCanTxMsg.data, (uint32_t *)CAN_TX_MAILBOX0);
		if (canStatus != HAL_OK){
			log_error("CAN Tx Error: %d\r\n", canStatus);
			
		}
		else
		{
			//printf("can_send\r\n");
		}
		freeCnt--;
	}
	
}
}

uint16_t pushTxMsg(CanTxMsg_s *msg)
{
    return pushTxMsg0(&(msg->header), msg->data);
}

uint16_t pushTxMsg0(CAN_TxHeaderTypeDef *header, uint8_t data[])
{
//�ź���ͬ��+ȫ�ֱ���������
	static BaseType_t xHigherPriorityTaskWoken;
    CanTxMsg_s tempCanTxMsg={0};
    memcpy(&tempCanTxMsg.header, header, sizeof(tempCanTxMsg.header));
    memcpy(&tempCanTxMsg.data, data, sizeof(tempCanTxMsg.data));
	TxRingBuff_writeOneMsg(tempCanTxMsg);
	//�ź���ͬ��
	//xSemaphoreGive(BinarySem_CanTx);
	xSemaphoreGiveFromISR(BinarySem_CanTx,&xHigherPriorityTaskWoken);
	
	
    return 0;
}



static void resolveRxFrame(void)
{
	static CanRxMsg_s tempCanRxMsgs;
	for(uint16_t i=0; i<getRxRingBuffLength();i++){
		RxRingBuff_ReadOneMsg(&tempCanRxMsgs);
		resolveFeedback(&tempCanRxMsgs);
	}
}

static int8_t can_filter_init()
{
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK){
    //Error_Handler();
	}
	HAL_CAN_Start(&hcan1);                                             // ����CAN
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //���������ж�����
}

