#include "can_handler.h"
#include "cmsis_os.h"
#include <string.h>
#include "common.h"
#include "motor.h"

#define __CAN_TX_TEST0  //__CAN_TX_TEST���һֱ���Ͳ�������
int tempt=0;
#define LOCAL_MSG_BUF_LEN 32





//can����/������Ϣ���о��
QueueHandle_t      		canRxMsgQueueHandle = NULL;
static QueueHandle_t      canTxMsgQueueHandle = NULL;

//static CanRxBuffer_s localCanRxBuffer;
//static CanTxBuffer_s localCanTxBuffer;
static CanTxBuffer_s CanTxMsgBuffer;//
CanRxBuffer_s CanRxMsgBuffer;
static CanRxMsg_s   localRxMsg[LOCAL_MSG_BUF_LEN];

uint16_t     localRxCnt;
#define idleLen()  (LOCAL_MSG_BUF_LEN-localRxCnt)

static TaskHandle_t CanReadTaskHandle = NULL;
static TaskHandle_t CanSendTaskHandle = NULL;

static void canReadTask(void *argument);
static void canSendTask(void *argument);
static void resolveRxFrame(void);
static uint16_t popRxMsg(void);

extern CAN_HandleTypeDef hcan1;
static SemaphoreHandle_t BinarySem_CanTx = NULL;
SemaphoreHandle_t BinarySem_CanRx = NULL;


extern bool g_motorEnFlag;
void initCANHandler()
{
    
	//memset(&localCanRxBuffer, 0, sizeof(localCanRxBuffer));
   // memset(&localCanTxBuffer, 0, sizeof(localCanTxBuffer));
	memset(&CanTxMsgBuffer, 0, sizeof(CanTxMsgBuffer));
	memset(&CanRxMsgBuffer, 0, sizeof(CanRxMsgBuffer));

    HAL_CAN_Start(&hcan1);                                             // ����CAN
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //���������ж�����
	
//	//�ź���ͬ��
	BinarySem_CanTx = xSemaphoreCreateBinary();
	BinarySem_CanRx = xSemaphoreCreateBinary();
	
	//��Ϣ����ͬ��
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
                512,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                /* �������  */
                7,                   /* �������ȼ�*/
                &CanReadTaskHandle); /* ������  */
    xTaskCreate(canSendTask,         /* ������  */
                "CanSendTask",       /* ������    */
                512,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
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
		if(!sFlag.motorFlags.nmt_startNodeFlag){
			//TODO ȷ���������ɹ������ʹ�ܵ��
			startAllNode_NMT();
			heartbeatFrame_send();
			sFlag.motorFlags.heartbeatSendFlag = true;
//			printf("send nmt,   g_motorEnFlag = %d, freqCount=%d\r\n",
//				g_motorEnFlag,freqCount);
		}
		freqCount++;
		//TODO PDO�������Զ�����
		if(freqCount%50==0){
			heartbeatFrame_send();
			//printf("send heartbeat \r\n");
			freqCount=0;
		}

		
		//osDelay(10);

//		if(xQueueReceive(canRxMsgQueueHandle,&localCanRxBuffer,30*portTICK_RATE_MS)==pdPASS){
//			resolveRxFrame();
//			
//		}
////		//�ź���ͬ��
//		static int16_t errorCount=0;
//		if(BinarySem_CanRx!=NULL){
//			if(xSemaphoreTake(BinarySem_CanRx,50*portTICK_RATE_MS)==pdFAIL){
//				errorCount++;
//				if(errorCount>100){
//					sFlag.motorFlags.nmt_startNodeFlag = false;
//					errorCount=0;
//				}
//				continue;
//			}
//		}else{
//			osDelay(50);
//		}
		//errorCount =0;
		osDelay(15);
		if(g_motorEnFlag){
			resolveRxFrame();
		}

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
        
        while (CanTxMsgBuffer.cnt > 0 && freeCnt > 0)
        {
            // ÿ�η��ͻ�������һ֡��FIFO
			//osDelay(1);
            HAL_StatusTypeDef canStatus = HAL_CAN_AddTxMessage(&hcan1, &(CanTxMsgBuffer.msg[0].header), CanTxMsgBuffer.msg[0].data, (uint32_t *)CAN_TX_MAILBOX0);
            if (canStatus == HAL_OK)
            {
                freeCnt--;
                taskENTER_CRITICAL(); //�ٽ���
                CanTxMsgBuffer.cnt--;
				taskEXIT_CRITICAL();
                memmove(CanTxMsgBuffer.msg, CanTxMsgBuffer.msg + 1, sizeof(CanTxMsgBuffer.msg[0]) * CanTxMsgBuffer.cnt); // δ��������ǰ��
                
            }
            else
            {
                log_error("CAN Tx Error: %d\r\n", canStatus);
            }
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
    
    memcpy(&(CanTxMsgBuffer.msg[CanTxMsgBuffer.cnt].header), header, sizeof(CanTxMsgBuffer.msg[0].header));
    memcpy(CanTxMsgBuffer.msg[CanTxMsgBuffer.cnt].data, data, sizeof(CanTxMsgBuffer.msg[0].data));
	taskENTER_CRITICAL(); //�ٽ���
    CanTxMsgBuffer.cnt = (CanTxMsgBuffer.cnt + 1)% CAN_MSG_BUF_LEN;   // ѭ����ӣ��������
    taskEXIT_CRITICAL();
	//�ź���ͬ��
	//xSemaphoreGive(BinarySem_CanTx);
	xSemaphoreGiveFromISR(BinarySem_CanTx,&xHigherPriorityTaskWoken);
	
	//////TODO----��Ϣ����
//	static CanTxMsg_s CanTxMsg_s;
//	&CanTxMsg_s.header=header;
//	memcpy(&(CanTxMsg_s.header), header, sizeof(CanTxMsg_s.header));
//	memcpy(CanTxMsg_s.data, data, sizeof(CanTxMsg_s.data));
//	//��Ϣ����
//	xQueueSend(canTxMsgQueueHandle,&CanTxMsg_s,20);
	
    return CanTxMsgBuffer.cnt;
}

static uint16_t popRxMsg()
{
    if (CanRxMsgBuffer.cnt == 0)
        return 0;
		
    
    uint16_t freeCnt = idleLen();
    
    if (CanRxMsgBuffer.cnt > 0 ){
		if(freeCnt >= CanRxMsgBuffer.cnt)
		{
			//taskENTER_CRITICAL(); //�ٽ���
			memcpy(localRxMsg + localRxCnt, CanRxMsgBuffer.msg, CanRxMsgBuffer.cnt * sizeof(CanRxMsg_s));
			
			localRxCnt += CanRxMsgBuffer.cnt;
			CanRxMsgBuffer.cnt = 0;
			//taskEXIT_CRITICAL();
			return CanRxMsgBuffer.cnt;
		}
		else
		{
			//taskENTER_CRITICAL(); //�ٽ���
			memcpy(localRxMsg + localRxCnt, CanRxMsgBuffer.msg, freeCnt * sizeof(CanRxMsg_s));
			memmove(CanRxMsgBuffer.msg, CanRxMsgBuffer.msg + freeCnt, (CanRxMsgBuffer.cnt - freeCnt) * sizeof(CanRxMsg_s));
			
			localRxCnt += freeCnt;
			//CanRxMsgBuffer.cnt -= freeCnt;
			//taskEXIT_CRITICAL();
			return freeCnt;
		}
    }
}

static void resolveRxFrame(void)
{
    if(popRxMsg()==0){
		//return ;
	}

//	// if(tempt%10==0){
//	// 	printf("resolve :localRxCnt%d\r\n ",localRxCnt);
//	// 		tempt=0;
//	// 	}
//	// tempt++;

	//localRxCnt = 10;
    for(int i = 0; i < localRxCnt; i++)
    {
        resolveFeedback(&localRxMsg[i]);
    }
    localRxCnt = 0; //������ɺ���½��ռ���
}
