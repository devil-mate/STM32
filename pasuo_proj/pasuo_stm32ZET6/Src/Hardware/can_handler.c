#include "can_handler.h"
#include "cmsis_os.h"
#include <string.h>
#include "common.h"
#include "motor.h"

#define __CAN_TX_TEST0  //__CAN_TX_TEST则会一直发送测试数据
int tempt=0;
#define LOCAL_MSG_BUF_LEN 32





//can接收/发送消息队列句柄
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

    HAL_CAN_Start(&hcan1);                                             // 开启CAN
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //开启挂起中断允许
	
//	//信号量同步
	BinarySem_CanTx = xSemaphoreCreateBinary();
	BinarySem_CanRx = xSemaphoreCreateBinary();
	
	//消息队列同步
	//canRxMsgQueueHandle=xQueueCreate(4,sizeof(localCanRxBuffer));
/*测试代码 ==============Begin====================*/
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
        // 注意：邮箱设置为优先级发送模式时，实际发送顺序可能与数组顺序不同，因为多帧发送时会仲裁优先级
        log_info("Index:%d\tID:%x\tData[0]:%x\r\n", i, CanTxBuffer.msg[i].header.StdId, CanTxBuffer.msg[i].data[0]);
    }
    
#endif
/*测试代码 ==============End=====================*/
    
    xTaskCreate(canReadTask,         /* 任务函数  */
                "CanReadTask",       /* 任务名    */
                512,                 /* 任务栈大小，单位word，也就是4字节 */
                NULL,                /* 任务参数  */
                7,                   /* 任务优先级*/
                &CanReadTaskHandle); /* 任务句柄  */
    xTaskCreate(canSendTask,         /* 任务函数  */
                "CanSendTask",       /* 任务名    */
                512,                 /* 任务栈大小，单位word，也就是4字节 */
                NULL,                /* 任务参数  */
                11,                   /* 任务优先级*/
                &CanSendTaskHandle); /* 任务句柄  */
}

static void canReadTask(void *argument)
{	
	
	//发送读取命令， pushTxMsg0 把需要的数据放到发送邮箱
	//TODO 等待使能后读取数据
	//osDelay(5000);
	//pushTx 触发发送信号
	//g_motorEnFlag 为true保证使能电机之后?
	int16_t freqCount=0;
    while (1)
    {
		if(!sFlag.motorFlags.nmt_startNodeFlag){
			//TODO 确保心跳包成功后才能使能电机
			startAllNode_NMT();
			heartbeatFrame_send();
			sFlag.motorFlags.heartbeatSendFlag = true;
//			printf("send nmt,   g_motorEnFlag = %d, freqCount=%d\r\n",
//				g_motorEnFlag,freqCount);
		}
		freqCount++;
		//TODO PDO驱动器自动发送
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
////		//信号量同步
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

	//消息队列
	//canTxMsgQueueHandle=xQueueCreate(4,sizeof(localCanTxBuffer));
	
    while (1)
    {
		
		//osDelay(100);
		//readMotorVelocity();
		//信号量同步
		if(BinarySem_CanTx!=NULL){
			if(xSemaphoreTake(BinarySem_CanTx,30*portTICK_RATE_MS)==pdFAIL){
				continue;
			}
		}else{
			osDelay(30);
		}
		//消息队列
//		if(xQueueReceive(canTxMsgQueueHandle,&localCanTxBuffer,30*portTICK_RATE_MS)!=pdPASS){
//			continue;
//		}
/*测试代码 ==============Begin====================*/
#if defined __CAN_TX_TEST
        static uint16_t id = 0;
        if(CanTxBuffer.cnt == 0)
        {
            CanTxBuffer.cnt = 1;
            CanTxBuffer.msg[0].header.StdId = id++;
        }
#endif
/*测试代码 ==============End=====================*/

        uint32_t freeCnt = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
        
        while (CanTxMsgBuffer.cnt > 0 && freeCnt > 0)
        {
            // 每次发送缓冲区第一帧，FIFO
			//osDelay(1);
            HAL_StatusTypeDef canStatus = HAL_CAN_AddTxMessage(&hcan1, &(CanTxMsgBuffer.msg[0].header), CanTxMsgBuffer.msg[0].data, (uint32_t *)CAN_TX_MAILBOX0);
            if (canStatus == HAL_OK)
            {
                freeCnt--;
                taskENTER_CRITICAL(); //临界区
                CanTxMsgBuffer.cnt--;
				taskEXIT_CRITICAL();
                memmove(CanTxMsgBuffer.msg, CanTxMsgBuffer.msg + 1, sizeof(CanTxMsgBuffer.msg[0]) * CanTxMsgBuffer.cnt); // 未发送数据前移
                
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
//信号量同步+全局变量缓冲区
	static BaseType_t xHigherPriorityTaskWoken;
    
    memcpy(&(CanTxMsgBuffer.msg[CanTxMsgBuffer.cnt].header), header, sizeof(CanTxMsgBuffer.msg[0].header));
    memcpy(CanTxMsgBuffer.msg[CanTxMsgBuffer.cnt].data, data, sizeof(CanTxMsgBuffer.msg[0].data));
	taskENTER_CRITICAL(); //临界区
    CanTxMsgBuffer.cnt = (CanTxMsgBuffer.cnt + 1)% CAN_MSG_BUF_LEN;   // 循环添加，避免溢出
    taskEXIT_CRITICAL();
	//信号量同步
	//xSemaphoreGive(BinarySem_CanTx);
	xSemaphoreGiveFromISR(BinarySem_CanTx,&xHigherPriorityTaskWoken);
	
	//////TODO----消息队列
//	static CanTxMsg_s CanTxMsg_s;
//	&CanTxMsg_s.header=header;
//	memcpy(&(CanTxMsg_s.header), header, sizeof(CanTxMsg_s.header));
//	memcpy(CanTxMsg_s.data, data, sizeof(CanTxMsg_s.data));
//	//消息队列
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
			//taskENTER_CRITICAL(); //临界区
			memcpy(localRxMsg + localRxCnt, CanRxMsgBuffer.msg, CanRxMsgBuffer.cnt * sizeof(CanRxMsg_s));
			
			localRxCnt += CanRxMsgBuffer.cnt;
			CanRxMsgBuffer.cnt = 0;
			//taskEXIT_CRITICAL();
			return CanRxMsgBuffer.cnt;
		}
		else
		{
			//taskENTER_CRITICAL(); //临界区
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
    localRxCnt = 0; //解析完成后更新接收计数
}
