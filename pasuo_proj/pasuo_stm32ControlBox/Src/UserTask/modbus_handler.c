#include "modbus_handler.h"
#include "modbus.h"
#include "cmsis_os.h"
#include <string.h>
#include "common.h"


#define __CAN_TX_TEST0  //__CAN_TX_TEST���һֱ���Ͳ�������


#define LOCAL_MSG_BUF_LEN 32

static uint16_t     localRxCnt;
#define idleLen()  (LOCAL_MSG_BUF_LEN-localRxCnt)

static TaskHandle_t modbusReadTaskHandle = NULL;
static TaskHandle_t modbusSendTaskHandle = NULL;
static void modbusReadTask(void *argument);
static void modbusSendTask(void *argument);

static void resolveModRxFrame(void);



void initModHandler()
{



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
    
    xTaskCreate(modbusReadTask,         /* ������  */
                "modbusReadTask",       /* ������    */
                512,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                /* �������  */
                2,                   /* �������ȼ�*/
                &modbusReadTaskHandle); /* ������  */
    xTaskCreate(modbusSendTask,         /* ������  */
                "modbusSendTask",       /* ������    */
                512,                 /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                /* �������  */
                1,                   /* �������ȼ�*/
                &modbusSendTaskHandle); /* ������  */
}

static void modbusReadTask(void *argument)
{

    while (1)
    {
        osDelay(50);

        //resolveModRxFrame();
    }
}

static void modbusSendTask(void *argument)
{

    while (1)
    {
        osDelay(20);
        
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
        
        
        
    }
}

//uint16_t pushTxMsg(CanTxMsg_s *msg)
//{
//    return pushTxMsg0(&(msg->header), msg->data);
//}

//uint16_t pushTxMsg0(CAN_TxHeaderTypeDef *header, uint8_t data[])
//{
//    taskENTER_CRITICAL(); //�ٽ���
//    memcpy(&(CanTxBuffer.msg[CanTxBuffer.cnt].header), header, sizeof(CanTxBuffer.msg[0].header));
//    memcpy(CanTxBuffer.msg[CanTxBuffer.cnt].data, data, sizeof(CanTxBuffer.msg[0].data));
//    CanTxBuffer.cnt = (CanTxBuffer.cnt + 1)% CAN_MSG_BUF_LEN;   // ѭ����ӣ��������
//    taskEXIT_CRITICAL();
//    return CanTxBuffer.cnt;
//}

//static uint16_t popRxMsg()
//{
//    if (CanRxBuffer.cnt == 0)
//        return 0;
//    
//    uint16_t freeCnt = idleLen();
//    
//    if (CanRxBuffer.cnt > 0 && freeCnt >= CanRxBuffer.cnt)
//    {
//        taskENTER_CRITICAL(); //�ٽ���
//        memcpy(localRxMsg + localRxCnt, CanRxBuffer.msg, CanRxBuffer.cnt * sizeof(CanRxMsg_s));
//        
//        localRxCnt += CanRxBuffer.cnt;
//        CanRxBuffer.cnt = 0;
//        taskEXIT_CRITICAL();
//        return CanRxBuffer.cnt;
//    }
//    else
//    {
//        taskENTER_CRITICAL(); //�ٽ���
//        memcpy(localRxMsg + localRxCnt, CanRxBuffer.msg, freeCnt * sizeof(CanRxMsg_s));
//        memmove(CanRxBuffer.msg, CanRxBuffer.msg + freeCnt, (CanRxBuffer.cnt - freeCnt) * sizeof(CanRxMsg_s));
//        
//        localRxCnt += freeCnt;
//        CanRxBuffer.cnt -= freeCnt;
//        taskEXIT_CRITICAL();
//        return freeCnt;
//    }
//}

//static void resolveModRxFrame(void)
//{
//    popRxMsg();

//    for(int i = 0; i < localRxCnt; i++)
//    {
////        log_info("RXID:%x\t Data[0]:%x\r\n", localRxMsg[i].header.StdId, localRxMsg[i].data[0]);
//        resolveModFeedback(&localRxMsg[i]);
//    }
//    localRxCnt = 0; //������ɺ���½��ռ���
//}
