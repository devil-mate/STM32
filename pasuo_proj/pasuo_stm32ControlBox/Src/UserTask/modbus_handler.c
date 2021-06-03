#include "modbus_handler.h"
#include "modbus.h"
#include "cmsis_os.h"
#include <string.h>
#include "common.h"


#define __CAN_TX_TEST0  //__CAN_TX_TEST则会一直发送测试数据


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
    
    xTaskCreate(modbusReadTask,         /* 任务函数  */
                "modbusReadTask",       /* 任务名    */
                512,                 /* 任务栈大小，单位word，也就是4字节 */
                NULL,                /* 任务参数  */
                2,                   /* 任务优先级*/
                &modbusReadTaskHandle); /* 任务句柄  */
    xTaskCreate(modbusSendTask,         /* 任务函数  */
                "modbusSendTask",       /* 任务名    */
                512,                 /* 任务栈大小，单位word，也就是4字节 */
                NULL,                /* 任务参数  */
                1,                   /* 任务优先级*/
                &modbusSendTaskHandle); /* 任务句柄  */
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
        
        
        
    }
}

//uint16_t pushTxMsg(CanTxMsg_s *msg)
//{
//    return pushTxMsg0(&(msg->header), msg->data);
//}

//uint16_t pushTxMsg0(CAN_TxHeaderTypeDef *header, uint8_t data[])
//{
//    taskENTER_CRITICAL(); //临界区
//    memcpy(&(CanTxBuffer.msg[CanTxBuffer.cnt].header), header, sizeof(CanTxBuffer.msg[0].header));
//    memcpy(CanTxBuffer.msg[CanTxBuffer.cnt].data, data, sizeof(CanTxBuffer.msg[0].data));
//    CanTxBuffer.cnt = (CanTxBuffer.cnt + 1)% CAN_MSG_BUF_LEN;   // 循环添加，避免溢出
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
//        taskENTER_CRITICAL(); //临界区
//        memcpy(localRxMsg + localRxCnt, CanRxBuffer.msg, CanRxBuffer.cnt * sizeof(CanRxMsg_s));
//        
//        localRxCnt += CanRxBuffer.cnt;
//        CanRxBuffer.cnt = 0;
//        taskEXIT_CRITICAL();
//        return CanRxBuffer.cnt;
//    }
//    else
//    {
//        taskENTER_CRITICAL(); //临界区
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
//    localRxCnt = 0; //解析完成后更新接收计数
//}
