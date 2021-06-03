#ifndef __COMMON_H
#define __COMMON_H

#include<stdint.h>
//#include "cmsis_os.h"
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
/*log info*/
#define log_debug(fmt, ...)     printf("[DEBUG](%s|%d)"fmt,__func__,__LINE__,##__VA_ARGS__)
#define log_info(fmt, ...)      printf("[INFO] (%s|%d)"fmt,__func__,__LINE__,##__VA_ARGS__)
#define log_warn(fmt, ...)      printf("[WARN] (%s|%d)"fmt,__func__,__LINE__,##__VA_ARGS__)
#define log_error(fmt, ...)     printf("[ERROR](%s|%d)"fmt,__func__,__LINE__,##__VA_ARGS__)

#define cMIN(a, b) (((a) > (b)) ? (b) : (a))
#define cMAX(a, b) (((a) > (b)) ? (a) : (b))
#define absLimiter(n, l) (n > 0 ? cMIN(n, l) : cMAX(n, -l))
#define cABS(a)	   (((a)>=0)?(a):(-(a)))
#define getbit(x,y)   ((x) >> (y)&1) //获取某一位的值
#define setbit(x,y) ((x)|=(1<<y)); //设置某一位
#define resetbit(x,y) ((x)&=(~(1<<y))) //

#define TEST_MODE

//typedef unsigned char u8;
//typedef unsigned short int  u16;
//typedef unsigned int  u32;

/*测试代码 ==============Begin====================*/
#if defined TEST_MODE

/*User Code*/

#endif
/*测试代码 ==============End=====================*/
int32_t avgFilter(int32_t in, uint8_t depth, uint8_t reset);
float movAvgDisFilter(float in, uint8_t depth, uint8_t reset);

#define VSIZE 20
#define RX_BUF_LEN (128)

#define DEV_COUNT 4 //超声波数量

#pragma pack(push, 1)
typedef struct
{
	uint8_t count;
	float inValue[VSIZE];
}MovAvgFilter_s;

typedef struct
{
	bool enableFlag[2];
	bool motorErrorFlag[2];//
	bool disconnectFlag[2];
	bool nmt_startNodeFlag;
	bool heartbeatSendFlag;

}MotorFlag_s;
//typedef struct
//{
//	bool laserStateFlagF;
//	bool laserStateFlagB;//
//	bool limitFlag[2];
//	

//}Distance_Sensor_s;
typedef struct
{
	bool disconnetFlag;//通信串口 disconnected
	MotorFlag_s motorFlags;
	bool ultraStateFlag[DEV_COUNT];
	bool limitFlag[DEV_COUNT];
	
	bool runningtFlag;
	
	bool autoStopFlag;
	bool triggerFlag;
	bool shutdownFlag;
	bool voltageFlag;
	bool canMotorReceiveFlag;
}Flag_s;
typedef struct
{
	bool linuxToStm32Error;
}ErrorFlag_s;
typedef struct 
{
    uint8_t    RxBuf[RX_BUF_LEN];
    int16_t    RxLen;
} RxBuf_s;
#pragma pack(pop)

#define RxIdleLen(u) (RX_BUF_LEN - u.RxLen)



void movAvgFilterInit(MovAvgFilter_s *MovAvgFilters);
float movAvgFilter(MovAvgFilter_s *MovAvgFilters , float in, uint8_t depth, uint8_t reset);

/**global 全局变量*/

//extern int8_t limitFlag[2];
//extern int8_t laserStateFlagF, laserStateFlagB;
extern Flag_s sFlag;

#define MAX_REC_LENGTH 512
#define REC_LENGTH 2
extern uint8_t UART4_Rx_Buf[MAX_REC_LENGTH]; //USART1存储接收数据
extern UART_HandleTypeDef huart4;
extern unsigned int  UART4_Rx_cnt;                   //USART1接受数据计数器
extern uint8_t  UART4_temp[REC_LENGTH];       //USART1接收数据缓存
//extern QueueHandle_t      rs485RxMsgQueueHandle;
#endif
