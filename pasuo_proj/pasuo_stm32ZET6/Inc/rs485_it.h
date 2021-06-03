#ifndef _RS485_H
#define _RS485_H
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"



  
__IO uint8_t Rx_Buf[255];            // 接收缓存,最大256字节
__IO uint8_t Tx_Buf[255];            // 发送缓存,最大256字节
__IO uint8_t tmp_Rx_Buf;             // 临时接收缓存
__IO uint16_t RxCount = 0;      // 接收字符计数


#define TIME_OVERRUN  50

typedef enum {
  MSG_ERR_FLAG  , // 接收错误 字符间超时
  MSG_IDLE      , // 空闲状态
  MSG_RXING     , // 正在接收数据
  MSG_COM       , // 接收完成
  MSG_INC       , // 数据帧不完整(两字符间的空闲间隔大于1.5个字符时间)
  MSG_OT        , // 超时无响应

}MSG_TypeDef;


__IO MSG_TypeDef Rx_MSG = MSG_IDLE;   // 接收报文状态


#endif
