/*
 * @Description: printf重定向，可重定向到SWO或USART，两种模式二选一，更改define宏定义即可
 * @Author: liy
 * @Date: 2019-07-01 17:49:10
 * @LastEditTime: 2019-07-01 18:29:42
 * @LastEditors: Please set LastEditors
 */

#include <stdio.h>

/**
 * 模式配置 
 */
// 重定向到SWO
//#define SWO_PRINT_MODE  //需在Keil中做如下设置：Option->Debug->(ST-Link Debugger)Settings->Trace，设置Core Clock为实际时钟频率(如72或168)，设置ITM_Stimulus_Ports->Enable=0x00000001。

// 重定向到UART
 #define USART_PRINT_MODE  //重定向到USART模式，需在其他地方初始化相应串口
 #define DEBUG_UART UART7



#if defined SWO_PRINT_MODE

#define ITM_Port8(n) (*((volatile unsigned char *)(0xE0000000 + 4 * n)))
#define ITM_Port16(n) (*((volatile unsigned short *)(0xE0000000 + 4 * n)))
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4 * n)))

#define DEMCR (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA 0x01000000
struct __FILE
{
  int handle; /* Add whatever you need here */
};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
  if (DEMCR & TRCENA)
  {
    while (ITM_Port32(0) == 0)  //循环发送
      ;
    ITM_Port8(0) = ch;
  }
  return (ch);
}

#elif defined USART_PRINT_MODE
  #include "stm32f4xx_hal.h"
  int fputc(int ch, FILE *f)
  {
    while ((DEBUG_UART->SR & 0X40) == 0)  //循环发送
      ;
    DEBUG_UART->DR = (unsigned char)ch;
    return ch;
  }

#endif
