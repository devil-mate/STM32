/*
 * @Description: printf�ض��򣬿��ض���SWO��USART������ģʽ��ѡһ������define�궨�弴��
 * @Author: liy
 * @Date: 2019-07-01 17:49:10
 * @LastEditTime: 2019-07-01 18:29:42
 * @LastEditors: Please set LastEditors
 */

#include <stdio.h>

/**
 * ģʽ���� 
 */
// �ض���SWO
//#define SWO_PRINT_MODE  //����Keil�����������ã�Option->Debug->(ST-Link Debugger)Settings->Trace������Core ClockΪʵ��ʱ��Ƶ��(��72��168)������ITM_Stimulus_Ports->Enable=0x00000001��

// �ض���UART
 #define USART_PRINT_MODE  //�ض���USARTģʽ�����������ط���ʼ����Ӧ����
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
    while (ITM_Port32(0) == 0)  //ѭ������
      ;
    ITM_Port8(0) = ch;
  }
  return (ch);
}

#elif defined USART_PRINT_MODE
  #include "stm32f4xx_hal.h"
  int fputc(int ch, FILE *f)
  {
    while ((DEBUG_UART->SR & 0X40) == 0)  //ѭ������
      ;
    DEBUG_UART->DR = (unsigned char)ch;
    return ch;
  }

#endif
