#ifndef _RS485_H
#define _RS485_H
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"



  
__IO uint8_t Rx_Buf[255];            // ���ջ���,���256�ֽ�
__IO uint8_t Tx_Buf[255];            // ���ͻ���,���256�ֽ�
__IO uint8_t tmp_Rx_Buf;             // ��ʱ���ջ���
__IO uint16_t RxCount = 0;      // �����ַ�����


#define TIME_OVERRUN  50

typedef enum {
  MSG_ERR_FLAG  , // ���մ��� �ַ��䳬ʱ
  MSG_IDLE      , // ����״̬
  MSG_RXING     , // ���ڽ�������
  MSG_COM       , // �������
  MSG_INC       , // ����֡������(���ַ���Ŀ��м������1.5���ַ�ʱ��)
  MSG_OT        , // ��ʱ����Ӧ

}MSG_TypeDef;


__IO MSG_TypeDef Rx_MSG = MSG_IDLE;   // ���ձ���״̬


#endif
