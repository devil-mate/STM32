#ifndef _MODBUS_H
#define _MODBUS_H
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* 类型定义 ------------------------------------------------------------------*/

#define MB_SLAVEADDR        0x0001
/**
  
  运行/停机参数地址：
  3000H~3016H
  
  变频器故障信息地址：
  5000H
  
  */
#define RS485_USARTx                                 USART3
#define RS485_USARTx_BAUDRATE                        19200
#define RS485_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART3_CLK_ENABLE()
#define RS485_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART3_CLK_DISABLE()

#define RS485_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define RS485_USARTx_Tx_GPIO_PIN                     GPIO_PIN_10
#define RS485_USARTx_Tx_GPIO                         GPIOB
#define RS485_USARTx_Rx_GPIO_PIN                     GPIO_PIN_11
#define RS485_USARTx_Rx_GPIO                         GPIOB

#define RS485_USARTx_AFx                             GPIO_AF7_USART3

#define RS485_USART_IRQn                             USART3_IRQn
#define RS485_USART_IRQHANDLER                       USART3_IRQHandler

/* 使用485通信的时候才会用到使能IO */
#define RS485_REDE_GPIO_ClK_ENABLE()                 __HAL_RCC_GPIOH_CLK_ENABLE()
#define RS485_REDE_PORT                              GPIOH
#define RS485_REDE_PIN                               GPIO_PIN_8	
#define RS485_RX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET)
#define RS485_TX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_SET)
void RS485_Tx(uint8_t *Tx_Buf,uint16_t TxCount);
/* 扩展变量 ------------------------------------------------------------------*/
//__IO uint8_t Rx_Buf[255];            // 接收缓存,最大256字节
//__IO uint8_t Tx_Buf[255];            // 发送缓存,最大256字节
//__IO uint8_t tmp_Rx_Buf;             // 临时接收缓存
//__IO uint16_t RxCount = 0;      // 接收字符计数

/* 函数声明 ------------------------------------------------------------------*/
uint16_t ReadWordFromSlave(uint16_t _RegAddr);
uint32_t ReadDWordFromSlave(uint16_t _RegAddr);
uint16_t WriteWordToSlave(uint16_t _RegAddr,uint16_t Data);
uint16_t WriteDWordToSlave(uint16_t _RegAddr,int32_t Data);
void HW_Identify(uint16_t _SlaveAddr);

extern UART_HandleTypeDef huart3; 
//#define husart_RS485  huart3;

#endif /* __BSP_MB_HOST_H__ */





