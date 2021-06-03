#ifndef __BUSIIC2_H
#define __BUSIIC2_H
#include "stm32f4xx.h"






//   	   		   
////IO方向设置
//#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0x00008000;}
//#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=0x00003000;}
#define SDA_INII()  {GPIOB->MODER&=~(3<<(15*2));GPIOB->MODER|=0<<15*2;}	//PB15输入模式
#define SDA_OUTII() {GPIOB->MODER&=~(3<<(15*2));GPIOB->MODER|=1<<15*2;} //PB15输出模式,SDA
#define IIC_SCLII(pin_status)        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, pin_status)
#define IIC_SDAII(pin_status)        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, pin_status)
#define READ_SDAII          (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) != GPIO_PIN_RESET)
 



#define STATUS_OKII       0x00
#define STATUS_FAILII     0x01

// 


//IIC所有操作函数
void IIC_InitII(void);                //初始化IIC的IO口				 
void IIC_StartII(void);				//发送IIC开始信号
void IIC_StopII(void);	  			//发送IIC停止信号
void IIC_Send_ByteII(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_ByteII(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_AckII(void); 				//IIC等待ACK信号
void IIC_AckII(void);					//IIC发送ACK信号
void IIC_NAckII(void);				//IIC不发送ACK信号

void IIC_Write_One_ByteII(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_ByteII(uint8_t daddr,uint8_t addr);	 
unsigned char I2C_ReadkeyII(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByteII(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByteII(unsigned char dev, unsigned char reg, unsigned char data);
unsigned char IICwriteCmdII(unsigned char dev, unsigned char cmd);
uint8_t IICwriteBytesII(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t IICwriteBitsII(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t IICwriteBitII(uint8_t dev,uint8_t reg,uint8_t bitNum,uint8_t data);
uint8_t IICreadBytesII(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
#endif
















