#ifndef _IICDATA_H
#define _IICDATA_H
#include "busiic.h"



//温度传感器地址
#define LM75A_ADDR0				0x48
#define LM75A_ADDR1				0x49
#define LM75A_ADDR2				0x4A
#define LM75A_ADDR3				0x4B
#define LM75A_ADDR4				0x4C
#define vl53l0x_addr1     0x54
#define vl53l0x_addr2     0x56
#define vl53l0x_addr3     0x58
#define vl53l0x_addr4     0x60

#define XSHUT1  GPIO_PIN_7
#define XSHUT2  GPIO_PIN_8
#define XSHUT3  GPIO_PIN_9
#define XSHUT4  GPIO_PIN_10

#define LM75A_TEMP_OUTH_REG		0x00	//温度值高八位寄存器

void initIICData();


float getTemperature(char addr); 
float getDistanceTOF();


int8_t setMode(uint8_t addr,uint8_t mode);//设置精度模式


//void ShortToChar(short sData,unsigned char cData[]);

//short CharToShort(unsigned char cData[]);




#endif