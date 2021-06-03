#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H
#include <stdbool.h>
#include <stdint.h>

#define RS485_TX_EN  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET) //发送1，接收0
#define RS485_RX_EN  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET)
void initRS485Data();

//#define FORWARD_ADDR 0xd2
#define FORWARD_ADDR 0xe8 //默认e8
#define BACKWARD_ADDR 0xe6
#define CmdLen 3
typedef struct{
	//uint8_t addr; //地址
	uint8_t buff[CmdLen]; //发送的命令,0 地址，1寄存器，2命令
	
}ultra_dev_s;


#endif