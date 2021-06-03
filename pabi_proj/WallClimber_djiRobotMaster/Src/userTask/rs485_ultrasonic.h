#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H
#include <stdbool.h>
#include <stdint.h>

//#define RS485_TX_EN  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET) //发送1，接收0
//#define RS485_RX_EN  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET)
void initRS485Data();

//#define FORWARD_ADDR 0xd2
#define LEFT_FORWARD_ADDR 0xe0
#define RIGHT_FORWARD_ADDR 0xe2 //默认e8
#define RIGHT_BACKWARD_ADDR 0xe4
#define LEFT_BACKWARD_ADDR 0xe6 //默认e8
#define CmdLen 3
typedef struct{
	//uint8_t addr; //地址
	uint8_t buff[CmdLen]; //发送的命令：0设备地址，1寄存器，2命令
	
}ultra_dev_s;

enum {
	left_forward_Index ,
	right_forward_Index,
	right_backward_Index ,
	left_backward_Index 
	
};
#endif