#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
//#define MotorL (0)
//#define MotorR (1)
#include <stdbool.h>
typedef enum{
	MotorL =0,
	MotorR =1
}MotorID_e;

#define CAN_MOTOTR (0)
#define UART_MOTOTR (1)

typedef uint8_t MotorIndex;
/*平台结构参数*/
#define PLATFORM_WHEEL_DIA              (60.0f)//直径60mm
#define PLATFORM_WHEEL_LEN            	(165.0f) //两轮间隔
#define PLATFORM_MAX_SPEED_M_PER_MIN    (47.0f) //m/min
#define PLATFORM_GEAR_RATIO				(2.0f) //齿轮比


#define PLATFORM_MAX_OMEGA              (6.0f)
#define PI  3.1415926f
#define	DIA_RATE						(0.8f) // 轮子的一定比例计算实际里程
#define PULS_NUM						(4000) //一圈脉冲数

#define CODER_WHEEL_DIA              (50.0f)//
#pragma pack(push, 1)
#if (UART_MOTOTR == 1)
	#define MOTOR_MAX_RPM					 40
#elif (CAN_MOTOTR == 1)

#endif
__weak bool setWheelVelocity(int16_t rpmL,int16_t rpmR); 
__weak bool getWheelVelocity(float *velocityL,float *velocityR); 




#endif
