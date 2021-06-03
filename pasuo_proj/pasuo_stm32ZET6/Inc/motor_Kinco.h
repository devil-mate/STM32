#ifndef _MOTOR_KINCO_H
#define _MOTOR_KINCO_H

#include <stdint.h>
#include <cmsis_os.h>

void initRotaMotor();

uint8_t resetMotor();


uint8_t setMotorAngle(float angle);

uint8_t setMotorSpeed_Kinco();

/* 宏定义 --------------------------------------------------------------------*/

#define MB_REG_ADDR         0x0003//寄存器地址

/* ----Kinco伺服驱动器寄存器地址---------------- */
#define MB_REG_CON          0x3100 // 控制指令，使能F
#define MB_REG_SPEED        0x6F00 // 速度
#define MB_REG_MODE     		0x3500 // 工作模式 3速度模式，1位置模式，
#define MB_REG_POS     		  0x4000	//位置

//读取
#define MB_REG_SATE     		  0x3200	//状态字

//#define MB_REG_ACOM         0x0007 // A频率指令选择
//#define MB_REG_FREQUP       0x030A // 上限频率设定源选择


//#define MB_ExREG_CTRL       0x1000 // 通讯控制命令
//#define MB_EXREG_STATE      0x1001 // 变频器状态

//#define MB_ExREG_FREQ       0x2000 // 通讯频率(实际转动频率)
//#define MB_ExREG_PID        0x2001 // PID给定
//#define MB_ExREG_PIDFB      0x2002 // PID反馈
//#define MB_ExREG_TQ         0x2003 // 转矩设定
//#define MB_ExREG_FREQUP     0x2004 // 上限频率设定





#endif