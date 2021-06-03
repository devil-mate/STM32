#ifndef _MOTOR_KINCO_H
#define _MOTOR_KINCO_H

#include <stdint.h>
#include <cmsis_os.h>

void initRotaMotor();

uint8_t resetMotor();


uint8_t setMotorAngle(float angle);

uint8_t setMotorSpeed_Kinco();

/* �궨�� --------------------------------------------------------------------*/

#define MB_REG_ADDR         0x0003//�Ĵ�����ַ

/* ----Kinco�ŷ��������Ĵ�����ַ---------------- */
#define MB_REG_CON          0x3100 // ����ָ�ʹ��F
#define MB_REG_SPEED        0x6F00 // �ٶ�
#define MB_REG_MODE     		0x3500 // ����ģʽ 3�ٶ�ģʽ��1λ��ģʽ��
#define MB_REG_POS     		  0x4000	//λ��

//��ȡ
#define MB_REG_SATE     		  0x3200	//״̬��

//#define MB_REG_ACOM         0x0007 // AƵ��ָ��ѡ��
//#define MB_REG_FREQUP       0x030A // ����Ƶ���趨Դѡ��


//#define MB_ExREG_CTRL       0x1000 // ͨѶ��������
//#define MB_EXREG_STATE      0x1001 // ��Ƶ��״̬

//#define MB_ExREG_FREQ       0x2000 // ͨѶƵ��(ʵ��ת��Ƶ��)
//#define MB_ExREG_PID        0x2001 // PID����
//#define MB_ExREG_PIDFB      0x2002 // PID����
//#define MB_ExREG_TQ         0x2003 // ת���趨
//#define MB_ExREG_FREQUP     0x2004 // ����Ƶ���趨





#endif