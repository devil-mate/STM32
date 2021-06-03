#ifndef _CAN_PLATFORM_PLUSGO_H
#define _CAN_PLATFORM_PLUSGO_H

//#include "stm32f4xx_hal.h"
#include "can_handler.h"
#include "pid.h"
#define MotorL (0)
#define MotorR (1)
typedef uint8_t MotorIndex;
/*ƽ̨�ṹ����*/
//#define PLATFORM_WHEEL_DIA              (60.0f)//ֱ��60mm
//#define PLATFORM_WHEEL_LEN            	(165.0f) //���ּ��
//#define PLATFORM_MAX_SPEED_M_PER_MIN    (47.0f) //m/min
//#define PLATFORM_GEAR_RATIO				(2.0f) //���ֱ�
//#define MOTOR_MAX_RPM					 500

//#define PLATFORM_MAX_OMEGA              (6.0f)
//#define PI  3.1415926f
//#define	DIA_RATE						(0.8f) // ���ӵ�һ����������ʵ�����
//#define PULS_NUM						(4000) //һȦ������

//#define CODER_WHEEL_DIA              (50.0f)//
#pragma pack(push, 1)
typedef struct
{
    float LeftRPM;
    float RightRPM;
} WheelRPM_s;

typedef struct
{
    int32_t realPosition_count[2];
    int32_t lastPosition_count[2];
	float realPosition[2];
} ReadPositon_s;



typedef struct
{
    int16_t LeftTorque;
    int16_t RightTorque;
} WheelTorque_s;

//��ŵ��ʵʱ(����)��Ϣ
typedef struct{
	float 		wheelCurrent[2];
	float 		wheelVelocity[2];
	float 		wheelPosition[2];
	uint16_t	motorState[2];
	uint16_t	voltageState[2];
}RealMotorInfo_s;

#pragma pack(push, 1)
typedef enum {
	C_GearN	 		=0,
	C_GearD 		=1,
	C_GearR			=2
}Gear_e;
//�����ṹ��

typedef struct{
	float 	angle; //ɲ���� 0~100 
	float 	velocity;
	float	torque;	
}MotorFeedData_s;
//���ͽṹ��

typedef struct{
	int16_t		motor01_current;
	int16_t		motor02_current;
	int16_t		motor03_current;
	int16_t		motor04_current;

}SendToPlatformData_s;



#pragma pack(pop)

extern MotorFeedData_s sMotorFeedData_g[];
extern pid_type_def sMotorVolecityPID[];

uint8_t initControl();
//uint8_t sendControlFrame_(uint8_t gearNDR,int16_t steerAngSpeed,int16_t steerAng,uint8_t throttle,uint8_t pressureDecel);
//uint8_t sendControlFrame(const SendToPlatformData_s* sendToPlatformDatas);
uint8_t sendControlFrame(); 
float getRobotV(float* speedV,float* speedW);





#endif
