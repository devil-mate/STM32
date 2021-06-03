#ifndef __MOTOR_H
#define __MOTOR_H

#include "common.h"
#include "can_handler.h"

#define MotorL (0)
#define MotorR (1)
typedef uint8_t MotorIndex;
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





#pragma pack(pop)
extern WheelRPM_s wheelRpmCmd;
extern ReadPositon_s sReadPositon;//����λ�ã�����ͨ�����������
extern RealMotorInfo_s sRealMotorInfo;
extern WheelTorque_s wheelTorqueS;
uint8_t initAllMotor(void);
uint8_t resetAllMotor(void);
uint8_t setMotorSpeed(void);
uint8_t setMotorTorque();

uint8_t readMotorPosition();
uint8_t readMotorVelocity();
uint8_t readMotorCurrent();
uint8_t readMotorState();
uint8_t readMotorVoltage();

uint8_t onlineCheck(MotorIndex motorId);
uint8_t disenableAllMotor(void);
void resolveFeedback(CanRxMsg_s *msg);

float getVelocity();
void getMotorState(uint8_t motorId);

bool startAllNode_NMT();
bool heartbeatFrame_send();
bool resetFaultAllMotor();

#endif // !__MOTOR_H
