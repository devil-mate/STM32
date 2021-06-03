/**
 * Linux��λ����Ƕ��ʽ��ͨѶЭ�����Ͷ���
 **/
#ifndef __PROTOCOL_TYPE_H
#define __PROTOCOL_TYPE_H

#include <stdint.h>
#include "cmsis_os.h"


typedef uint16_t ProtocolMask;

#define PROTOCOL_HEAD (0xB3BB)
#define PROTOCOL_TAIL (0xDDD3)

#define InitProtocolFrame(t)     \
    t.head = PROTOCOL_HEAD; \
    t.tail = PROTOCOL_TAIL
#define BodyLen(t) (sizeof(t) -   sizeof(PROTOCOL_HEAD) - 1)

#define CheckMask(addr, mask) (*((uint16_t *)addr) == (uint16_t)mask)
#pragma pack(push, 1)
typedef enum
{
    /*Command*/
    Func_MoveCmd        = 0xC1,
	FUnc_ParamConfig	= 0xC2,
    //Func_VideoCamCmd    = 0xC2,
    Func_LidarInfo      = 0xC3,
    Func_PidParamCmd    = 0xC4,
	

    /*Feedback*/
    Func_RobotPosture   = 0xF1,
    Func_StateFeed     = 0xF2,
	Func_StateFrame		= 0xF3,
} FuncCode_e;   //������(������)

/*��λ���·�*/
typedef struct
{
    float posX;
} Posture_s;    //������λ�ˣ���X Y����ͺ����

typedef struct
{
    uint16_t     digit; //���ֱ�־λ��
    float       speed;	//�ֶ��ٶ�
	float       autoSpeed;//�Զ��ٶ�
    Posture_s   posture;
} MoveCmd_s; //�������˶�ָ��

typedef struct
{
    uint8_t digit;      //���ֱ�־λ,bit0=1ȫ�����㣬bit0=0��������
    float pitchAcc;     //�����Ǳ仯��,��λdeg/s
    float yawAcc;       //ƫ���Ǳ仯��,��λdeg/s
} VideoCamCmd_s;        //��Ƶ����˶�ָ��

typedef struct
{
    uint8_t digit; //���ֱ�־λ
} LidarInfo_s;     //�����״�����֡

typedef struct
{
	float Kp;
	float Ki;
    float Kd;
	float IGate;    //��������
	float OutGate;  //���������
} PidParamCmd_s;    //�·�PID����

typedef struct
{
	float obstacleDis[2]; //
	uint16_t taskTotal;
	float picDis;
} ParamConfig_s;    //


/*��λ������*/

typedef struct 
{
    float batVoltage;
}ElecInfo_s;

typedef struct 
{
    float temp[2];
}TemperatureInfo_s;

typedef struct
{	
	float roll;
    float picth;
    float yaw;	   
} IMUPosture_s; 


typedef struct
{
	float manulTargetSpeed;//�ֶ� Ŀ���ٶ� 
	float autoTargetSpeed;//�Զ�ʱ Ŀ���ٶ�
	float feedbackSpeed;//ʵ���ٶ�
}MotionState_s;

typedef struct 
{
    float wheelRPM[2];//������ʵ���ٶ�
    float motorCurrent[2];//������ʵ������
}MotorState_s;


typedef struct
{
    uint16_t            head;
    uint8_t             funcCode;

    Posture_s           posture;

    uint8_t             checkSum;
    uint16_t            tail;
} RobotPostureFrame_s; //������λ����Ϣ����

// 
typedef struct
{
    uint16_t            head;
    uint8_t             funcCode;

    uint8_t            digit;
    uint16_t           motorState;//���������
    ElecInfo_s          elecInfo;
    TemperatureInfo_s   temperature;   
    MotionState_s       sMotionState; //�˶�״̬
    MotorState_s        sMotorState; //���״̬
	
    uint8_t             checkSum;
    uint16_t            tail;
} RobotStateFrame_s; //������״̬��Ϣ���Ӻ���

/////////////////////////////////////////

typedef struct
{
	float obstacleDis[2];//ǰ���ϰ�����루�����볬���ںϺ��ʵ�ʾ��룩

}ObstacleDis_s;

typedef struct
{
	float odom;//��̼���Ϣ
	float time;//ʱ��
	//bool picTrigger;//�����źţ�digit,10��
}picOdom_s;

// 2+8+8 + 6 = 24
typedef struct
{
    uint16_t            head;
    uint8_t             funcCode;
	
    uint16_t             digit;      //���ֱ�־λ
	ObstacleDis_s 		sObstacleDis;
    picOdom_s       	sPicOdom;
	//float				realOdom;
	
    uint8_t             checkSum;
    uint16_t            tail;
} RobotStateFeed_s; //������״̬��Ϣ��������




#pragma pack(pop)


extern ParamConfig_s		ParamConfigs;
extern QueueHandle_t      QueueHandle;


extern RobotStateFrame_s   RobotStateFrame;
extern RobotStateFeed_s   RobotStateFeed;

#endif
