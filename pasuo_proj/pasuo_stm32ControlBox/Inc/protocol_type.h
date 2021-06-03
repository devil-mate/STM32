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
    /*Feedback*/
    Func_Feed_State    = 0xF2,
//	Func_Feed_later		= 0xF3,
} FuncCode_e;   //������(������)

/*�·�����ݮ��*/

typedef struct
{	
	uint16_t            head;
	uint8_t             funcCode;
	
	//1 �ֶ� 2�Զ� 3��λ 6.ǰ�� 7.����/��ͷ  9.��ͣ
    uint16_t     digit; //���ֱ�־λ��
//    float       manualspeed;	//�ֶ��ٶ� (���Զ��ٶ���linux�У������ɽ������/�����ʼΪֵ)
//	float       autoSpeed;//�Զ��ٶ�
	
	uint8_t             checkSum;
	uint16_t            tail;
} ControlCmd_s; //�������˶�ָ��




/*��ݮ�ɷ���״̬��Ϣ*/

typedef struct 
{
    float batVoltage;
	float temp[2];
}ElecInfo_s;

typedef struct
{
  float speed;          //�������ٶ� 
  float mileage;        //��̼�
} MotionData_s;
typedef struct
{
    float obstacleDis[2];//ǰ���ϰ�����루�����볬���ںϺ��ʵ�ʾ��룩

}ObstacleDis_s;

typedef struct
{
	uint8_t cameraStateDigit; //0�������0/�쳣1��1���1����/�쳣��2���2����/�쳣��3���3����/�쳣
	uint16_t picNum;//��Ƭ����
}CameraState_s;


typedef struct
{
	float taskOdom;//�������������(һ�������Ĵ�ų���)������
	float odom;	// ʵʱ������ݣ������ݼ�
}Odom_s;

typedef struct
{
//6+3+4+8+8+3=32

    uint8_t             modeDigit; //����ģʽ
    uint16_t             digit;      //���ֱ�־λ
    ElecInfo_s          elecInfo;   //������Ϣ(������ʾ
    MotionData_s       MotionData;   //�������ٶȺ���̼�
    ObstacleDis_s       ObstacleDis; //����ǰ���ϰ������
    CameraState_s        CameraSate;//���״̬����������

} StateBackToBox_s; //������״̬��Ϣ����

typedef struct
{
//6+3+4+8+8+3=32
	uint16_t            head; 
	uint8_t             funcCode; //0xF3
	StateBackToBox_s 	StateBackToBoxs;
    uint8_t             checkSum;
	uint16_t            tail;
} StateBackToGUI_s; //������״̬��Ϣ����



#pragma pack(pop)



extern QueueHandle_t      QueueHandle;
extern ControlCmd_s controlCmds;

extern StateBackToBox_s   stateBackToBoxs;

#endif
