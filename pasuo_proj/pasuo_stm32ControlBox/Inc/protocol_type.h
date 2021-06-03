/**
 * Linux上位机与嵌入式板通讯协议类型定义
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
} FuncCode_e;   //功能码(控制字)

/*下发到树莓派*/

typedef struct
{	
	uint16_t            head;
	uint8_t             funcCode;
	
	//1 手动 2自动 3复位 6.前进 7.后退/掉头  9.急停
    uint16_t     digit; //数字标志位，
//    float       manualspeed;	//手动速度 (手自动速度在linux中，或者由界面给出/程序初始为值)
//	float       autoSpeed;//自动速度
	
	uint8_t             checkSum;
	uint16_t            tail;
} ControlCmd_s; //机器人运动指令




/*树莓派反馈状态信息*/

typedef struct 
{
    float batVoltage;
	float temp[2];
}ElecInfo_s;

typedef struct
{
  float speed;          //机器人速度 
  float mileage;        //里程计
} MotionData_s;
typedef struct
{
    float obstacleDis[2];//前后障碍物距离（激光与超声融合后的实际距离）

}ObstacleDis_s;

typedef struct
{
	uint8_t cameraStateDigit; //0相机正常0/异常1，1相机1正常/异常，2相机2正常/异常，3相机3正常/异常
	uint16_t picNum;//照片数量
}CameraState_s;


typedef struct
{
	float taskOdom;//单次任务总里程(一根缆索的大概长度)，递增
	float odom;	// 实时里程数据，递增递减
}Odom_s;

typedef struct
{
//6+3+4+8+8+3=32

    uint8_t             modeDigit; //控制模式
    uint16_t             digit;      //数字标志位
    ElecInfo_s          elecInfo;   //电气信息(电量显示
    MotionData_s       MotionData;   //机器人速度和里程计
    ObstacleDis_s       ObstacleDis; //激光前后障碍物距离
    CameraState_s        CameraSate;//相机状态和拍照数量

} StateBackToBox_s; //机器人状态信息反馈

typedef struct
{
//6+3+4+8+8+3=32
	uint16_t            head; 
	uint8_t             funcCode; //0xF3
	StateBackToBox_s 	StateBackToBoxs;
    uint8_t             checkSum;
	uint16_t            tail;
} StateBackToGUI_s; //机器人状态信息反馈



#pragma pack(pop)



extern QueueHandle_t      QueueHandle;
extern ControlCmd_s controlCmds;

extern StateBackToBox_s   stateBackToBoxs;

#endif
