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
	FUnc_ParamConfig	= 0xC2,
    //Func_VideoCamCmd    = 0xC2,
    Func_LidarInfo      = 0xC3,
    Func_PidParamCmd    = 0xC4,
	

    /*Feedback*/
    Func_RobotPosture   = 0xF1,
    Func_StateFeed     = 0xF2,
	Func_StateFrame		= 0xF3,
} FuncCode_e;   //功能码(控制字)

/*上位机下发*/
typedef struct
{
    float posX;
} Posture_s;    //机器人位姿，含X Y坐标和航向角

typedef struct
{
    uint16_t     digit; //数字标志位，
    float       speed;	//手动速度
	float       autoSpeed;//自动速度
    Posture_s   posture;
} MoveCmd_s; //机器人运动指令

typedef struct
{
    uint8_t digit;      //数字标志位,bit0=1全部回零，bit0=0正常控制
    float pitchAcc;     //俯仰角变化率,单位deg/s
    float yawAcc;       //偏航角变化率,单位deg/s
} VideoCamCmd_s;        //视频相机运动指令

typedef struct
{
    uint8_t digit; //数字标志位
} LidarInfo_s;     //激光雷达数据帧

typedef struct
{
	float Kp;
	float Ki;
    float Kd;
	float IGate;    //积分限制
	float OutGate;  //总输出限制
} PidParamCmd_s;    //下发PID参数

typedef struct
{
	float obstacleDis[2]; //
	uint16_t taskTotal;
	float picDis;
} ParamConfig_s;    //


/*下位机反馈*/

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
	float manulTargetSpeed;//手动 目标速度 
	float autoTargetSpeed;//自动时 目标速度
	float feedbackSpeed;//实际速度
}MotionState_s;

typedef struct 
{
    float wheelRPM[2];//左右轮实际速度
    float motorCurrent[2];//左右轮实际力矩
}MotorState_s;


typedef struct
{
    uint16_t            head;
    uint8_t             funcCode;

    Posture_s           posture;

    uint8_t             checkSum;
    uint16_t            tail;
} RobotPostureFrame_s; //机器人位姿信息反馈

// 
typedef struct
{
    uint16_t            head;
    uint8_t             funcCode;

    uint8_t            digit;
    uint16_t           motorState;//电机控制字
    ElecInfo_s          elecInfo;
    TemperatureInfo_s   temperature;   
    MotionState_s       sMotionState; //运动状态
    MotorState_s        sMotorState; //电机状态
	
    uint8_t             checkSum;
    uint16_t            tail;
} RobotStateFrame_s; //机器人状态信息可延后反馈

/////////////////////////////////////////

typedef struct
{
	float obstacleDis[2];//前后障碍物距离（激光与超声融合后的实际距离）

}ObstacleDis_s;

typedef struct
{
	float odom;//里程计信息
	float time;//时间
	//bool picTrigger;//触发信号（digit,10）
}picOdom_s;

// 2+8+8 + 6 = 24
typedef struct
{
    uint16_t            head;
    uint8_t             funcCode;
	
    uint16_t             digit;      //数字标志位
	ObstacleDis_s 		sObstacleDis;
    picOdom_s       	sPicOdom;
	//float				realOdom;
	
    uint8_t             checkSum;
    uint16_t            tail;
} RobotStateFeed_s; //机器人状态信息立即反馈




#pragma pack(pop)


extern ParamConfig_s		ParamConfigs;
extern QueueHandle_t      QueueHandle;


extern RobotStateFrame_s   RobotStateFrame;
extern RobotStateFeed_s   RobotStateFeed;

#endif
