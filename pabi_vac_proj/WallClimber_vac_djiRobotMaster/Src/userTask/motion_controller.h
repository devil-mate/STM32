#ifndef __MOTION_CONTROLLER_H
#define __MOTION_CONTROLLER_H

#include <string.h>
#include "common.h"

void initMotionController(void);
#pragma pack(push, 1)
typedef  struct{
//	float targetData;
//	float feedBackData;
//	float err;
	float kp,ki,kd;
	float err_next;  //ek-2 绝对式PID
	float err_last; //ek-1
	//float outData;
	float IGate; //积分限制
	float threshold; //输出限制
	float mTi; // 积分项
} _PID ;
//与上位机 中保持一致
enum Control_Cmd_Digit{
    UN_DEFINE   = 0,
    MAN_CMD     =1,
    AUTO_CMD    =2,
    RESET_CMD   =3,
    ERR_DECIDED =4,
    ERR_FLAG    =5,
    FORWARD_CMD =6,
    BACKWARD_CMD    =7,
    CONTROL_BOX_OFF =8,
    EMERGENCY_CMD   =9,
    STOP_CMD    =10,
    F_CREATE_FAIL_FLAG   =11,
    CAMERA_STATE_ERR_FLAG =12,
    SAVE_PIC_ERR_FLAG   =13,
	RECONNECT_REDIO_CMD =14
};

typedef struct{
	int8_t ControlMode;
	bool Stop;
	bool Enable;
	bool OnceTask;
	bool LimitVerify;
	bool errorVerify;
	bool Start_Forward;
	bool Stop_Backward;
	bool controlBox_disconnect;
	bool Emergency;
	bool createFailFlag_cmd;
	bool resetRedio;
		
	
	
}ControlDigit_s;//解析得到上位机命令

typedef struct{
	uint32_t 	picNum;
	uint32_t	picOdomNum;
}OnceTaskInfo_s;

typedef enum {
	manulModeState=0x01,
	autoModeState,
	resetState,
	stopState,
	errorState,
	
}ModeStateFSM_e;

//enum {
////	judgeState,
//	forwardState,
//	backwardState,
//	pauseState,	
//	
//};
enum {
//	judgeState,
	linearState, //直线运动
	rotateState, //旋转运动
	
};


#pragma pack(pop)
extern OnceTaskInfo_s onceTaskInfo_s;
void pidInit(_PID* pid);

#endif // !__MOTION_CONTROLLER_H
