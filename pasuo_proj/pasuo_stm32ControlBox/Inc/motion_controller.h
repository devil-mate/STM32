#ifndef __MOTION_CONTROLLER_H
#define __MOTION_CONTROLLER_H

#include <string.h>
#include "common.h"

void initMotionController(void);
typedef  struct{
//	float targetData;
//	float feedBackData;
//	float err;
	float kp,ki,kd;
	float err_next;  //ek-2 绝对式PID
	float err_last; //ek-1
	//float outData;
	float threshold;
	float mTi; // 积分项
} _PID ;

typedef struct{
	int8_t ControlMode;
	bool Start;
	bool Enable;
	bool OnceTask;
	bool LimitVerify;
	bool errorVerify;
	bool autoStart;
	bool autoStop;
	bool controlBox_dis;
	bool backControl;
	
}ControlDigit_s;


typedef enum {
	manulModeState=0x01,
	autoModeState,
	resetState,
	stopState,
	errorState,
	
}ModeStateFSM_e;

enum {
	judgeState,
	forwardState,
	backwardState,
	pauseState,	
	
};


void pidInit(_PID* pid);
float pidController(float target, float feedbackData);
#endif // !__MOTION_CONTROLLER_H
