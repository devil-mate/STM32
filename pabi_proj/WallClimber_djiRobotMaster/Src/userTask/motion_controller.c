//#include "motor.h"
#include "motion_controller.h"
#include "cmsis_os.h"
//#include "platform.h"
#include "protocol_type.h"
#include "stm32f4xx_hal_iwdg.h"
#include "can_platform_c610Motor.h"
#include "pid.h"
//#define TEST_DEBUG 1
#define leftForwardLimit 	limitFlag[0]
#define rightForwardLimit 	limitFlag[1]
#define rightBackwardLimit 	limitFlag[2]
#define leftBackwardLimit   limitFlag[3]

#define leftForwardState 	ultraStateFlag[0]
#define rightForwardState	ultraStateFlag[1]
#define rightBackwardState 	ultraStateFlag[2]
#define leftBackwardState   ultraStateFlag[3]

#define testAutoSpeed  (10.0f)

WheelRPM_s wheelRpmTarget_g = {0.0, 0.0};
WheelTorque_s wheelTorqueS ={0,0};
SemaphoreHandle_t wheelRpmMutex;
SemaphoreHandle_t movCmdMutex;

//////////函数
static void motionControllerTask(void *argument);
static TaskHandle_t MotionControllerTaskHandle = NULL;

//static int16_t getMotorCurrent(MotorIndex motorId);

static void copyGlobal2Local();
//static void receiveCmdMessage(Queue_Message_s *message);
//static void resolveWheelRpm(void);
static void resolveWheelRpm(float speed,float omega);
static void getControlDigit(ControlDigit_s *ControlDigit);
static void manualOperation();
static void autoOperation();
static void delayFeedBack();

static float pidController(float target, float feedbackData);
//static void TorquesResolve(float torque);
//int getMode(uint8_t* mode,int8_t *start);
//float autoSpeedControler(float speed);
float manualSpeedControler(float speed);
float torqueController(float targetSpeed);


//////////变量

extern MoveCmd_s           	MoveCmd;
extern PidParam_s       	PidParamCmd;
extern Flag_s 				sFlag;

extern pid_type_def sMotorVolecityPID[];
static StateFeed_promptly_s     LocalRobotStateFeed; //立即返回
static RobotStateFrame_s		LocalRobotStateFrame; //延时返回
static MoveCmd_s           		LocalMoveCmd;

static ParamConfig_s       	LocalParamConfig;
static Flag_s 				localFlag;



//static Queue_Message_s localQueueMessage;
extern IWDG_HandleTypeDef hiwdg;

//本地全局变量
static ControlDigit_s sControlDigit;

static int taskCount=0;
static int8_t currentState;
static int8_t 	autoState;

static bool gLocal_startFlag=false;
static bool gLocal_brokenFlag=false; //异常结束
bool initSpeedFlag=false;

//float gOutTorque=0.0;
OnceTaskInfo_s onceTaskInfo_s;
//float Kp=0.01;
static _PID speedPid;
MovAvgFilter_s speedFilter;
bool g_motorEnFlag=0;
bool oneTaskFlag=false;
enum StateMode_e{
	MANUAL_MODE 	= 1,
	AUTO_MODE		= 2,
	RESET_MODE		= 3,
};
//localFlag使用

//ModeStateFSM_e eModeState;
//AutoStateFSM_e eAutoState;
//void feedback_mode();
void initMotionController()
{
	memset(&localFlag,0,sizeof(localFlag));
	memset(&LocalMoveCmd,0,sizeof(LocalMoveCmd));
	memset(&LocalParamConfig,0,sizeof(LocalParamConfig));
	
	memset(&LocalRobotStateFeed,0,sizeof(LocalRobotStateFeed));
	memset(&LocalRobotStateFrame,0,sizeof(LocalRobotStateFrame));
	//互斥
	wheelRpmMutex = xSemaphoreCreateMutex();
	movCmdMutex  = xSemaphoreCreateMutex();
	
	xSemaphoreGive( wheelRpmMutex ); 
	xSemaphoreGive(movCmdMutex);
    xTaskCreate(motionControllerTask,         /* 任务函数  */
                "MotionControllerTask",       /* 任务名    */
                256,                         /* 任务栈大小，单位word，也就是4字节 */
                NULL,                         /* 任务参数  */
                12,                            /* 任务优先级*/
                &MotionControllerTaskHandle); /* 任务句柄  */
}

static void motionControllerTask(void *argument)
{	
	
    bool reseted = false;

	InitProtocolFrame(LocalRobotStateFeed);
	InitProtocolFrame(LocalRobotStateFrame);
	LocalRobotStateFeed.funcCode = Func_StateFeed;
	LocalRobotStateFrame.funcCode = Func_StateFrame;
	
	//disenableAllMotor();
	pidInit(&speedPid);
	movAvgFilterInit(&speedFilter);
	initSpeedFlag=true;
	LocalParamConfig.taskTotal=1;
	currentState=stopState;
	autoState=linearState;
	float tempOutTargetTorque=0.0;
	bool motorEnableFlag=false;
	/***********test**********/
	currentState=manulModeState;
//	LocalMoveCmd.speed = 15;
//	LocalMoveCmd.omega = 0.2;
//	start=1;
//	sControlDigit.ControlMode=0x01;
//	sControlDigit.Start=1;
	//currentState=autoModeState;
	//TODO使能电机不成功处理
	/***********test**********/
	g_motorEnFlag=true;
    while (1)
    {	
		//HAL_IWDG_Refresh(&hiwdg);
		osDelay(20);
		copyGlobal2Local();
		getControlDigit(&sControlDigit);
		
        if (currentState != resetState && reseted){
            reseted = false;
        }

//		if(sControlDigit.Stop){
//			currentState=stopState;
//		}
        switch (currentState)
        {
			default:
				//printf("have no this controlMode");
				currentState =manulModeState;
				//currentState=stopState;
				break;
//			//手动模式0x01
			case manulModeState:
				manualOperation();
//				//3 复位，2自动，1手动，自动时不直接进入自动模式，先进入停止状态，根据启动命令 进入自动模式的前进后退状态
//				if(sControlDigit.ControlMode==RESET_MODE){
//					currentState=resetState;
//				}
//				if(sControlDigit.ControlMode==AUTO_MODE){
//					currentState=stopState;
//				}
				break;
//			
//			//自动模式0x02
			case autoModeState:
				autoOperation();
				if(sControlDigit.ControlMode==RESET_MODE){
					currentState=resetState;
				}
				if(sControlDigit.ControlMode==MANUAL_MODE){
					currentState=stopState;
				}			
				break;
////			//停止模式0x04
			case stopState:
				resolveWheelRpm(0,0); //速度，角速为0
				gLocal_startFlag =false;
				gLocal_brokenFlag =false;
				taskCount=0;
				localFlag.runningtFlag=false;
				//autoState=forwardState;
				if(sControlDigit.ControlMode==MANUAL_MODE){
					currentState=manulModeState;
					
				}
				else if(sControlDigit.ControlMode==AUTO_MODE){
					//停止模式中判断，如果在自动挡，判断进入自动的前进还是后退;或者，前超声波正常才进入自动模式
//					if(sControlDigit.Start_Forward && motorEnableFlag && sControlDigit.createFailFlag_cmd ){
//						//speedPid.mTi = 0;
//						//TODO反馈不能启动原因
//						if(!(localFlag.fowardLimit || localFlag.laserStateFlagF || localFlag.voltageFlag )){
//							currentState=autoModeState;
//						}	
//					}
					currentState=autoModeState;
					autoState=linearState;
				}
				else if(sControlDigit.ControlMode==RESET_MODE){
					currentState=resetState;
					
				}
				//检测到障碍物，同时又断线(中途误检测后极限)
				//停止模式下，断线，失能
				if(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect){
					
				}
				
				break;
//		//			//复位0x03
//			case resetState:
//				gLocal_brokenFlag =false;
//				gLocal_startFlag =false;
//				if(sControlDigit.ControlMode==AUTO_MODE ){
//					currentState=stopState;
//				}
//				if(sControlDigit.ControlMode==MANUAL_MODE){
//					if(motorEnableFlag){
//						currentState=manulModeState;
//					}
//				}
//				//复位模式断线
//				if(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect){
//						TorquesResolve(0);//力矩为0
//						//disenableAllMotor();
//				}
//				if (!reseted){	
//					//taskENTER_CRITICAL();					
//					memset(&MoveCmd, 0, sizeof(MoveCmd));
//					memset(&sFlag,0,sizeof(sFlag));
//					memset(&onceTaskInfo_s,0,sizeof(onceTaskInfo_s));
//					if(!localFlag.motorFlags.nmt_startNodeFlag){
//						//没有正常收到PDO数据
//						printf("PDO data back error\r\n");
//						continue;
//					}		
//					if(!localFlag.motorFlags.heartbeatSendFlag){
//						//没有正常收到PDO数据
//						printf("heartbeatSendFlag  error\r\n");
//						continue;
//					}
//					//taskEXIT_CRITICAL();
//					//resetFaultAllMotor();
//					//resetAllMotor();
//					//readMotorState();
//					log_info("============================Reset all motor...\r\n");					

//					osDelay(300);				
//					taskENTER_CRITICAL();
//					bool enable01 = sFlag.motorFlags.enableFlag[0];
//					bool enable02 = sFlag.motorFlags.enableFlag[1];
//					taskEXIT_CRITICAL();
//					motorEnableFlag = enable01 & enable02;
//					if(motorEnableFlag){
//						reseted = true;
//						autoState=forwardState;
//						speedPid.mTi = 0;  //积分置0
//					}else{
//						osDelay(500);
//					}
//					
//				}
//				break;

        }
		static uint8_t tcount=0;
		tcount++;
		if(tcount%10==0){
			delayFeedBack();
			tcount=0;
		}

	}		
}
static void manualOperation()
{	
	static float tempManualSpeed=0,tempOmega=0;
	float OutTargetTorque=0.0;
	static float lastSpeed =0;
	//stm32通信串口断线，controlBox_dis控制箱断线
	//|| sControlDigit.controlBox_disconnect
	if(localFlag.disconnetFlag ){
		tempManualSpeed = tempOmega=0;
		//TODO 力矩为0（pid输出为0）
	}
	else{
		if(cABS(LocalMoveCmd.speed)<=0.1){
			tempManualSpeed=0; //没有手动指令后,停在原处，断线后才自由下滑
		}
		else{
		//	tempManualSpeed=-manualSpeedControler(LocalMoveCmd.speed);	
			tempManualSpeed=LocalMoveCmd.speed; //向前向后速度由命令给定
		}
		if(cABS(LocalMoveCmd.omega)<=0.05){
			tempOmega =0;
		}
		else{
			tempOmega=LocalMoveCmd.omega; 
		}

	}
	printf("target speed: [%3.2f],  omega[%3.2f]\r\n",tempManualSpeed,tempOmega);
	resolveWheelRpm(tempManualSpeed,tempOmega);
	RobotStateFrame.sMotionState.manulTargetSpeed = -tempManualSpeed;
}
static void autoOperation()
{
	float tempAutoSpeed=0.0,tempOmega=0;
	float OutTargetTorque=0.0;
	switch(autoState){	
	default:
		currentState=stopState;
		printf("error--\r\n");
		return;
	case linearState:
		gLocal_startFlag =true;
	
		if(localFlag.leftForwardLimit || localFlag.leftForwardState || localFlag.rightForwardLimit || localFlag.rightForwardState ||
			localFlag.disconnetFlag || sControlDigit.controlBox_disconnect  || localFlag.voltageFlag){
			currentState=stopState;
			tempAutoSpeed=0;
		}
		//TODO 一条直线任务任务	
		if(localFlag.leftForwardLimit || localFlag.leftForwardState || localFlag.disconnetFlag || 
			sControlDigit.controlBox_disconnect || sControlDigit.Stop_Backward || localFlag.voltageFlag){
			
			//autoState=rotateState;
			
		}
		else{
			tempAutoSpeed = cABS(LocalMoveCmd.speed);//相机朝前-；相机朝后+
			localFlag.autoStopFlag=0;
			localFlag.runningtFlag=1;
		}
		
//		if(localFlag.laserStateFlagF || localFlag.disconnetFlag || 
//			sControlDigit.controlBox_disconnect || sControlDigit.Stop_Backward ){
//				taskCount=LocalParamConfig.taskTotal+1;
//				gLocal_brokenFlag = true;
//		}
		break;
	case rotateState:
//			if( !(localFlag.backwardLimit) || localFlag.laserStateFlagB){
//				//tempAutoSpeed = cABS(LocalMoveCmd.autoSpeed);
//			}
//			else{
//				//如果没有掉线，判断进入stopState；如果掉线，继续退回
//				if(!(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect)){
//					taskCount++;
//					/*****feedback*** temp**/
//					//LocalRobotStateFrame.posture.picth=taskCount;
//					/*****feedback*** temp**/
//					if(taskCount>=LocalParamConfig.taskTotal ){
//						memset(&onceTaskInfo_s,0,sizeof(onceTaskInfo_s));
//						//任务结束，需要重新复位创建文件夹。
//						sControlDigit.createFailFlag_cmd=false;
//						tempAutoSpeed = 0;
//						currentState=stopState;
//						autoState=forwardState;
//						localFlag.autoStopFlag=1;
//						localFlag.runningtFlag=0;
//						taskCount=0;
//				// todo ----任务结束标志位上传
//					}else{
//						speedPid.mTi=0;
//						tempAutoSpeed = 0;
//						autoState=forwardState;
//					}
//					

//				}
//				else{
//					// 检测到障碍物,如果掉线，继续退回，即保持tempAutoSpeed，但是会有碰撞问题。
//					//TODO，退回碰撞问题解决,计时？
//					taskCount=LocalParamConfig.taskTotal+1;
//					localFlag.autoStopFlag=1;
//					localFlag.runningtFlag=0;
//				}

//			}
//			
////			if(localFlag.laserStateFlagB){
////				//TODO报警
////			}
	
		break;
	}

	resolveWheelRpm(tempAutoSpeed,tempOmega);
	/*****feedback*** 给定目标速度temp**/
	RobotStateFrame.sMotionState.autoTargetSpeed=-tempAutoSpeed;
	/*****feedback*** 给定目标速度temptemp**/
}
static void copyGlobal2Local()
{
			//PID参数
	static PidParam_s pidParamL;
	static PidParam_s pidParamR;
	
	xSemaphoreTake(movCmdMutex,portMAX_DELAY);
    memcpy(&LocalMoveCmd, &MoveCmd, sizeof(MoveCmd));
	memcpy(&localFlag, &sFlag, sizeof(sFlag));
	memcpy(&LocalParamConfig, &ParamConfigs, sizeof(ParamConfigs));
	memcpy (&pidParamL,&PidParamCmd,sizeof(pidParamL));
	memcpy (&pidParamR,&PidParamCmd,sizeof(pidParamR));
	xSemaphoreGive(movCmdMutex);
	
	PID_setParam(&sMotorVolecityPID[MotorL],&pidParamL);
	PID_setParam(&sMotorVolecityPID[MotorR],&pidParamR);
	//memcpy(&LocalRobotStateFeed, &RobotStateFeed, sizeof(RobotStateFeed));
	LocalRobotStateFeed.digit = StateFeed_promptlys.digit;
	LocalRobotStateFrame.digit = RobotStateFrame.digit;
    
	
}

//static void receiveCmdMessage(Queue_Message_s *message)
//{

//	if(xQueueReceive(QueueHandle,message,30)==pdPASS){
//		getControlDigit(&sControlDigit);
//	}
//	
//	
//}


////linux传入数据爬索最大20m/min
//static void resolveWheelRpm() 
//{
//    LocalMoveCmd.speed = absLimiter(LocalMoveCmd.speed, PLATFORM_MAX_SPEED_M_PER_MIN);
//	wheelRpmCmd.RightRPM  = (LocalMoveCmd.speed * 1000 ) / PI / PLATFORM_WHEEL_DIA ;//角速度单位rpm
//	wheelRpmCmd.LeftRPM  = -(LocalMoveCmd.speed * 1000 ) / PI / PLATFORM_WHEEL_DIA;

//}
///两轮差动模型解算
//爬壁38m/min
static void resolveWheelRpm(float speed,float omega)
{
	static float tempSpeed, tempOmega,rpmWheel[2];
    tempSpeed= absLimiter(speed, PLATFORM_MAX_SPEED_M_PER_MIN);
    tempOmega = absLimiter(omega, PLATFORM_MAX_OMEGA);
	
	//轮子转速rpm
	rpmWheel[1] = (tempSpeed * 1000 / 60 + PLATFORM_WHEEL_LEN / 2 * tempOmega) / PI / PLATFORM_WHEEL_DIA * 60;  //右轮
    rpmWheel[0]  = -(tempSpeed * 1000 / 60 - PLATFORM_WHEEL_LEN / 2 * tempOmega) / PI / PLATFORM_WHEEL_DIA * 60; //取负数，方向相反
    //motor转速
	for(uint8_t i=0;i<2;i++){
		rpmWheel[i]= rpmWheel[i]*PLATFORM_GEAR_RATIO;
		rpmWheel[i] = absLimiter(rpmWheel[i],MOTOR_MAX_RPM);
	}
	xSemaphoreTake( wheelRpmMutex, portMAX_DELAY );
	wheelRpmTarget_g.LeftRPM = rpmWheel[0];
	wheelRpmTarget_g.RightRPM = rpmWheel[1];
//	printf("targetLrpm: %3.2f, Rrpm:%3.2f\r\n",
//		wheelRpmTarget_g.LeftRPM,wheelRpmTarget_g.RightRPM);
	xSemaphoreGive( wheelRpmMutex ); 
}



//static int16_t getMotorCurrent(MotorIndex motorId)
//{
////	int16_t tempCurrent;
////	tempCurrent = (int16_t)LocalRobotStateFrame.sMotorState.motorCurrent[MotorL];//反馈的电流
////	 if(cABS(tempCurrent)<50){
////	 	tempCurrent=0;
////	 }
////	 
////	 tempCurrent = avgFilter(tempCurrent, 10, cABS(tempCurrent)<50);
////	// if(tempCurrent>5000){
////	// 	tempCurrent=tempCurrent-65535;
//////	// }
//////	printf("tempCurrent:========================%d \r\n",tempCurrent);
////	return tempCurrent;
//	
//}
//int getMode(uint8_t* mode,int8_t *start)
//{

//	
//	int i=0;
//	*mode=0;
//	//手自动复位模式,第1位手动，2自动，3复位
//	for(i=1;i<5;i++){
//		if (getbit(LocalMoveCmd.digit,i)){
//			*mode=i;		
//		}
//	}
//	//第一位数据启停
//	if(getbit(LocalMoveCmd.digit,0)){
//		*start=1;
//	}
//	else{
//		*start=0;
//	}
//	return 0;
//}
void getControlDigit(ControlDigit_s *ControlDigit)
{	
	
	//printf("LocalMoveCmd.digit: %d \r\n",LocalMoveCmd.digit);
	//手动启动/使能？
//	if(getbit(LocalMoveCmd.digit,0)){
//		ControlDigit->Start=1;
//	}
//	else{
//		ControlDigit->Start=0;
//	}
	//123 手动/自动/复位
	for(int i=1;i<4;i++){
		if (getbit(LocalMoveCmd.digit,i)){
			ControlDigit->ControlMode =i;		
		}
	}
	//第4位，错误确认
//	if(getbit(LocalMoveCmd.digit,4)){
//		ControlDigit->errorVerify =1;
//	}
//	else{
//		ControlDigit->errorVerify=0;
//	}
	//第5位，激光传感器极限位/断联确认
//	if(getbit(LocalMoveCmd.digit,5)){
//		ControlDigit->LimitVerify =1;
//	}
//	else{
//		ControlDigit->LimitVerify=0;
//	}
	//第6位，自动启动/手动前进
	if(getbit(LocalMoveCmd.digit,6)){
		ControlDigit->Start_Forward =true;
	}
	else{
		ControlDigit->Start_Forward=false;
	}

	//第7位，掉头/手动后退
	if(getbit(LocalMoveCmd.digit,7)){
		ControlDigit->Stop_Backward =true;
	}
	else{
		ControlDigit->Stop_Backward=false;
	}
	//第8位，手提箱掉线
	if(getbit(LocalMoveCmd.digit,8)){
		ControlDigit->controlBox_disconnect =true;
	}
	else{
		ControlDigit->controlBox_disconnect=false;
	}
	//第9位，急停
	if(getbit(LocalMoveCmd.digit,9)){
		ControlDigit->Emergency =true;
	}
	else{
		ControlDigit->Emergency=false;
	}
	//10停止
	if(getbit(LocalMoveCmd.digit,10)){
		ControlDigit->Stop =true;
	}
	else{
		ControlDigit->Stop=false;
	}
	//11 创建文件，成功后保持为true;
	if(getbit(LocalMoveCmd.digit,F_CREATE_FAIL_FLAG)){
		ControlDigit->createFailFlag_cmd =true;
		//oneTaskFlag=true;
	}
	else{
		//ControlDigit->createFailFlag_cmd=false;
	}
	if(getbit(LocalMoveCmd.digit,RECONNECT_REDIO_CMD)){
		ControlDigit->resetRedio =true;
	}
	else{
		ControlDigit->resetRedio=false;
	}
	
}







static void delayFeedBack()
{
	//TODO===读取电机状态
			//readMotorState();
			//反馈 手自动复位模式

			if(currentState==manulModeState){
				setbit(LocalRobotStateFrame.digit,0);
				resetbit(LocalRobotStateFrame.digit,1);
				resetbit(LocalRobotStateFrame.digit,2);
				resetbit(LocalRobotStateFrame.digit,3);
				resetbit(LocalRobotStateFrame.digit,4);
			}

			else if(currentState==autoModeState){
				setbit(LocalRobotStateFrame.digit,1);
				resetbit(LocalRobotStateFrame.digit,0);
				resetbit(LocalRobotStateFrame.digit,2);
				resetbit(LocalRobotStateFrame.digit,3);
				resetbit(LocalRobotStateFrame.digit,4);
			}
			else if(currentState==resetState){
				setbit(LocalRobotStateFrame.digit,2);
				resetbit(LocalRobotStateFrame.digit,0);
				resetbit(LocalRobotStateFrame.digit,1);
				resetbit(LocalRobotStateFrame.digit,3);
				resetbit(LocalRobotStateFrame.digit,4);
			}
			else if(currentState==stopState){
				setbit(LocalRobotStateFrame.digit,3);
				resetbit(LocalRobotStateFrame.digit,0);
				resetbit(LocalRobotStateFrame.digit,2);
				resetbit(LocalRobotStateFrame.digit,1);
				resetbit(LocalRobotStateFrame.digit,4);
			}
			else if(currentState==errorState){
				setbit(LocalRobotStateFrame.digit,4);
				resetbit(LocalRobotStateFrame.digit,0);
				resetbit(LocalRobotStateFrame.digit,2);
				resetbit(LocalRobotStateFrame.digit,3);
				resetbit(LocalRobotStateFrame.digit,1);
			}
			
			//LocalRobotStateFeed
			if(localFlag.leftForwardState){
				setbit(LocalRobotStateFeed.digit,0); // 0,1 前后激光/距离传感器状态
			}
			else{
				resetbit(LocalRobotStateFeed.digit,0); 
			}
			if(localFlag.leftBackwardState){
				setbit(LocalRobotStateFeed.digit,1); // 0,1 前后激光/距离传感器状态
			}
			else{
				resetbit(LocalRobotStateFeed.digit,1); 
			}
			if(currentState==autoModeState && autoState==linearState){
				setbit(LocalRobotStateFeed.digit,3); //前进中
				resetbit(LocalRobotStateFeed.digit,4);
			}
			else if(currentState==autoModeState && autoState==rotateState){
				setbit(LocalRobotStateFeed.digit,4); //旋转中
				resetbit(LocalRobotStateFeed.digit,3);
			}
			else{
				resetbit(LocalRobotStateFeed.digit,3);
				resetbit(LocalRobotStateFeed.digit,4);
			}
			//TODO ----
			if(localFlag.disconnetFlag ||localFlag.motorFlags.motorErrorFlag[0] ||localFlag.motorFlags.disconnectFlag[0]
				|| localFlag.voltageFlag || (gLocal_startFlag && gLocal_brokenFlag)){
				setbit(LocalRobotStateFeed.digit,5); //stm32运行异常/报错
			}
			else{
				resetbit(LocalRobotStateFeed.digit,5);
			}
			if (gLocal_startFlag && gLocal_brokenFlag){
				setbit(LocalRobotStateFeed.digit,6); //任务异常结束
			}
			else{
				resetbit(LocalRobotStateFeed.digit,6); 
			}
			//7 电机使能，motor.c中
			if (localFlag.voltageFlag){
				setbit(LocalRobotStateFeed.digit,8); //电池电压
			}
			else{
				resetbit(LocalRobotStateFeed.digit,8); 
			}
			
			if(gLocal_startFlag && !(gLocal_brokenFlag)){
				setbit(LocalRobotStateFeed.digit,9); //任务开始/自动运行中
			}
			else{
				resetbit(LocalRobotStateFeed.digit,9); //任务结束
			}
			//10自动触发信号，coder.c
			//11 关机信号 getControlDigit()
			//备用
			if(localFlag.disconnetFlag){
				setbit(LocalRobotStateFeed.digit,12); //任务开始/自动运行中
			}
			else{
				resetbit(LocalRobotStateFeed.digit,12); //任务结束
			}
			resetbit(LocalRobotStateFeed.digit,2);
//			printf("localFlag.motorFlags.enableFlag[0]: %d,localFlag.motorFlags.enableFlag[1]: %d\r\n",
//					localFlag.motorFlags.enableFlag[0],localFlag.motorFlags.enableFlag[1]);

			
			
			taskENTER_CRITICAL();
			StateFeed_promptlys.digit = LocalRobotStateFeed.digit;
			RobotStateFrame.digit = LocalRobotStateFrame.digit;
			//memcpy(&RobotStateFeed, &LocalRobotStateFeed, sizeof(LocalRobotStateFeed));
			//memcpy(&RobotStateFrame, &LocalRobotStateFrame, sizeof(LocalRobotStateFrame));
			taskEXIT_CRITICAL();
			//printf("local to RobotStateFeed.digit:%d\r\n",RobotStateFeed.digit);

		/**************feedback*****************/
	
}

static float pidController(float target,float feedBackData)
{

	//#if (TEST_DEBUG==1)
	//TODO会一直发送

	/////////////////

	float err=0;
	err=target-feedBackData;
	float daltaSpeed;
	/*********测试用**********/	
	float tempKp,tempTi,tempDt; //比例项、积分项、微分项
	float tempPidOut=0,pidOutData=0;
//	tempKp=speedPid.kp*(speedPid.err-speedPid.err_next);
//	tempTi=speedPid.ki*speedPid.err;
//	tempDt=speedPid.kd*(speedPid.err-2*speedPid.err_next+speedPid.err_last );
//	daltaSpeed=tempKp+tempKi+tempKd;

	//增量式PID，加初始阈值
	//daltaSpeed=speedPid.kp*(speedPid.err-speedPid.err_next) + speedPid.ki*speedPid.err + speedPid.kd*(speedPid.err-2*speedPid.err_next+speedPid.err_last );
	//speedPid.outData+=daltaSpeed;

	
	speedPid.mTi+=speedPid.ki*err;
	speedPid.mTi = absLimiter(speedPid.mTi,speedPid.threshold);
	tempDt=speedPid.kd*(err-speedPid.err_last);
//	//TODO=========?????位置式
	tempPidOut=speedPid.kp*err +speedPid.mTi+tempDt;
	//printf("speedPid.outData%3.2f \r\n",speedPid.outData);
	
	//变相位置式PID，响应速度？
//	daltaSpeed=speedPid.kp*(speedPid.err) +speedPid.mTi;
//	speedPid.outData+=daltaSpeed;
	
	speedPid.err_last =err;	
	pidOutData=absLimiter(tempPidOut,speedPid.threshold);
	
//	printf("targetData%3.2f,feedBackData%3.2f,err%3.2f,outData%3.2f,---------feedback speed0%3.2f feedback speed1%3.2f\r\n",
//		speedPid.targetData,speedPid.feedBackData,speedPid.err,speedPid.outData,
//		LocalRobotStateFrame.motionState.wheelRPM[0],LocalRobotStateFrame.motionState.wheelRPM[1]);
	return pidOutData;
	
}
void pidInit(_PID* pid)
{	
//	pid->targetData=0.0;
//	pid->feedBackData =0.0;
//	pid->err=0.0;
	pid->err_last=0.0;
	pid->err_next=0.0;
	pid->kp=0.6;
	//TODO------获取初始参数，一直发？
	pid->ki=0.0;
	pid->kd=0.0;
	pid->IGate = 3000;
	pid->threshold =6000;
	pid->mTi=0.0;
}
