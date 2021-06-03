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

//////////����
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


//////////����

extern MoveCmd_s           	MoveCmd;
extern PidParam_s       	PidParamCmd;
extern Flag_s 				sFlag;

extern pid_type_def sMotorVolecityPID[];
static StateFeed_promptly_s     LocalRobotStateFeed; //��������
static RobotStateFrame_s		LocalRobotStateFrame; //��ʱ����
static MoveCmd_s           		LocalMoveCmd;

static ParamConfig_s       	LocalParamConfig;
static Flag_s 				localFlag;



//static Queue_Message_s localQueueMessage;
extern IWDG_HandleTypeDef hiwdg;

//����ȫ�ֱ���
static ControlDigit_s sControlDigit;

static int taskCount=0;
static int8_t currentState;
static int8_t 	autoState;

static bool gLocal_startFlag=false;
static bool gLocal_brokenFlag=false; //�쳣����
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
//localFlagʹ��

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
	//����
	wheelRpmMutex = xSemaphoreCreateMutex();
	movCmdMutex  = xSemaphoreCreateMutex();
	
	xSemaphoreGive( wheelRpmMutex ); 
	xSemaphoreGive(movCmdMutex);
    xTaskCreate(motionControllerTask,         /* ������  */
                "MotionControllerTask",       /* ������    */
                256,                         /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                         /* �������  */
                12,                            /* �������ȼ�*/
                &MotionControllerTaskHandle); /* ������  */
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
	//TODOʹ�ܵ�����ɹ�����
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
//			//�ֶ�ģʽ0x01
			case manulModeState:
				manualOperation();
//				//3 ��λ��2�Զ���1�ֶ����Զ�ʱ��ֱ�ӽ����Զ�ģʽ���Ƚ���ֹͣ״̬�������������� �����Զ�ģʽ��ǰ������״̬
//				if(sControlDigit.ControlMode==RESET_MODE){
//					currentState=resetState;
//				}
//				if(sControlDigit.ControlMode==AUTO_MODE){
//					currentState=stopState;
//				}
				break;
//			
//			//�Զ�ģʽ0x02
			case autoModeState:
				autoOperation();
				if(sControlDigit.ControlMode==RESET_MODE){
					currentState=resetState;
				}
				if(sControlDigit.ControlMode==MANUAL_MODE){
					currentState=stopState;
				}			
				break;
////			//ֹͣģʽ0x04
			case stopState:
				resolveWheelRpm(0,0); //�ٶȣ�����Ϊ0
				gLocal_startFlag =false;
				gLocal_brokenFlag =false;
				taskCount=0;
				localFlag.runningtFlag=false;
				//autoState=forwardState;
				if(sControlDigit.ControlMode==MANUAL_MODE){
					currentState=manulModeState;
					
				}
				else if(sControlDigit.ControlMode==AUTO_MODE){
					//ֹͣģʽ���жϣ�������Զ������жϽ����Զ���ǰ�����Ǻ���;���ߣ�ǰ�����������Ž����Զ�ģʽ
//					if(sControlDigit.Start_Forward && motorEnableFlag && sControlDigit.createFailFlag_cmd ){
//						//speedPid.mTi = 0;
//						//TODO������������ԭ��
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
				//��⵽�ϰ��ͬʱ�ֶ���(��;�������)
				//ֹͣģʽ�£����ߣ�ʧ��
				if(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect){
					
				}
				
				break;
//		//			//��λ0x03
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
//				//��λģʽ����
//				if(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect){
//						TorquesResolve(0);//����Ϊ0
//						//disenableAllMotor();
//				}
//				if (!reseted){	
//					//taskENTER_CRITICAL();					
//					memset(&MoveCmd, 0, sizeof(MoveCmd));
//					memset(&sFlag,0,sizeof(sFlag));
//					memset(&onceTaskInfo_s,0,sizeof(onceTaskInfo_s));
//					if(!localFlag.motorFlags.nmt_startNodeFlag){
//						//û�������յ�PDO����
//						printf("PDO data back error\r\n");
//						continue;
//					}		
//					if(!localFlag.motorFlags.heartbeatSendFlag){
//						//û�������յ�PDO����
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
//						speedPid.mTi = 0;  //������0
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
	//stm32ͨ�Ŵ��ڶ��ߣ�controlBox_dis���������
	//|| sControlDigit.controlBox_disconnect
	if(localFlag.disconnetFlag ){
		tempManualSpeed = tempOmega=0;
		//TODO ����Ϊ0��pid���Ϊ0��
	}
	else{
		if(cABS(LocalMoveCmd.speed)<=0.1){
			tempManualSpeed=0; //û���ֶ�ָ���,ͣ��ԭ�������ߺ�������»�
		}
		else{
		//	tempManualSpeed=-manualSpeedControler(LocalMoveCmd.speed);	
			tempManualSpeed=LocalMoveCmd.speed; //��ǰ����ٶ����������
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
		//TODO һ��ֱ����������	
		if(localFlag.leftForwardLimit || localFlag.leftForwardState || localFlag.disconnetFlag || 
			sControlDigit.controlBox_disconnect || sControlDigit.Stop_Backward || localFlag.voltageFlag){
			
			//autoState=rotateState;
			
		}
		else{
			tempAutoSpeed = cABS(LocalMoveCmd.speed);//�����ǰ-���������+
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
//				//���û�е��ߣ��жϽ���stopState��������ߣ������˻�
//				if(!(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect)){
//					taskCount++;
//					/*****feedback*** temp**/
//					//LocalRobotStateFrame.posture.picth=taskCount;
//					/*****feedback*** temp**/
//					if(taskCount>=LocalParamConfig.taskTotal ){
//						memset(&onceTaskInfo_s,0,sizeof(onceTaskInfo_s));
//						//�����������Ҫ���¸�λ�����ļ��С�
//						sControlDigit.createFailFlag_cmd=false;
//						tempAutoSpeed = 0;
//						currentState=stopState;
//						autoState=forwardState;
//						localFlag.autoStopFlag=1;
//						localFlag.runningtFlag=0;
//						taskCount=0;
//				// todo ----���������־λ�ϴ�
//					}else{
//						speedPid.mTi=0;
//						tempAutoSpeed = 0;
//						autoState=forwardState;
//					}
//					

//				}
//				else{
//					// ��⵽�ϰ���,������ߣ������˻أ�������tempAutoSpeed�����ǻ�����ײ���⡣
//					//TODO���˻���ײ������,��ʱ��
//					taskCount=LocalParamConfig.taskTotal+1;
//					localFlag.autoStopFlag=1;
//					localFlag.runningtFlag=0;
//				}

//			}
//			
////			if(localFlag.laserStateFlagB){
////				//TODO����
////			}
	
		break;
	}

	resolveWheelRpm(tempAutoSpeed,tempOmega);
	/*****feedback*** ����Ŀ���ٶ�temp**/
	RobotStateFrame.sMotionState.autoTargetSpeed=-tempAutoSpeed;
	/*****feedback*** ����Ŀ���ٶ�temptemp**/
}
static void copyGlobal2Local()
{
			//PID����
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


////linux���������������20m/min
//static void resolveWheelRpm() 
//{
//    LocalMoveCmd.speed = absLimiter(LocalMoveCmd.speed, PLATFORM_MAX_SPEED_M_PER_MIN);
//	wheelRpmCmd.RightRPM  = (LocalMoveCmd.speed * 1000 ) / PI / PLATFORM_WHEEL_DIA ;//���ٶȵ�λrpm
//	wheelRpmCmd.LeftRPM  = -(LocalMoveCmd.speed * 1000 ) / PI / PLATFORM_WHEEL_DIA;

//}
///���ֲģ�ͽ���
//����38m/min
static void resolveWheelRpm(float speed,float omega)
{
	static float tempSpeed, tempOmega,rpmWheel[2];
    tempSpeed= absLimiter(speed, PLATFORM_MAX_SPEED_M_PER_MIN);
    tempOmega = absLimiter(omega, PLATFORM_MAX_OMEGA);
	
	//����ת��rpm
	rpmWheel[1] = (tempSpeed * 1000 / 60 + PLATFORM_WHEEL_LEN / 2 * tempOmega) / PI / PLATFORM_WHEEL_DIA * 60;  //����
    rpmWheel[0]  = -(tempSpeed * 1000 / 60 - PLATFORM_WHEEL_LEN / 2 * tempOmega) / PI / PLATFORM_WHEEL_DIA * 60; //ȡ�����������෴
    //motorת��
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
////	tempCurrent = (int16_t)LocalRobotStateFrame.sMotorState.motorCurrent[MotorL];//�����ĵ���
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
//	//���Զ���λģʽ,��1λ�ֶ���2�Զ���3��λ
//	for(i=1;i<5;i++){
//		if (getbit(LocalMoveCmd.digit,i)){
//			*mode=i;		
//		}
//	}
//	//��һλ������ͣ
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
	//�ֶ�����/ʹ�ܣ�
//	if(getbit(LocalMoveCmd.digit,0)){
//		ControlDigit->Start=1;
//	}
//	else{
//		ControlDigit->Start=0;
//	}
	//123 �ֶ�/�Զ�/��λ
	for(int i=1;i<4;i++){
		if (getbit(LocalMoveCmd.digit,i)){
			ControlDigit->ControlMode =i;		
		}
	}
	//��4λ������ȷ��
//	if(getbit(LocalMoveCmd.digit,4)){
//		ControlDigit->errorVerify =1;
//	}
//	else{
//		ControlDigit->errorVerify=0;
//	}
	//��5λ�����⴫��������λ/����ȷ��
//	if(getbit(LocalMoveCmd.digit,5)){
//		ControlDigit->LimitVerify =1;
//	}
//	else{
//		ControlDigit->LimitVerify=0;
//	}
	//��6λ���Զ�����/�ֶ�ǰ��
	if(getbit(LocalMoveCmd.digit,6)){
		ControlDigit->Start_Forward =true;
	}
	else{
		ControlDigit->Start_Forward=false;
	}

	//��7λ����ͷ/�ֶ�����
	if(getbit(LocalMoveCmd.digit,7)){
		ControlDigit->Stop_Backward =true;
	}
	else{
		ControlDigit->Stop_Backward=false;
	}
	//��8λ�����������
	if(getbit(LocalMoveCmd.digit,8)){
		ControlDigit->controlBox_disconnect =true;
	}
	else{
		ControlDigit->controlBox_disconnect=false;
	}
	//��9λ����ͣ
	if(getbit(LocalMoveCmd.digit,9)){
		ControlDigit->Emergency =true;
	}
	else{
		ControlDigit->Emergency=false;
	}
	//10ֹͣ
	if(getbit(LocalMoveCmd.digit,10)){
		ControlDigit->Stop =true;
	}
	else{
		ControlDigit->Stop=false;
	}
	//11 �����ļ����ɹ��󱣳�Ϊtrue;
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
	//TODO===��ȡ���״̬
			//readMotorState();
			//���� ���Զ���λģʽ

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
				setbit(LocalRobotStateFeed.digit,0); // 0,1 ǰ�󼤹�/���봫����״̬
			}
			else{
				resetbit(LocalRobotStateFeed.digit,0); 
			}
			if(localFlag.leftBackwardState){
				setbit(LocalRobotStateFeed.digit,1); // 0,1 ǰ�󼤹�/���봫����״̬
			}
			else{
				resetbit(LocalRobotStateFeed.digit,1); 
			}
			if(currentState==autoModeState && autoState==linearState){
				setbit(LocalRobotStateFeed.digit,3); //ǰ����
				resetbit(LocalRobotStateFeed.digit,4);
			}
			else if(currentState==autoModeState && autoState==rotateState){
				setbit(LocalRobotStateFeed.digit,4); //��ת��
				resetbit(LocalRobotStateFeed.digit,3);
			}
			else{
				resetbit(LocalRobotStateFeed.digit,3);
				resetbit(LocalRobotStateFeed.digit,4);
			}
			//TODO ----
			if(localFlag.disconnetFlag ||localFlag.motorFlags.motorErrorFlag[0] ||localFlag.motorFlags.disconnectFlag[0]
				|| localFlag.voltageFlag || (gLocal_startFlag && gLocal_brokenFlag)){
				setbit(LocalRobotStateFeed.digit,5); //stm32�����쳣/����
			}
			else{
				resetbit(LocalRobotStateFeed.digit,5);
			}
			if (gLocal_startFlag && gLocal_brokenFlag){
				setbit(LocalRobotStateFeed.digit,6); //�����쳣����
			}
			else{
				resetbit(LocalRobotStateFeed.digit,6); 
			}
			//7 ���ʹ�ܣ�motor.c��
			if (localFlag.voltageFlag){
				setbit(LocalRobotStateFeed.digit,8); //��ص�ѹ
			}
			else{
				resetbit(LocalRobotStateFeed.digit,8); 
			}
			
			if(gLocal_startFlag && !(gLocal_brokenFlag)){
				setbit(LocalRobotStateFeed.digit,9); //����ʼ/�Զ�������
			}
			else{
				resetbit(LocalRobotStateFeed.digit,9); //�������
			}
			//10�Զ������źţ�coder.c
			//11 �ػ��ź� getControlDigit()
			//����
			if(localFlag.disconnetFlag){
				setbit(LocalRobotStateFeed.digit,12); //����ʼ/�Զ�������
			}
			else{
				resetbit(LocalRobotStateFeed.digit,12); //�������
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
	//TODO��һֱ����

	/////////////////

	float err=0;
	err=target-feedBackData;
	float daltaSpeed;
	/*********������**********/	
	float tempKp,tempTi,tempDt; //����������΢����
	float tempPidOut=0,pidOutData=0;
//	tempKp=speedPid.kp*(speedPid.err-speedPid.err_next);
//	tempTi=speedPid.ki*speedPid.err;
//	tempDt=speedPid.kd*(speedPid.err-2*speedPid.err_next+speedPid.err_last );
//	daltaSpeed=tempKp+tempKi+tempKd;

	//����ʽPID���ӳ�ʼ��ֵ
	//daltaSpeed=speedPid.kp*(speedPid.err-speedPid.err_next) + speedPid.ki*speedPid.err + speedPid.kd*(speedPid.err-2*speedPid.err_next+speedPid.err_last );
	//speedPid.outData+=daltaSpeed;

	
	speedPid.mTi+=speedPid.ki*err;
	speedPid.mTi = absLimiter(speedPid.mTi,speedPid.threshold);
	tempDt=speedPid.kd*(err-speedPid.err_last);
//	//TODO=========?????λ��ʽ
	tempPidOut=speedPid.kp*err +speedPid.mTi+tempDt;
	//printf("speedPid.outData%3.2f \r\n",speedPid.outData);
	
	//����λ��ʽPID����Ӧ�ٶȣ�
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
	//TODO------��ȡ��ʼ������һֱ����
	pid->ki=0.0;
	pid->kd=0.0;
	pid->IGate = 3000;
	pid->threshold =6000;
	pid->mTi=0.0;
}
