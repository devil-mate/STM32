#include "motor.h"
#include "motion_controller.h"
#include "cmsis_os.h"
#include "platform.h"
#include "protocol_type.h"
#include "stm32f4xx_hal_iwdg.h"

//#define TEST_DEBUG 1
#define fowardLimit limitFlag[0]
#define backwardLimit limitFlag[1]
#define testAutoSpeed  (10.0f)



WheelRPM_s wheelRpmCmd = {0.0, 0.0};
WheelTorque_s wheelTorqueS ={0,0};

//////////����
static void motionControllerTask(void *argument);
static TaskHandle_t MotionControllerTaskHandle = NULL;

//static int16_t getMotorCurrent(MotorIndex motorId);

static void copyGlobal2Local();
//static void receiveCmdMessage(Queue_Message_s *message);
static void resolveWheelRpm(void);
static void getControlDigit(ControlDigit_s *ControlDigit);
static void manualOperation();
static void autoOperation();
static void delayFeedBack();

static float pidController(float target, float feedbackData);
static void TorquesResolve(float torque);
//int getMode(uint8_t* mode,int8_t *start);
//float autoSpeedControler(float speed);
float manualSpeedControler(float speed);
float torqueController(float targetSpeed);


//////////����

extern MoveCmd_s           	MoveCmd;
extern PidParamCmd_s       	PidParamCmd;
extern Flag_s 				sFlag;


static RobotStateFeed_s     LocalRobotStateFeed; //��������
static RobotStateFrame_s	LocalRobotStateFrame; //��ʱ����
static MoveCmd_s           	LocalMoveCmd;
static PidParamCmd_s       	LocalPidParamCmd;
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
	memset(&LocalPidParamCmd,0,sizeof(LocalPidParamCmd));
	memset(&LocalParamConfig,0,sizeof(LocalParamConfig));
	
	memset(&LocalRobotStateFeed,0,sizeof(LocalRobotStateFeed));
	memset(&LocalRobotStateFrame,0,sizeof(LocalRobotStateFrame));
    xTaskCreate(motionControllerTask,         /* ������  */
                "MotionControllerTask",       /* ������    */
                2048,                         /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                         /* �������  */
                12,                            /* �������ȼ�*/
                &MotionControllerTaskHandle); /* ������  */
}

static void motionControllerTask(void *argument)
{	
	//TODO========�������Ź���־�ش�	
	printf("===================================start");
//	for(int k = 0; k < 50; k++) {
//		osDelay(100);
//		HAL_IWDG_Refresh(&hiwdg);
//	}
	
    bool reseted = false;

	

	InitProtocolFrame(LocalRobotStateFeed);
	InitProtocolFrame(LocalRobotStateFrame);
	
	LocalRobotStateFeed.funcCode = Func_StateFeed;
	LocalRobotStateFrame.funcCode = Func_StateFrame;
	
	//��ʼ��
	//������500k�������˲��÷�������
	
	//heartbeatFrame_send();
	
	disenableAllMotor();
	pidInit(&speedPid);
	movAvgFilterInit(&speedFilter);
	initSpeedFlag=true;
	LocalParamConfig.taskTotal=1;
	currentState=stopState;
	autoState=forwardState;
	float tempOutTargetTorque=0.0;
	bool motorEnableFlag=false;
	/***********test**********/
//	currentState=autoModeState;
//	start=1;
//	sControlDigit.ControlMode=0x01;
//	sControlDigit.Start=1;
	//currentState=autoModeState;
	//TODOʹ�ܵ�����ɹ�����
	/***********test**********/
	g_motorEnFlag=true;
    while (1)
    {	
		HAL_IWDG_Refresh(&hiwdg);
		osDelay(20);
		copyGlobal2Local();
		// ��֤����������ģʽ/PDO�������Լ����������ͺ󣬲���ʹ�ܵ����

		
		//TODO ��ʱ������PID
		//��Ϣ���н�����λ����������������ݣ��������������sControlDigit
        //receiveCmdMessage(&localQueueMessage);
		//xQueueReceive(QueueHandle,&localQueueMessage,30);

		getControlDigit(&sControlDigit);
        if (currentState != resetState && reseted){
            reseted = false;
        }

		if(sControlDigit.Stop){
			currentState=stopState;
		}
        switch (currentState)
        {
			default:
				//printf("have no this controlMode");
				currentState=stopState;
				break;
//			//�ֶ�ģʽ0x01
			case manulModeState:
				if(motorEnableFlag){
						
				}
				manualOperation();
				setMotorTorque();
//				//3 ��λ��2�Զ���1�ֶ����Զ�ʱ��ֱ�ӽ����Զ�ģʽ���Ƚ���ֹͣ״̬�������������� �����Զ�ģʽ��ǰ������״̬
				if(sControlDigit.ControlMode==RESET_MODE){
					currentState=resetState;
				}
				if(sControlDigit.ControlMode==AUTO_MODE){
					currentState=stopState;
				}
				break;
//			
//			//�Զ�ģʽ0x02
			case autoModeState:
				autoOperation();
				setMotorTorque();
				if(sControlDigit.ControlMode==RESET_MODE){
					currentState=resetState;
				}
				if(sControlDigit.ControlMode==MANUAL_MODE){
					currentState=stopState;
				}			
				break;

//			//ֹͣģʽ0x04
			case stopState:
				gLocal_startFlag =false;
				gLocal_brokenFlag =false;
				//taskENTER_CRITICAL();
				//memset(&onceTaskInfo_s,0,sizeof(onceTaskInfo_s));
				//taskEXIT_CRITICAL();
				taskCount=0;
				localFlag.runningtFlag=0;
				//wheelRpmCmd.LeftRPM=wheelRpmCmd.RightRPM=0;
				autoState=forwardState;
				tempOutTargetTorque=torqueController(0);//ֹͣģʽ���ٶ�Ϊ0
				TorquesResolve(tempOutTargetTorque);
				//TorquesResolve(0);//�ֽ⵽����
//				movAvgFilterInit(&speedFilter);
				setMotorTorque();

//				if(sControlDigit.errorVerify){
//					if(!(sFlag.disconnetFlag && sControlDigit.controlBox_dis)){
//						resetAllMotor();
//						log_info("Reset all motor...\r\n");						
//					}
//				}
				if(sControlDigit.ControlMode==MANUAL_MODE){
					currentState=manulModeState;
					memset(&onceTaskInfo_s,0,sizeof(onceTaskInfo_s));
				}
				else if(sControlDigit.ControlMode==AUTO_MODE){
					//ֹͣģʽ���жϣ�������Զ������жϽ����Զ���ǰ�����Ǻ���;���ߣ�ǰ�����������Ž����Զ�ģʽ
					if(sControlDigit.Start_Forward && motorEnableFlag && sControlDigit.createFailFlag_cmd ){
						//speedPid.mTi = 0;
						//TODO������������ԭ��
						if(!(localFlag.fowardLimit || localFlag.laserStateFlagF || localFlag.voltageFlag )){
							currentState=autoModeState;
						}	
					}
				}
				else if(sControlDigit.ControlMode==RESET_MODE){
					currentState=resetState;
				}
				//��⵽�ϰ��ͬʱ�ֶ���(��;�������)
				//ֹͣģʽ�£����ߣ�ʧ��
//				if(localFlag.backwardLimit){
//					if(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect){
//						TorquesResolve(0);//����Ϊ0
//						disenableAllMotor();
//					}
//				}
				if(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect){
						TorquesResolve(0);//����Ϊ0
						disenableAllMotor();
				}
//				
				break;
		//			//��λ0x03
			case resetState:
				gLocal_brokenFlag =false;
				gLocal_startFlag =false;
				if(sControlDigit.ControlMode==AUTO_MODE ){
					currentState=stopState;
				}
				if(sControlDigit.ControlMode==MANUAL_MODE){
					if(motorEnableFlag){
						currentState=manulModeState;
					}
				}
				//��λģʽ����
				if(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect){
						TorquesResolve(0);//����Ϊ0
						disenableAllMotor();
				}
				if (!reseted){	
					//taskENTER_CRITICAL();					
					memset(&MoveCmd, 0, sizeof(MoveCmd));
					memset(&sFlag,0,sizeof(sFlag));
					memset(&onceTaskInfo_s,0,sizeof(onceTaskInfo_s));
					if(!localFlag.motorFlags.nmt_startNodeFlag){
						//û�������յ�PDO����
						printf("PDO data back error\r\n");
						continue;
					}		
					if(!localFlag.motorFlags.heartbeatSendFlag){
						//û�������յ�PDO����
						printf("heartbeatSendFlag  error\r\n");
						continue;
					}
					//taskEXIT_CRITICAL();
					//resetFaultAllMotor();
					resetAllMotor();
					//readMotorState();
					log_info("============================Reset all motor...\r\n");					

					osDelay(300);				
					taskENTER_CRITICAL();
					bool enable01 = sFlag.motorFlags.enableFlag[0];
					bool enable02 = sFlag.motorFlags.enableFlag[1];
					taskEXIT_CRITICAL();
					motorEnableFlag = enable01 & enable02;
					if(motorEnableFlag){
						reseted = true;
						autoState=forwardState;
						speedPid.mTi = 0;  //������0
					}else{
						osDelay(500);
					}
					
				}
				break;
//			//����ģʽ0x05
//			case errorState:
//				wheelRpmCmd.LeftRPM=wheelRpmCmd.RightRPM=0;
//				wheelTorqueS.LeftTorque=wheelTorqueS.RightTorque=0;
//			
//				autoState=judgeState;
//				//movAvgFilterInit(&speedFilter);
////				TorquesResolve(0);
////				setMotorTorque();
//				//disenableAllMotor();
//				
//				if(sControlDigit.errorVerify){
//					if(!(sFlag.disconnetFlag && sControlDigit.controlBox_dis)){
//						initAllMotor();
//						osDelay(100);
//					}
//					if(sControlDigit.ControlMode==1){
//						currentState=manulModeState;
//					}
//					if(sControlDigit.ControlMode==2){
//						currentState=stopState;
//					}

//				}
//				if(sControlDigit.ControlMode==3){
//					currentState=resetState;
//				}
//				break;

        }

///**************feedback*****************/
//		//������������
		static uint8_t tcount=0;
		tcount++;
		if(tcount%10==0){
			delayFeedBack();
			tcount=0;
		}
///**************feedback*****************/
///********************printf****************/

/********************printf****************/		

	}		
}

static void copyGlobal2Local()
{
    taskENTER_CRITICAL();
    memcpy(&LocalMoveCmd, &MoveCmd, sizeof(MoveCmd));
	memcpy(&localFlag, &sFlag, sizeof(sFlag));
	taskEXIT_CRITICAL();
	
    memcpy(&LocalPidParamCmd, &PidParamCmd, sizeof(PidParamCmd));
	memcpy(&LocalParamConfig, &ParamConfigs, sizeof(ParamConfigs));
	
	//memcpy(&LocalRobotStateFeed, &RobotStateFeed, sizeof(RobotStateFeed));
	LocalRobotStateFeed.digit = RobotStateFeed.digit;
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


//linux���������������20m/min
static void resolveWheelRpm() 
{
    LocalMoveCmd.speed = absLimiter(LocalMoveCmd.speed, PLATFORM_MAX_SPEED_M_PER_MIN);
//    wheelRpmCmd.LeftRPM=LocalMoveCmd.speed* 1000 / 60;
//	wheelRpmCmd.RightRPM=LocalMoveCmd.speed* 1000 / 60;
	
//	
	wheelRpmCmd.RightRPM  = (LocalMoveCmd.speed * 1000 ) / PI / PLATFORM_WHEEL_DIA ;//���ٶȵ�λrpm
	wheelRpmCmd.LeftRPM  = -(LocalMoveCmd.speed * 1000 ) / PI / PLATFORM_WHEEL_DIA;
	
	//���������ٶ�
    //wheelRpmCmd.LeftRPM  = (LocalMoveCmd.speed * 1000 / 60 - PLATFORM_WHEEL_TREAD / 2 * LocalMoveCmd.omega) / PI / PLATFORM_WHEEL_DIA * 60;
   
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
	
	

	//PID����
	speedPid.kp=LocalPidParamCmd.Kp;
	speedPid.ki=LocalPidParamCmd.Ki;
	speedPid.kd=LocalPidParamCmd.Kd;
	speedPid.IGate = LocalPidParamCmd.IGate;
	speedPid.threshold=LocalPidParamCmd.OutGate;
	

	
}

float manualSpeedControler(float speed)
{
	float manualSpeed=speed;
	
	if((localFlag.laserStateFlagF && localFlag.laserStateFlagB) ||(localFlag.fowardLimit && localFlag.backwardLimit)){
		manualSpeed=0;
	}
	else if(localFlag.laserStateFlagF || localFlag.fowardLimit ){
		if(speed>=0){
			manualSpeed=0;
		}
		else{
			manualSpeed=speed;
		}
	}
	else if(localFlag.laserStateFlagB || localFlag.backwardLimit){
		if(speed<=0){
			manualSpeed=0;
		}
		else{
			manualSpeed=speed;
		}
	}
	

	return manualSpeed;
}
//float autoSpeedControler(float speed)
//{	
//	
//	static float autospeed;
//	if(initSpeedFlag){
//		autospeed=speed;
//		initSpeedFlag=0;
//	}
////	if(!(fowardLimit && backwardLimit)){
////		if(initSpeedFlag){
////			autospeed=speed;
////			
////		}
////	}
////	
//	if(localFlag.fowardLimit){
//		
//		autospeed=-speed;
//		
//	}
//	if(localFlag.backwardLimit){
//		autospeed=speed;
//		
//	}
//	if((localFlag.fowardLimit && localFlag.backwardLimit)){
//		autospeed=0;
//		
//	}
//	return autospeed;
//	
//}



float torqueController(float targetSpeed)
{
	float feedbackSpeed,backFliterSpeed,errorSpeed,OutTorque=0;
	//static float outTorque=0.0,testOutTorque=0.0; 

//#endif 	
	float wheelBackSpeedL=0,wheelBackSpeedR=0;
	
//	//TODO========����һ���ٶȶ�ȡ����
//	if (sFlag.motorErrorFlag[0]){
//		feedbackSpeed = -LocalRobotStateFrame.sMotorState.wheelRPM[1];
//	}
//	else if (sFlag.motorErrorFlag[1]){
//		feedbackSpeed = LocalRobotStateFrame.sMotorState.wheelRPM[0];
//	}
//	else{
//		feedbackSpeed = (LocalRobotStateFrame.sMotorState.wheelRPM[0]-LocalRobotStateFrame.sMotorState.wheelRPM[1])/2.0;
//	}
//	if((!motorErrorFlag[0] )&& (!motorErrorFlag[1])){
//		feedbackSpeed= (LocalRobotStateFrame.motionState.wheelRPM[0]-LocalRobotStateFrame.motionState.wheelRPM[1])/2;//����Ϊ��
//	}
	feedbackSpeed=getVelocity();
	backFliterSpeed=movAvgFilter(&speedFilter,feedbackSpeed,5,0);

	//TODO==����ʽPID��ʼ��ֵ
	
	OutTorque=pidController(targetSpeed,backFliterSpeed);
	
	
	
////////////printf
//	static int t=0;
//	t++;
//	if(t%10==0){ 


//		//printf("speedPid.err: %3.2f   speedPid.eror-1: %3.2f    speedPid.err-2: %3.2f \r\n",speedPid.err,speedPid.err_next,speedPid.err_last);
//		printf("targetSpeed%3.2f    feedbackSpeed%3.2f  feedbackSpeed(Fliter)%3.2f outTorque%3.2f\r\n",
//			targetSpeed,feedbackSpeed,backFliterSpeed, OutTorque);
//		t=0;
//	}
////////////printf	
	return OutTorque;
	
}


//void autoTorquesResolve(float torque)
//{
////	if(!(fowardLimit && backwardLimit)){
////		if(initSpeedFlag){
////			wheelTorqueS.LeftTorque=(int16_t)(torque/2+0.5);
////			wheelTorqueS.RightTorque=(int16_t)(-torque/2+0.5);
////			initSpeedFlag=0;
////		}
////	}
////	if(fowardLimit){
////		
////		wheelTorqueS.LeftTorque=-torque/2;;
////		wheelTorqueS.RightTorque=torque/2;
////	}
////	if(backwardLimit){
////		wheelTorqueS.LeftTorque=-torque/2;
////		wheelTorqueS.RightTorque=torque/2;
////		
////	}
////	if((fowardLimit && backwardLimit)){
////		wheelTorqueS.LeftTorque=wheelTorqueS.RightTorque=0;
////		
////		
////	}
////	initSpeedFlag=0;
////	
//}
static void TorquesResolve(float torque)
{
	wheelTorqueS.LeftTorque=(int16_t)(torque/1+0.5);
	wheelTorqueS.RightTorque=(int16_t)(-torque/1+0.5);
	//printf("++++LeftTorque%d   RightTorque%d     \r\n",wheelTorqueS.LeftTorque,wheelTorqueS.RightTorque);
	
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

//void taskController(bool *start,float* speed)
//{
//	bool forwardFlag,backFlag;
//	if(sControlDigit.OnceTask==1){
//		//TODO===��ʼ�����ж�
//		*start=1;
//	}else{
//		*start=0;
//		
//	}
//	if(start && LocalRobotStateFrame.sMotorState.wheelRPM[0]>0){
//		forwardFlag=1;
//		backFlag=0;
//	}
//	if(forwardFlag && localFlag.fowardLimit){
//		backFlag=1;
//		forwardFlag=0;
//	}
//	if(backFlag && localFlag.backwardLimit){
//		*start=0;
//	}
//		
//	
//	
//}
//void feedback_mode()
//{

//}
static void manualOperation()
{	
	float tempManualSpeed=0;
	float OutTargetTorque=0.0;

	//�ֶ��ſ�����λ�򴫸�������
	//TODO------���ߺ��·����ٶ���֮ǰ���ٶ�
	//				
	//if(sControlDigit.errorVerify!=0){
	//�ж��ֶ��ٶȹ����½���
	//bool speedDownFlag = false;
	static float lastSpeed =0;
	//stm32ͨ�Ŵ��ڶ��ߣ�controlBox_dis���������
	if(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect){
		TorquesResolve(0);//����Ϊ0
		disenableAllMotor();
	}
	else{
		if(cABS(LocalMoveCmd.speed)<=0.1){
			//OutTargetTorque=0; //û���ֶ�ָ��������»�
//			if(cABS(lastSpeed)>0.1){
//				//speedPid.mTi=0;
//			}
			tempManualSpeed=0; //û���ֶ�ָ���,ͣ��ԭ�������ߺ�������»�
		}else{
			//0 �����ٶȱ���manualSpeedControler�����������ǰ�������ת��-���������+
		//	tempManualSpeed=-manualSpeedControler(LocalMoveCmd.speed);	
			tempManualSpeed=-LocalMoveCmd.speed; //��ǰ����ٶ����������

		}
		//lastSpeed= LocalMoveCmd.speed;

	}

	if(!sControlDigit.Emergency){
		disenableAllMotor();
		currentState=stopState;
	}
	//printf("OutTargetTorque%3.2f\r\n",OutTargetTorque);
	OutTargetTorque=torqueController(tempManualSpeed);//����ģʽ�������ٶ� ��pid����
	TorquesResolve(OutTargetTorque);
	RobotStateFrame.sMotionState.manulTargetSpeed = -tempManualSpeed;
}
static void autoOperation()
{
	float tempAutoSpeed=0.0;
	float OutTargetTorque=0.0;
	switch(autoState){	
	default:
		//autoState = backwardState;
		return;
	case forwardState:
		gLocal_startFlag =true;
		if(localFlag.fowardLimit || localFlag.laserStateFlagF || localFlag.disconnetFlag || 
			sControlDigit.controlBox_disconnect || sControlDigit.Stop_Backward || localFlag.voltageFlag){
			//speedPid.mTi=0;
			
			autoState=backwardState;
			tempAutoSpeed=0;
		}
		else{
			tempAutoSpeed = -cABS(LocalMoveCmd.autoSpeed);//�����ǰ-���������+
			localFlag.autoStopFlag=0;
			localFlag.runningtFlag=1;
		}
		if(localFlag.laserStateFlagF || localFlag.disconnetFlag || 
			sControlDigit.controlBox_disconnect || sControlDigit.Stop_Backward ){
				taskCount=LocalParamConfig.taskTotal+1;
				gLocal_brokenFlag = true;
		}
			//test-----
//		if(sControlDigit.controlBox_disconnect ){
//			TorquesResolve(0);
//			while(1){
//				osDelay(300);
//				printf("controlBox_disconnect is true! %d \r\n",sControlDigit.controlBox_disconnect);
//			}
//		}
//		if(sControlDigit.Stop_Backward ){
//			TorquesResolve(0);
//			while(1){
//				osDelay(300);
//				printf("Stop_Backward is true!  %d\r\n",sControlDigit.Stop_Backward );
//			}
//		}
//		if(localFlag.laserStateFlagF ){
//			TorquesResolve(0);
//			while(1){
//				osDelay(300);
//				printf("laserStateFlagF is true! %d \r\n",localFlag.laserStateFlagF);
//			}
//		}
//		if(localFlag.fowardLimit ){
//			TorquesResolve(0);
//			while(1){
//				osDelay(300);
//				printf("fowardLimit is true! %d \r\n",localFlag.fowardLimit);
//				if(sControlDigit.Stop_Backward){
//					break;
//				}
//			}
//		}
//		if(localFlag.disconnetFlag ){

//			while(1){
//				osDelay(300);
//				printf("disconnetFlag is true! %d \r\n",localFlag.disconnetFlag);
//			}
//		}

		break;
	case backwardState:
			if( !(localFlag.backwardLimit) || localFlag.laserStateFlagB){
				tempAutoSpeed = cABS(LocalMoveCmd.autoSpeed);
			}
			else{
				//���û�е��ߣ��жϽ���stopState��������ߣ������˻�
				if(!(localFlag.disconnetFlag || sControlDigit.controlBox_disconnect)){
					taskCount++;
					/*****feedback*** temp**/
					//LocalRobotStateFrame.posture.picth=taskCount;
					/*****feedback*** temp**/
					if(taskCount>=LocalParamConfig.taskTotal ){
						memset(&onceTaskInfo_s,0,sizeof(onceTaskInfo_s));
						//�����������Ҫ���¸�λ�����ļ��С�
						sControlDigit.createFailFlag_cmd=false;
						tempAutoSpeed = 0;
						currentState=stopState;
						autoState=forwardState;
						localFlag.autoStopFlag=1;
						localFlag.runningtFlag=0;
						taskCount=0;
				// todo ----���������־λ�ϴ�
					}else{
						speedPid.mTi=0;
						tempAutoSpeed = 0;
						autoState=forwardState;
					}
					

				}
				else{
					// ��⵽�ϰ���,������ߣ������˻أ�������tempAutoSpeed�����ǻ�����ײ���⡣
					//TODO���˻���ײ������,��ʱ��
					taskCount=LocalParamConfig.taskTotal+1;
					localFlag.autoStopFlag=1;
					localFlag.runningtFlag=0;
				}

			}
			
//			if(localFlag.laserStateFlagB){
//				//TODO����
//			}
	
		break;
	}

	OutTargetTorque=torqueController(tempAutoSpeed);
	TorquesResolve(OutTargetTorque);//�ֽ⵽����
	/*****feedback*** ����Ŀ���ٶ�temp**/
	RobotStateFrame.sMotionState.autoTargetSpeed=-tempAutoSpeed;
	/*****feedback*** ����Ŀ���ٶ�temptemp**/
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
			if(localFlag.laserStateFlagF){
				setbit(LocalRobotStateFeed.digit,0); // 0,1 ǰ�󼤹�/���봫����״̬
			}
			else{
				resetbit(LocalRobotStateFeed.digit,0); 
			}
			if(localFlag.laserStateFlagB){
				setbit(LocalRobotStateFeed.digit,1); // 0,1 ǰ�󼤹�/���봫����״̬
			}
			else{
				resetbit(LocalRobotStateFeed.digit,1); 
			}
			if(currentState==autoModeState && autoState==forwardState){
				setbit(LocalRobotStateFeed.digit,3); //ǰ����
				resetbit(LocalRobotStateFeed.digit,4);
			}
			else if(currentState==autoModeState && autoState==backwardState){
				setbit(LocalRobotStateFeed.digit,4); //������
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
			RobotStateFeed.digit = LocalRobotStateFeed.digit;
			RobotStateFrame.digit = LocalRobotStateFrame.digit;
			//memcpy(&RobotStateFeed, &LocalRobotStateFeed, sizeof(LocalRobotStateFeed));
			//memcpy(&RobotStateFrame, &LocalRobotStateFrame, sizeof(LocalRobotStateFrame));
			taskEXIT_CRITICAL();
			//printf("local to RobotStateFeed.digit:%d\r\n",RobotStateFeed.digit);

		/**************feedback*****************/
	
}

