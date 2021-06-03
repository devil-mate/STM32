#include "platform.h"

#if (FAULHABER_MOTOR == 1)
#include "motor.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_conf.h"

#include "common.h"
#include "motor_faulhaber.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_conf.h"
#include "protocol_type.h"

#define 	getMotorId(header)   (((header.StdId & 0x000F) ) - 1)
#define 	VOLTAGE_MIN 	18.50 //最低电压/17.92 断电电压
#define 	VOLTAGE_MAX 	25.2
#define 	VOLTAGE_WARN 	20.0  //保证能完成一次任务电压

#define 	START_NODE_FUNCODE 	0x000
#define 	START_NODE_ID		0x0
#define		START_NODE_DATANUM 	2

#define 	HEARTBEAT_FUNCODE 	0x700
#define 	HEARTBEAT_ID		0x05
#define		HEARTBEAT_DATANUM 	1

//extern IWDG_HandleTypeDef hiwdg;
static uint16_t genCanId(MotorIndex motorId);
static void     setCanHeader(CAN_TxHeaderTypeDef *header, MotorIndex motorId); //get CODE-ID
static void     setCanHeader_(CAN_TxHeaderTypeDef *header, uint16_t Funcode, uint8_t motorId,uint8_t dataNum );
static void     stopMotor(MotorIndex motorId);
static void     switchOnMotor(MotorIndex motorId);
static void     enableOnMotor(MotorIndex motorId);
static void 	resetFaultMotor(MotorIndex motorId);
	
static void     setMotorSpeedCmd(MotorIndex motorId, int32_t speed);
static void     setMotorTorqueCmd(MotorIndex motorId, int16_t torque);
static void 	readMotorPositionCmd(MotorIndex motorId);
static void     readMotorVelocityCmd(MotorIndex motorId);
static void     readMotorCurrentCmd(MotorIndex motorId);
static void 	readMotorStateCmd(MotorIndex motorId);
static void 	readMotorVoltageCmd(MotorIndex motorId);
	
static void     setFeedbackInteval(MotorIndex motorId, uint8_t motorInfoInterval, uint8_t miscInterval);

static uint8_t  onlineState[2] = {0, 0};
uint16_t motorState=0;
ReadPositon_s sReadPositon;
//MotorFlag_s gMotorFlag_s;
static RealMotorInfo_s sRealMotorInfo={0};

float getVoltage();
bool gCanErrorFlag=false;
bool motorOnlineFlag = false;
/*motor.h公共函数部分*/
uint8_t initAllMotor()
{

	stopMotor(MotorL);//0x06停止
    stopMotor(MotorR);
    osDelay(100);
	//HAL_IWDG_Refresh(&hiwdg);

    switchOnMotor(MotorL);//0x07 switch on
    switchOnMotor(MotorR);
    osDelay(100);
	

//    setFeedbackInteval(MotorL, 100, 0);
//    setFeedbackInteval(MotorR, 100, 0);
	
	enableOnMotor(MotorL);//0x0F enable
    enableOnMotor(MotorR);
	//HAL_IWDG_Refresh(&hiwdg);
    osDelay(100);
    
    
//    onlineCheck(MotorL);
//    onlineCheck(MotorR);
//    osDelay(100);
	//printf("initAllMotor====== ");
  return onlineState[0] && onlineState[1];
	//return 0;
}

uint8_t resetAllMotor()
{
	resetFaultAllMotor();
    return initAllMotor();
}

uint8_t disenableAllMotor()
{
	osDelay(30);
    stopMotor(MotorL);//0x06
	osDelay(30);
    stopMotor(MotorR);
	printf("[WARN]disenableAllMotor--\r\n");
	
}
bool resetFaultAllMotor()
{
	//osDelay(30);
    resetFaultMotor(MotorL);//0x80故障复位
	//osDelay(30);
    resetFaultMotor(MotorR);
}
bool startAllNode_NMT()
{
	CAN_TxHeaderTypeDef header;
    uint8_t data[2] = {0};//
	data[0] = 0x01;
	data[1] =0x00;	//启动全部节点
	//功能0x0，地址0x0，数据字节2 
    setCanHeader_(&header,START_NODE_FUNCODE,START_NODE_ID,START_NODE_DATANUM);
    pushTxMsg0(&header, data);
}
bool heartbeatFrame_send()
{
	CAN_TxHeaderTypeDef header;
    uint8_t data[1] = {0};//
	data[0] = 0x05;
	//功能0x0，地址0x0，数据字节2 
    setCanHeader_(&header,HEARTBEAT_FUNCODE,HEARTBEAT_ID,HEARTBEAT_DATANUM);
    pushTxMsg0(&header, data);
	
}
uint8_t setMotorSpeed()
{
    int32_t motorRPM[2];
	
    motorRPM[MotorL] = (int32_t)(wheelRpmCmd.LeftRPM  * PLATFORM_GEAR_RATIO); //角速度*减速比=电机速度rpm
    motorRPM[MotorR] = (int32_t)(wheelRpmCmd.RightRPM * PLATFORM_GEAR_RATIO);
	motorRPM[MotorL] = absLimiter(motorRPM[MotorL], MAX_PWM_LIMIT);
	
    setMotorSpeedCmd(MotorL, motorRPM[MotorL]);	
    setMotorSpeedCmd(MotorR, motorRPM[MotorR]);
//	printf("motorRPML %d    motorRPMR %d\r\n",motorRPM[MotorL],motorRPM[MotorR]);
    return 1;
}
uint8_t setMotorTorque()
{
	int16_t motorTorque[2];
	
	motorTorque[MotorL]=(int16_t)(wheelTorqueS.LeftTorque);
	motorTorque[MotorR]=(int16_t)(wheelTorqueS.RightTorque);
	
	//驱动器配置保证不超过范围
//	motorTorque[MotorL] = absLimiter(motorTorque[MotorL], MAX_TORQUE_LIMIT);
//	motorTorque[MotorR] = absLimiter(motorTorque[MotorR], MAX_TORQUE_LIMIT);
//printf("torqueL: %d  torqueR: %d\r\n",motorTorque[MotorL],motorTorque[MotorR]);
	setMotorTorqueCmd(MotorL, motorTorque[MotorL]);
	setMotorTorqueCmd(MotorR, motorTorque[MotorR]);
	return 0;

}

uint8_t onlineCheck(MotorIndex motorId)
{
    onlineState[motorId] = 0;   //重置在线状态

    //发送检测帧
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();

    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);

    // 检测是否置1，超时判断离线
    uint32_t t = 0;
    while(onlineState[motorId] == 0 && t++ < ONLINE_CHECK_TIMEOUT)
    {
        osDelay(50);
				
    }
    return onlineState[motorId] == 1;
}
uint8_t stateCheck(MotorIndex motorId)
{
	CAN_TxHeaderTypeDef header;
    uint8_t data[8] = stateData();
    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
}

uint8_t readMotorVelocity()
{

	readMotorVelocityCmd(MotorL);
//	osDelay(10);
	readMotorVelocityCmd(MotorR);	

}
uint8_t readMotorPosition()
{
	readMotorPositionCmd(MotorL);
	osDelay(30);
	readMotorPositionCmd(MotorR);	
}

uint8_t readMotorCurrent()
{
	readMotorCurrentCmd(MotorL);
	readMotorCurrentCmd(MotorR);	
}
uint8_t readMotorState()
{
	readMotorStateCmd(MotorL);
//	osDelay(10);
	readMotorStateCmd(MotorR);	
}
uint8_t readMotorVoltage()
{
	readMotorVoltageCmd(MotorL);
	readMotorVoltageCmd(MotorR);	
}
void resolveFeedback(CanRxMsg_s *msg)
{
	
    int16_t realSatusWord;
    int32_t realVelocity,realPosition,deltaPos_count[2]; 
	float deltaPosition[2];
	uint8_t motorId = getMotorId(msg->header);
	//printf("motorId=====%d\r\n",motorId);
	static int16_t tcount=0;
	static int32_t newcount[2]={0},oldcount[2]={0};
	static int16_t canErrorCount = 0;

 // PDO模式下接收(pdo1 速度4，状态2，力矩2；pd02 电压2)
    uint8_t func = msg->header.StdId >> 4;
    switch (func)
    {
		
		//TPDO2 COB-id
    case 0x28:	
		sRealMotorInfo.voltageState[0]=(msg->data[0]) | (msg->data[1]<<8);
		float tempVoltage= getVoltage();
		RobotStateFrame.elecInfo.batVoltage=tempVoltage;
	    //realPosition=(msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
		sFlag.motorFlags.nmt_startNodeFlag = true;
		canErrorCount =0;
        break;
    case 0x18:
        //realVelocity = (msg->data[2] << 8) | msg->data[3];
		//0 1 2 3字节返回速度
        realVelocity = (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
		sRealMotorInfo.wheelVelocity[motorId] = realVelocity / PLATFORM_GEAR_RATIO*PI*PLATFORM_WHEEL_DIA/1000;
		RobotStateFrame.sMotionState.feedbackSpeed=-getVelocity();
		//4 5 字节返回状态
		realSatusWord = (msg->data[5] << 8) | msg->data[4];
		sRealMotorInfo.motorState[motorId]=realSatusWord;
		getMotorState( motorId);
		//电流
		int16_t realCurrent=(msg->data[6]) | (msg->data[7]<<8);
		RobotStateFrame.sMotorState.motorCurrent[motorId] = realCurrent ;


        break;
    default:
		canErrorCount++;
		if(canErrorCount>50){
			sFlag.motorFlags.nmt_startNodeFlag = false;
			canErrorCount=0;
		}
		
        break;
    }
//	if (func==0x16 | func==0x26){
//		onlineState[motorId] = 0;
//	}

//	//单独发送读指令后接收
//	//电流
//	if((msg->data[0]==0x4B) && (msg->data[1]==0x77) && (msg->data[2]==0x60) ){
////		realCurrent=*((int16_t*)(&(msg->data[4])));
//		
//        int16_t realCurrent=(msg->data[4]) | (msg->data[5]<<8);

//		RobotStateFrame.sMotorState.motorCurrent[motorId] = realCurrent ;
//		//正常读取电流，否则canError判断
//		canErrorCount = 0;
////		sRealMotorInfo.wheelCurrent[motorId] = realCurrent / 1.0f;
//		//printf("current[%d]:%3.4f \r\n",motorId,RobotStateFrame.sMotorState.motorCurrent[motorId]);
//		
//	}
//	//速度 
//	else if((msg->data[0]==0x43) && (msg->data[1]==0x6C) && (msg->data[2]==0x60)){

////		memcpy(&realVelocity, msg->data+4, 4);
//		//realVelocity=*((int32_t*)(msg->data+3));
//        realVelocity=(msg->data[4]) | (msg->data[5])<<8 | (msg->data[6])<<16 |(msg->data[7])<<24;
//		//  返回线速度速度，单位同给定目标速度，用作PID控制
//		sRealMotorInfo.wheelVelocity[motorId] = realVelocity / PLATFORM_GEAR_RATIO*PI*PLATFORM_WHEEL_DIA/1000;
//		RobotStateFrame.sMotionState.feedbackSpeed=-getVelocity();
//		//printf("speed[%d]:%3.4f \r\n",motorId,sRealMotorInfo.wheelVelocity[motorId]);
//		
//	}
//	//位置
//	else if((msg->data[0]==0x43) && (msg->data[1]==0x64) && (msg->data[2]==0x60)){

//        realPosition=(msg->data[4]) | (msg->data[5])<<8 | (msg->data[6])<<16 |(msg->data[7])<<24;
//		sReadPositon.realPosition_count[motorId] = realPosition;
//		newcount[motorId]=realPosition;
//		

//	}
//	//状态
//	else if((msg->data[0]==0x4B) && (msg->data[1]==0x41) && (msg->data[2]==0x60)){
//		sRealMotorInfo.motorState[motorId]=(msg->data[4]) | (msg->data[5]<<8);
//		getMotorState( motorId);

//	}
//	//电压
//	else if((msg->data[0]==0x4B) && (msg->data[1]==0x25) && (msg->data[2]==0x23)){
//		//只读取一个
//		sRealMotorInfo.voltageState[0]=(msg->data[4]) | (msg->data[5]<<8);
//		float tempVoltage= getVoltage();
//		RobotStateFrame.elecInfo.batVoltage=tempVoltage;

//	}
////	else{
////		canErrorCount++;
////		if(canErrorCount>1000){
////			gCanErrorFlag = true;
////		}else{
////			gCanErrorFlag = false;
////		}
////		
////	}

	tcount++;
	if(tcount%100==0){
		printf("batVoltage: %3.2f,feedbackSpeed: %3.2f, wheelVelocity[%d]: %3.2f\r\n",
			RobotStateFrame.elecInfo.batVoltage, RobotStateFrame.sMotionState.feedbackSpeed, 
			motorId,sRealMotorInfo.wheelVelocity[motorId]);
		printf("realSatusWord:============%d\r\n" ,realSatusWord);
		
		tcount=0;
	}
		

	
	
}
float getVelocity()
{
	//TODO 电机掉线判断
	float tempSpeed=0;
	//taskENTER_CRITICAL();
	if (!sFlag.motorFlags.enableFlag[0] || sFlag.motorFlags.motorErrorFlag[0]){
		tempSpeed = -sRealMotorInfo.wheelVelocity[1];;
	}
	else if (!sFlag.motorFlags.enableFlag[1] || sFlag.motorFlags.motorErrorFlag[1]){
		tempSpeed = sRealMotorInfo.wheelVelocity[0];;
	}
	
	else{
		tempSpeed = (sRealMotorInfo.wheelVelocity[0]-sRealMotorInfo.wheelVelocity[1])/2.0;
	}
	//taskEXIT_CRITICAL();
	return tempSpeed;
}
float getVoltage()
{
	float tempVoltage,resultV;
	tempVoltage = sRealMotorInfo.voltageState[0]/100.0f; //10mv 单位
	if(tempVoltage <VOLTAGE_WARN){
		osDelay(1000);
		sFlag.voltageFlag = true;
	}
	else{
		sFlag.voltageFlag = false;
	}

	resultV = (tempVoltage-VOLTAGE_MIN)/cABS(VOLTAGE_MAX-VOLTAGE_MIN); //返回电压百分比
	//printf("batVoltage: %3.2f\r\n",resultV);
	return resultV;
}

/*私有函数部分*/
void getMotorState(uint8_t motorId){
	uint8_t tempstate = sRealMotorInfo.motorState[motorId];
	//故障 1000 = 0x08
	if(0x08 ==(tempstate &0x08)){
		sFlag.motorFlags.motorErrorFlag[motorId]=true;
	}
	else{
		sFlag.motorFlags.motorErrorFlag[motorId]=false;
	}
	//操作/使能 0010 0111  =0x27
	if(0x27 ==(tempstate &0x27)){
		sFlag.motorFlags.enableFlag[motorId]=true;
		
	}
	else{
		sFlag.motorFlags.enableFlag[motorId]=false;
		
	}
		
	taskENTER_CRITICAL();
	if(sFlag.motorFlags.enableFlag[0] && sFlag.motorFlags.enableFlag[1]){
		setbit(RobotStateFeed.digit,7); //电机使能
	}
	else{
		resetbit(RobotStateFeed.digit,7); 
	}
	taskEXIT_CRITICAL();
//	printf("tempstate:%d",tempstate);
//	printf("sFlag.motorFlags.enableFlag[%d]:  %d   motorErrorFlag[%d]:%d\r\n",motorId,
//					sFlag.motorFlags.enableFlag[motorId], motorId,sFlag.motorFlags.motorErrorFlag[motorId]);
}


static uint16_t genCanId(MotorIndex motorId)
{
//    return    func | MOTOR_GROUP_NUM << 8 | (motorId + 1) << 4;
		return 0x601+motorId;
}

static void setCanHeader(CAN_TxHeaderTypeDef *header, MotorIndex motorId)
{
    header->StdId = genCanId(motorId);
    header->DLC = 8;
    header->ExtId = 0x00;
    header->IDE = CAN_ID_STD;
    header->RTR = CAN_RTR_DATA;
}
static void setCanHeader_(CAN_TxHeaderTypeDef *header, uint16_t Funcode,uint8_t motorId,uint8_t dataNum )
{
    header->StdId = Funcode+motorId;
    header->DLC = dataNum;
    header->ExtId = 0x00;
    header->IDE = CAN_ID_STD;
    header->RTR = CAN_RTR_DATA;
}

static void stopMotor(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();//0x06停止

    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
}


static void switchOnMotor(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();
    data[4] = 0x07;
    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
}
static void enableOnMotor(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();
    data[4] = 0x0F;
    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
}

static void resetFaultMotor(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();//0x06停止
	data[4] = 0x80;
    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
}
static void setMotorSpeedCmd(MotorIndex motorId, int32_t speed)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = speedData();
    setCanHeader(&header, motorId);
	
    data[7] = (uint8_t)((speed >> 24) & 0xff);
    data[6] = (uint8_t)((speed >> 16) & 0xff);
    data[5] = (uint8_t)((speed >> 8) & 0xff);
    data[4] = (uint8_t)(speed & 0xff);

    pushTxMsg0(&header, data);
	
}
static void setMotorTorqueCmd(MotorIndex motorId, int16_t torque)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = torqueData();
    setCanHeader(&header, motorId);
	

    data[5] = (uint8_t)((torque >> 8) & 0xff);
    data[4] = (uint8_t)(torque & 0xff);
//	printf("data4:%x  data5:%x\r\n",data[4],data[5]);
//	data[5] = 0x01;
//    data[4] = 0x2c;

    pushTxMsg0(&header, data);
	
}
static void readMotorVelocityCmd(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = readVelocityData();
    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
	
}
static void readMotorPositionCmd(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = readPositionData();
    setCanHeader(&header, motorId);

    pushTxMsg0(&header, data);
	
}
static void readMotorCurrentCmd(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = readCurrentData();
    setCanHeader(&header, motorId);

    pushTxMsg0(&header, data);
	
}
static void readMotorVoltageCmd(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = voltageData();
    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
	
}

static void readMotorStateCmd(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = stateData();
    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
	
}

/**
 * @description: 设置[实时电流、速度、位置值]及混杂项反馈间隔时间(ms)
 * @param {type} 
 * @return: 
 */
//static void setFeedbackInteval(MotorIndex motorId, uint8_t motorInfoInterval, uint8_t miscInterval)
//{
//    CAN_TxHeaderTypeDef header;
//    uint8_t data[8] = initData();
//    setCanHeader(&header, motorId);
//    data[0] = motorInfoInterval;
//    data[1] = miscInterval;

//    pushTxMsg0(&header, data);
//}
#endif
