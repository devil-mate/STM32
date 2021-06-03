#include "common.h"
#include "motor.h"

#include "UartDirver.h"
//#include "UartDriver.h"
#if (UART_MOTOTR == 1)
#include "cmsis_os.h"
#include "uart_platform_motor.h"
#include "can_handler.h"
#include <string.h>
#include "motor_ctrl_task.h"
#include "pid.h"

TickType_t xTicksToWait=10; //ms
static bool setWheelVelocityCmd(MotorID_e motorID,int16_t rpm);

//extern CAN_HandleTypeDef hcan1;
#define 	ATTESTATION_CANID 			0x77
#define 	CONTROL_CANID				0x200
#define 	STEERSPEED_CANID			0xE0
//#define 	FB_BRAKE_PRESSURE_CANID  	0x30
//#define 	FB_YAW_CANID  				0x33
//#define 	FB_SYSMODE_CANID  			0x36
#define		DATANUM 					0x08
#define 	MAX_CURRENT					10000
#define 	MIN_CURRENT					-10000
#define		MOTOR_NUM					2
#define  	MOTOR_SPEED_MAX				6000

#define  IniCmd_setV(ID,v) ("#e,SVS,1,#v\r\n")
#define  IniCmd_readV(ID,v) ("#e,SVS,1,#v\r\n")
char IniCmd_setVL[]="1,SVS,1,100\r\n";

char IniCmd2[]="AT+UART=115200,8,0,0\r\n";
char IniCmd3[]="AT+NAME=GPSM\r\n";
char IniCmd4[]="AT+ENTM\r\n";
char IniCmd5[]="AT+UART?\r\n";


#pragma pack(push, 1)
typedef struct{
	int16_t		motor01_current;
	int16_t		motor02_current;
	int16_t		motor03_current;
	int16_t		motor04_current;	
}SentData_s;  //sendData,对输入的数据做限制

typedef struct{
	int16_t 	angle; //刹车度 0~100 
	int16_t 	velocity;
	int16_t		torque;		

}TempMotorFeedData_s;



#pragma pack(pop)
CAN_TxHeaderTypeDef CAN1_TxHeader;
CAN_RxHeaderTypeDef CAN1_RxHeader;

static  TempMotorFeedData_s sTempMotorFeedData_[MOTOR_NUM]={0};
MotorFeedData_s sMotorFeedData_g[MOTOR_NUM]={0};


static void     setCanHeader_(CAN_TxHeaderTypeDef *header, uint16_t MsgID,uint32_t dataNum );
static void     setCanHeader(CAN_TxHeaderTypeDef *header, uint16_t MsgID,uint32_t dataNum );
static void 	setCanHeaderXID(CAN_TxHeaderTypeDef *header,uint32_t dataNum );

static float getMotorAngle (uint8_t motorID);
static float getMotorVelocity (uint8_t motorID);
static float getMotorTorque (uint8_t motorID);
//static uint8_t sendCurrentCmd( int16_t sendDatas[],const uint8_t length);
static uint8_t sendCurrentCmd( int16_t sendDatas[]);//length 有电机数量决定


pid_type_def sMotorVolecityPID[MOTOR_NUM];
/**
* 接收解析
*/



/**
  * 发送
  */

uint8_t initControl()
{


}
//TODO 使能
uint8_t resetControl()
{

}
bool getWheelVelocity(float *velocityL,float *velocityR){
	
}

bool setWheelVelocity(int16_t rpmL,int16_t rpmR){
	setWheelVelocityCmd(MotorL,rpmL);
	setWheelVelocityCmd(MotorR,rpmR);

}

// 输入速度命令 rpm
static bool setWheelVelocityCmd(MotorID_e motorID,int16_t rpm) //
{
	if(rpm >MOTOR_MAX_RPM){
		rpm = MOTOR_MAX_RPM;
	}
	char buf[]=IniCmd_setV(motorID,rpm);
	xQueueSend(ComPorts[Uart_motor]->TxQueue,buf,xTicksToWait);
}
static bool readWheelVelocityCmd(MotorID_e motorID) //
{

	char buf[]=IniCmd_setV(motorID,rpm);
	xQueueSend(ComPorts[Uart_motor]->TxQueue,buf,xTicksToWait);
}
uint8_t sendControlFrame()
{
	int16_t tempOut[MOTOR_NUM];
	//获取PID结果--PID在计时器中计算
	for(uint8_t i=0;i<MOTOR_NUM;i++){
		tempOut[i]= (int16_t)sMotorVolecityPID[i].out;
		//printf("set torque: %d",tempOut[i]);
		//tempOut[i]=300;
	}
	sendCurrentCmd(tempOut);	
	
}
void resolveFeedback(CanRxMsg_s *msg)
{
	static bool update=0;
	static int16_t canErrorCount = 0;
//// // PDO模式下接收
	uint8_t motorID = msg->header.StdId-1;
    uint8_t func = msg->header.StdId>>4; //0x20

    switch (func)
    {
		default:
			canErrorCount++;
			if(canErrorCount>50){
	//			sFlag.motorFlags.nmt_startNodeFlag = false;
	//			printf("can have no receive data\r\n");
				//TODO can receive error
				sFlag.canMotorReceiveFlag =false;
				canErrorCount=0;
				PID_clear(&sMotorVolecityPID[MotorR]);
				memset(&sTempMotorFeedData_,0,sizeof(sTempMotorFeedData_));
			}
			//
			break;
		//TODO 电机故障判断
		//电机角度、速度、力矩反馈
		case 0x20:	
			canErrorCount =0;

			sFlag.canMotorReceiveFlag =true;
			//sFlag_g.canInitReciveFlag=true;


			sTempMotorFeedData_[motorID].angle = (msg->data[0])<<8 | (msg->data[1]);
			sTempMotorFeedData_[motorID].velocity = (msg->data[2])<<8 | (msg->data[3]);
			//sTempMotorFeedData_[motorID].torque = (msg->data[4])<<8 | (msg->data[5]);
			//sFlag.motorFlags.nmt_startNodeFlag = true;
		//全局
			sMotorFeedData_g[motorID].angle = getMotorAngle(motorID);
			sMotorFeedData_g[motorID].velocity = getMotorVelocity(motorID);
			sMotorFeedData_g[motorID].torque = getMotorTorque(motorID);
			break;

    }
		if(update==1)
		{
			//sendChannelData(USB_Vehicle,USB_AUTO_INFO,sizeof(PlatformFeed_s),&sPlatformFeed_g);
			update=0;
		}

}

/* 私有函数部分*/
static uint16_t genCanId(uint8_t motorId)
{
		return 0x200;
}
//setCanHeader_ FDCAN header
static void setCanHeader_(CAN_TxHeaderTypeDef *header, uint16_t MsgID,uint32_t dataNum )
{
	header->StdId = 0x200;
	header->IDE = CAN_ID_STD;
	header->RTR = CAN_RTR_DATA;
	header->DLC = 0x08;

}
//设置标准can头
static void setCanHeader(CAN_TxHeaderTypeDef *header, uint16_t MsgID,uint32_t dataNum )
{
    header->StdId = MsgID;
    header->DLC = dataNum;
    header->ExtId = 0x00;
    header->IDE = CAN_ID_STD;
    header->RTR = CAN_RTR_DATA;
}
//计算ID后设置头
static void setCanHeaderXID(CAN_TxHeaderTypeDef *header,uint32_t dataNum )
{
    header->StdId = genCanId(0);
    header->DLC = dataNum;
    header->ExtId = 0x00;
    header->IDE = CAN_ID_STD;
    header->RTR = CAN_RTR_DATA;
}


////////////////////////

static bool setCurrentData(int16_t SteerAng)
{

}
//参数:传入8字节的数据/4个电机，电流
static uint8_t sendCurrentCmd( int16_t sendDatas[])
{
	CAN_TxHeaderTypeDef header;
	setCanHeader(&header,CONTROL_CANID, DATANUM);
	//setCurrentData();
	uint8_t data[8]={0};
	for(int8_t i=0;i<MOTOR_NUM;i++){
		LimitBoundary(sendDatas[i],MAX_CURRENT,MIN_CURRENT);
		data[2*i] = sendDatas[i]>>8;
		data[2*i+1] = sendDatas[i];
		
	}
	pushTxMsg0(&header, data);
	
}

/*接收

*/
static float getMotorAngle (uint8_t motorID){
	 return sTempMotorFeedData_[motorID].angle/360+0.5; //0~8191数字量对应0~360度
}
//需要除以10（手册中没有说明）
static float getMotorVelocity (uint8_t motorID){
	 return sTempMotorFeedData_[motorID].velocity/10.0f; //rpm
}
static float getMotorTorque (uint8_t motorID){
	 return sTempMotorFeedData_[motorID].torque; //0~8191数字量对应0~360度
}

float getRobotV(float *speedV,float* speedW){
	//rpmWheel[1] = (tempSpeed * 1000 / 60 + PLATFORM_WHEEL_LEN / 2 * tempOmega) / PI / PLATFORM_WHEEL_DIA * 60;  //右轮
	static float speed,tempSpeed,tempV,tempW;
	tempV=(-sTempMotorFeedData_[0].velocity+sTempMotorFeedData_[1].velocity); //左轮取反
	tempSpeed = tempV*PI*PLATFORM_WHEEL_DIA/60/PLATFORM_GEAR_RATIO/1000/10; //速度 m/s; 反馈的转速需要除以10
	
	tempW = (-sTempMotorFeedData_[0].velocity-sTempMotorFeedData_[1].velocity)*PI*PLATFORM_WHEEL_DIA/60/PLATFORM_GEAR_RATIO/10/PLATFORM_WHEEL_LEN;
	//printf("VL = %d, VR = %d, tempW=%3.2f，tempSpeed=%3.2f\r\n",
		//sTempMotorFeedData_[0].velocity,sTempMotorFeedData_[1].velocity,tempW,tempSpeed);
	*speedV = tempSpeed;
	*speedW =  tempW;
	
	return speed;
		
}



#endif




