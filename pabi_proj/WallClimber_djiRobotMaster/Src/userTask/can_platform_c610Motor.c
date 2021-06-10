#include "cmsis_os.h"
#include "can_platform_c610Motor.h"
#include "can_handler.h"
#include <string.h>
#include "motor_ctrl_task.h"
#include "pid.h"
#define cMIN(a, b) (((a) > (b)) ? (b) : (a))
#define cMAX(a, b) (((a) > (b)) ? (a) : (b))
#define absLimiter(n, l) (n > 0 ? cMIN(n, l) : cMAX(n, -l))
#define cABS(a)	   (((a)>=0)?(a):(-(a)))
#define getbit(x,y)   ((x) >> (y)&1) //获取某一位的值
#define setbit(x,y) ((x)|=(1<<y)); //设置某一位
#define resetbit(x,y) ((x)&=(~(1<<y))) //
#define  LimitBoundary(x,max,min) do{\
    if(x>max) x=max;\
    if(x<min) x=min;\
    x=x;\
    }while(0)


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
// 输入速度命令 rpm，电机数量MOTOR_NUM决定
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
float getWheelVelocity (uint8_t motorID){
	 return (sTempMotorFeedData_[motorID].velocity/10.0f * PI* PLATFORM_WHEEL_DIA/60.0/1000); //轮子线速度m/s
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





//int16_t PidCtrl(uint8_t id, int16_t set_speed);
//int16_t PidCtrl(uint8_t id, int16_t set_speed) {
//  const float kp = 0.58;
//  const float ki = 0.066;
//  const float kd = 0.6;

//  static float i_speed[2] = {0,0};
//  static int16_t last_speed[2] = {0,0};

//  int16_t cur_speed = motor_chassis[id].speed_rpm;
//  int16_t delta = set_speed - cur_speed;
//  i_speed[id] += ki * (float)delta;
//  if (i_speed[id] > I_SPEED_MAX)
//    i_speed[id] = I_SPEED_MAX;
//  else if (i_speed[id] < -I_SPEED_MAX)
//    i_speed[id] = -I_SPEED_MAX;
//  float f_current =
//      kp * (float)delta + i_speed[id] + kd * (float)(cur_speed - last_speed[id]);

//  int16_t set_current = (int16_t)f_current;

//  if (set_current > CURRENT_MAX)
//    set_current = CURRENT_MAX;
//  else if (set_current < -CURRENT_MAX)
//    set_current = -CURRENT_MAX;
//  last_speed[id] = cur_speed;
//	return set_current;
//}
//void testMotorControl() {
//	int16_t motor_cur[2];
//	int cnt = 0;
//    //CAN_cmd_chassis(300, 500, 0, 0);
//    //usart6_printf("deg:%d spd:%d tr:%d\r\n", motor_chassis[0].ecd,
//    //              motor_chassis[0].speed_rpm, motor_chassis[0].given_current);
////		motor_cur[0] = PidCtrl(0,8000);
////		motor_cur[1] = PidCtrl(1,4000);
//	  	motor_cur[0] = 1000;
//		motor_cur[1] = 1000;
//		CAN_cmd_chassis(motor_cur[0], motor_cur[1], 0, 0);
//		if(cnt%10==0)
//		{
//			//usart6_printf("spd1:%d spd2:%d s_c1:%d\r\n", motor_chassis[0].speed_rpm,
//                  //motor_chassis[1].speed_rpm, motor_cur[0]);
//		}
//    //osDelay(10);
//		cnt ++;
//  
//}
//void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3,
//                     int16_t motor4) {
//  uint32_t send_mail_box;
//  chassis_tx_message.StdId = 0x200;
//  chassis_tx_message.IDE = CAN_ID_STD;
//  chassis_tx_message.RTR = CAN_RTR_DATA;
//  chassis_tx_message.DLC = 0x08;
//  chassis_can_send_data[0] = motor1 >> 8;
//  chassis_can_send_data[1] = motor1;
//  chassis_can_send_data[2] = motor2 >> 8;
//  chassis_can_send_data[3] = motor2;
//  chassis_can_send_data[4] = motor3 >> 8;
//  chassis_can_send_data[5] = motor3;
//  chassis_can_send_data[6] = motor4 >> 8;
//  chassis_can_send_data[7] = motor4;
//						 
//// HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data,
////&send_mail_box);

//}


