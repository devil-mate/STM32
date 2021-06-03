#include "platform.h"

#if (KEYA_MOTOR == 0)
#include "motor.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_conf.h"

#include "common.h"
#include "motor_keya.h"
#include "protocol_type.h"

#define getMotorId(header)   (((header.StdId & 0x000F) ) - 1)

static uint16_t genCanId(MotorIndex motorId);
static void     setCanHeader(CAN_TxHeaderTypeDef *header, MotorIndex motorId);
static void     stopMotor(MotorIndex motorId);
static void     diaenableMotor(MotorIndex motorId);
static void     switchOnMotor(MotorIndex motorId);
static void     enableOnMotor(MotorIndex motorId);
static void     setMotorSpeedCmd(MotorIndex motorId, int32_t speed);
static void     setFeedbackInteval(MotorIndex motorId, uint8_t motorInfoInterval, uint8_t miscInterval);
static uint8_t  onlineState[2] = {0, 0};

/*motor.h公共函数部分*/
uint8_t initAllMotor()
{
    stopMotor(MotorL);//0x06停止
    stopMotor(MotorR);
		diaenableMotor(MotorL);//失能
    osDelay(200);

//    switchOnMotor(MotorL);//0x07 switch on 
//    switchOnMotor(MotorR);
//    osDelay(200);

//    setFeedbackInteval(MotorL, 100, 0);
//    setFeedbackInteval(MotorR, 100, 0);
	
	  enableOnMotor(MotorL);//0x0F enable
    enableOnMotor(MotorR);
    osDelay(300);
    
    
//    onlineCheck(MotorL);
//    onlineCheck(MotorR);
    osDelay(100);
//    
  return onlineState[0] && onlineState[1];
	//return 0;
}

uint8_t resetAllMotor()
{
    return initAllMotor();
}

uint8_t setMotorSpeed()
{
    int32_t motorRPM[2];
   
    motorRPM[MotorL] = (int16_t)(wheelRpmCmd.LeftRPM  * PLATFORM_GEAR_RATIO);
    motorRPM[MotorR] = (int16_t)(wheelRpmCmd.RightRPM * PLATFORM_GEAR_RATIO);
		
    setMotorSpeedCmd(MotorL, motorRPM[MotorL]);
    setMotorSpeedCmd(MotorR, motorRPM[MotorR]);
    
    return 1;
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

void resolveFeedback(CanRxMsg_s *msg)
{
    int16_t realCurrent,realSatusWord;
    int32_t realVelocity,realPosition;
    // int32_t realPosition;
    
    uint8_t motorId = getMotorId(msg->header);
    uint8_t func = msg->header.StdId >> 4;
    switch (func)
    {
    case 0x28:
				realPosition=(msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | msg->data[0];
				
		
        break;
    case 0x18:
        realCurrent = (msg->data[1] << 8) | msg->data[0];
        //realVelocity = (msg->data[2] << 8) | msg->data[3];
        realVelocity = (msg->data[5] << 24) | (msg->data[4] << 16) | (msg->data[3] << 8) | msg->data[2];
				realSatusWord = (msg->data[7] << 8) | msg->data[6];
		
        RobotStateFrame.motionState.wheelRPM[motorId] = realVelocity / PLATFORM_GEAR_RATIO; //motor RPM to wheel RPM
        RobotStateFrame.elecInfo.motorCurrent[motorId] = realCurrent / 1000.0f; // mA to Amp
        break;
    default:
        break;
    }
		if (func==0x16 | func==0x26){
			onlineState[motorId] = 0;
		}
}

/*私有函数部分*/
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

static void stopMotor(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();//0x06停止

    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
}
static void diaenableMotor(MotorIndex motorId)
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
    data[4] = 0x0C; 								//使能
    setCanHeader(&header, motorId);
    pushTxMsg0(&header, data);
}

static void setMotorSpeedCmd(MotorIndex motorId, int32_t speed)  //speed 单位rpm
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = speedData();
    setCanHeader(&header, motorId);
	
//    data[7] = (uint8_t)((speed >> 24) & 0xff);
//    data[6] = (uint8_t)((speed >> 16) & 0xff);
    data[4] = (uint8_t)((speed >> 8) & 0xff);
    data[5] = (uint8_t)(speed & 0xff);
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
