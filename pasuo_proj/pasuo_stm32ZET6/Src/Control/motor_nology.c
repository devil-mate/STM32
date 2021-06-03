#include "platform.h"
#if (NOLOGY_MOTOR == 0)
#include "common.h"
#include "motor.h"
#include "motor_nology.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_conf.h"
#include "protocol_type.h"

#define getMotorId(header)   (((header.StdId & 0x00F0) >> 4) - 1)

static uint16_t genCanId(MotorIndex motorId, NologyFuncCode func);
static void     setCanHeader(CAN_TxHeaderTypeDef *header, MotorIndex motorId, NologyFuncCode func);
static void     resetModule(MotorIndex motorId);
static void     setDrvMode(MotorIndex motorId, NologyDrvMode mode);
static void     setMotorSpeedCmd(MotorIndex motorId, uint16_t pwmLimit, int16_t speed);
static void     setFeedbackInteval(MotorIndex motorId, uint8_t motorInfoInterval, uint8_t miscInterval);
static uint8_t  onlineState[2] = {0, 0};

/*motor.h公共函数部分*/
uint8_t initAllMotor()
{
    resetModule(MotorL);
    resetModule(MotorR);
    osDelay(600);

    setDrvMode(MotorL, Velocity_Mode);
    setDrvMode(MotorR, Velocity_Mode);
    osDelay(600);

    setFeedbackInteval(MotorL, 100, 0);
    setFeedbackInteval(MotorR, 100, 0);
    osDelay(200);
    
    onlineCheck(MotorL);
    onlineCheck(MotorR);
    osDelay(100);
    
    return onlineState[0] && onlineState[1];
}

uint8_t resetAllMotor()
{
    return initAllMotor();
}

uint8_t setMotorSpeed()
{
    int16_t motorRPM[2];
   
    motorRPM[MotorL] = (int16_t)(wheelRpmCmd.LeftRPM  * PLATFORM_GEAR_RATIO);
    motorRPM[MotorR] = (int16_t)(wheelRpmCmd.RightRPM * PLATFORM_GEAR_RATIO);

    setMotorSpeedCmd(MotorL, MAX_PWM_LIMIT, motorRPM[MotorL]);
    setMotorSpeedCmd(MotorR, MAX_PWM_LIMIT, motorRPM[MotorR]);
    
    return 1;
}

uint8_t onlineCheck(MotorIndex motorId)
{
    onlineState[motorId] = 0;   //重置在线状态

    //发送检测帧
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();

    setCanHeader(&header, motorId, Func_OnlineCheck);
    pushTxMsg0(&header, data);

    // 检测是否置1，超时判断离线
    uint32_t t = 0;
    while(onlineState[motorId] == 0 && t++ < ONLINE_CHECK_TIMEOUT)
    {
        osDelay(1);
    }
    return onlineState[motorId] == 1;
}

void resolveFeedback(CanRxMsg_s *msg)
{
    int16_t realCurrent;
    int16_t realVelocity;
    // int32_t realPosition;
    
    uint8_t motorId = getMotorId(msg->header);
    uint8_t func = msg->header.StdId & 0x0F;
    switch (func)
    {
    case Func_OnlineCheck:
        if (msg->header.StdId == genCanId(MotorL, Func_OnlineCheck))
            onlineState[MotorL] = 1;
        else if (msg->header.StdId == genCanId(MotorR, Func_OnlineCheck))
            onlineState[MotorR] = 1;
        break;
    case Func_Fb_CurrVelocity:
        realCurrent = (msg->data[0] << 8) | msg->data[1];
        realVelocity = (msg->data[2] << 8) | msg->data[3];
        // realPosition = (msg->data[4] << 24) | (msg->data[5] << 16) | (msg->data[6] << 8) | msg->data[7];
        
        RobotStateFrame.motionState.wheelRPM[motorId] = realVelocity / PLATFORM_GEAR_RATIO; //motor RPM to wheel RPM
        RobotStateFrame.elecInfo.motorCurrent[motorId] = realCurrent / 1000.0f; // mA to Amp
        break;
    default:
        break;
    }
    onlineState[0] = 1;
    onlineState[1] = 1;
}

/*私有函数部分*/
static uint16_t genCanId(MotorIndex motorId, NologyFuncCode func)
{
    return    func | MOTOR_GROUP_NUM << 8 | (motorId + 1) << 4;
}

static void setCanHeader(CAN_TxHeaderTypeDef *header, MotorIndex motorId, NologyFuncCode func)
{
    header->StdId = genCanId(motorId, func);
    header->DLC = 8;
    header->ExtId = 0x00;
    header->IDE = CAN_ID_STD;
    header->RTR = CAN_RTR_DATA;
}

static void resetModule(MotorIndex motorId)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();

    setCanHeader(&header, motorId, Func_Reset);
    pushTxMsg0(&header, data);
}

static void setDrvMode(MotorIndex motorId, NologyDrvMode mode)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();

    data[0] = mode;

    setCanHeader(&header, motorId, Func_SetMode);
    pushTxMsg0(&header, data);
}

static void setMotorSpeedCmd(MotorIndex motorId, uint16_t pwmLimit, int16_t speed)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();
    setCanHeader(&header, motorId, Func_Cmd_VelocityMode);
    data[0] = (uint8_t)((pwmLimit >> 8) & 0xff);
    data[1] = (uint8_t)(pwmLimit & 0xff);
    data[2] = (uint8_t)((speed >> 8) & 0xff);
    data[3] = (uint8_t)(speed & 0xff);

    pushTxMsg0(&header, data);
}

/**
 * @description: 设置[实时电流、速度、位置值]及混杂项反馈间隔时间(ms)
 * @param {type} 
 * @return: 
 */
static void setFeedbackInteval(MotorIndex motorId, uint8_t motorInfoInterval, uint8_t miscInterval)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = initData();
    setCanHeader(&header, motorId, Func_Fb_Config);
    data[0] = motorInfoInterval;
    data[1] = miscInterval;

    pushTxMsg0(&header, data);
}
#endif
