/*旋转电机 modbus协议*/

#include "rotaArm_controller.h"

#include "cmsis_os.h"
#include <string.h>
#include "common.h"
#include "modbus.h"
#include "motor_Kinco.h"

static TaskHandle_t RotaArmControllerTaskHandle = NULL;
static void RotaArmControllerTask(void *argument);


void initRotaArmController()
{
    //initRotaMotor();
    xTaskCreate(RotaArmControllerTask,         	/* 任务函数  */
                "MotionControllerTask",       	/* 任务名    */
                1024,                         	/* 任务栈大小，单位word，也就是4字节 */
                NULL,                         	/* 任务参数  */
                1,                            	/* 任务优先级*/
                &RotaArmControllerTaskHandle); 	/* 任务句柄  */
}

static void RotaArmControllerTask(void *argument)
{		
		osDelay(200);
		initRotaMotor();
	
    static uint8_t reseted = 0;
    while (1)
    {
      osDelay(20);
			uint16_t state=0;
			state=ReadDWordFromSlave(MB_REG_SPEED);
			osDelay(10);
			printf("state:%x",state);
			int32_t data=60*512*10000/1875;//60rpm
			WriteDWordToSlave(MB_REG_SPEED,data);//速度变量
        //copyGlobal2Local();
//        if (LocalMoveCmd.digit != 0x02 && reseted)
//        {
//            reseted = 0;
//        }
//        switch (LocalMoveCmd.digit)
//        {
//        case 0x00:
////						float pos=3.0;
////            setMotorAngle();
//            break;
//        case 0x01:
//            float pos=3.0;
//            setMotorAngle(pos);
//            break;
//        case 0x02:
//            
//            if (!reseted)
//            {
//                log_info("Reset all motor...\r\n");
//                resetAllMotor();
//                memset(&MoveCmd, 0, sizeof(MoveCmd));
//                reseted = 1;
//            }
//            break;
//        default:
//            break;
//        }
    }
}

//static void copyGlobal2Local(void)
//{
//    taskENTER_CRITICAL();
//    memcpy(&LocalMoveCmd, &MoveCmd, sizeof(MoveCmd));
//    memcpy(&LocalLidarInfo, &LidarInfo, sizeof(LidarInfo));
//    memcpy(&LocalPidParamCmd, &PidParamCmd, sizeof(PidParamCmd));
//    taskEXIT_CRITICAL();
//}

//static void resolveWheelRpm()
//{
//	LocalMoveCmd.speed = absLimiter(LocalMoveCmd.speed, PLATFORM_MAX_SPEED_M_PER_MIN);
//	LocalMoveCmd.omega = absLimiter(LocalMoveCmd.omega, PLATFORM_MAX_OMEGA);

//	wheelRpmCmd.LeftRPM  = (LocalMoveCmd.speed * 1000 / 60 - PLATFORM_WHEEL_TREAD / 2 * LocalMoveCmd.omega) / PI / PLATFORM_WHEEL_DIA * 60;
//	wheelRpmCmd.RightRPM = (LocalMoveCmd.speed * 1000 / 60 + PLATFORM_WHEEL_TREAD / 2 * LocalMoveCmd.omega) / PI / PLATFORM_WHEEL_DIA * 60;
//}
