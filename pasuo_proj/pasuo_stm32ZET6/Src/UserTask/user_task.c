#include "user_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"

#include "upper_comm.h"
#include "can_handler.h"
#include "motion_controller.h"
//#include "IMUData.h"
//#include "IICData.h"
#include "rs485_ultrasonic.h"
#include "adcTask.h"
#include "readData.h"
//extern  TaskHandle_t VideoCamControllerTaskHandle;

void userTaskInit()
{
	//
	initCommunication();  //1024*2 £¬8k
	initCANHandler();		// 1024*2 £¬£¬ 8k
	initMotionController();  // 2048 £¬	8k
	initReadData();			//512 £¬2k
	initRS485Data();		//512 2k
	IniAdc();				//256 1k
	


    TaskStatus_t pxTaskStatus; BaseType_t xGetFreeStackSpace; eTaskState eState;

    while(1)
    {
        osDelay(500);
        //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
		//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
		//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_11);
        //vTaskGetInfo(VideoCamControllerTaskHandle, &pxTaskStatus, xGetFreeStackSpace, eState);

    }
}
