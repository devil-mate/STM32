#include "user_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
//#include "main.h"

#include "upper_comm.h"
#include "can_handler.h"
#include "IMUData.h"
//#include "IICData.h"
#include "rs485_ultrasonic.h"
#include "motion_controller.h"
#include "inputDataTask.h"
//#include "readData.h"
//extern  TaskHandle_t VideoCamControllerTaskHandle;

void user_taskInit()
{
	//
	initCommunication();  
	initCANHandler();		
	initMotionController();  
//	//initReadData();			
//	initRS485Data();		
	InitInputData();
	initIMUData();	
	


    TaskStatus_t pxTaskStatus; BaseType_t xGetFreeStackSpace; eTaskState eState;

    while(1)
    {
        osDelay(500);
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
//		HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
//		//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
//		//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_11);
//        //vTaskGetInfo(VideoCamControllerTaskHandle, &pxTaskStatus, xGetFreeStackSpace, eState);

    }
}
