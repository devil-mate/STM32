#include "user_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"

#include "communication_up.h"
#include "communication_down.h"
#include "motion_controller.h"



//extern  TaskHandle_t VideoCamControllerTaskHandle;

void userTaskInit()
{
	
	initCommunication_up();
	initCommunication_down();
	initMotionController();

    TaskStatus_t pxTaskStatus; BaseType_t xGetFreeStackSpace; eTaskState eState;

    while(1)
    {
        osDelay(500);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
//		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_11);
        //vTaskGetInfo(VideoCamControllerTaskHandle, &pxTaskStatus, xGetFreeStackSpace, eState);

    }
}
