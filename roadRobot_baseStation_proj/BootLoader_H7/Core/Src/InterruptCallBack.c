#include "main.h"
#include "USBDriver.h"
#include "FlyProtocol.h"
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim==&htim13)
		{
			StatusWatcherTask(NULL);
			flyProtocolTick();
		}
		else if(htim==&htim14) 
		{
			sendUSBBuf();
		}
}
