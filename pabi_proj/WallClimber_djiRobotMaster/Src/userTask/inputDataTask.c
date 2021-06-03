#include "inputDataTask.h"
#include "cmsis_os.h"
#include "protocol_type.h"
#include "stdbool.h"
#include "common.h"
#include "main.h"

static void InputDataTask(void * argument);
static TaskHandle_t HandleAdcTaskIF = NULL;

#define SHUTDOWN_IN_PORT GPIOI 
#define SHUTDOWN_IN_PIN GPIO_PIN_9
#define	SHUUTDOWN_FINISH_PIN  GPIO_PIN_8
#define SHUTDOWN_OUT_PORT GPIOA 
#define SHUTDOWN_OUT_PIN GPIO_PIN_4 
bool g_shutdownFlag=false;
extern bool gCanErrorFlag;
static bool  readPin(GPIO_TypeDef* port, uint16_t pin, uint32_t filter_millisec );
void InitInputData()
{
	xTaskCreate( InputDataTask,   	/* 任务函数  */
							 "AdcTaskIF",     	/* 任务名    */
							 128,               	/* 任务栈大小，单位word，也就是4字节 */
							 NULL,              	/* 任务参数  */
							 8,                 	/* 任务优先级*/
							 &HandleAdcTaskIF );  /* 任务句柄  */
}


static void InputDataTask(void * argument)
{
//	adc_DmaInit();
//	//TickType_t xLastWakeTime;
//	//xLastWakeTime = xTaskGetTickCount();
//	HAL_ADC_Start_IT(&hadc1);
	HAL_GPIO_WritePin(VCC_OUT2_GPIO_Port,VCC_OUT2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(VCC_OUT3_GPIO_Port,VCC_OUT3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(VCC_OUT4_GPIO_Port,VCC_OUT4_Pin,GPIO_PIN_SET);
	while(1)
	{
		
		osDelay(300);
		//printf("g_shutdownFlag:%d \r\n",g_shutdownFlag);
		//HAL_ADC_Start_IT(&hadc1);
			//关机命令
//		if(gCanErrorFlag==true){
//			osDelay(300);
//			if(gCanErrorFlag==true){
//				
//			}else{
//			}
//		}
		//||gCanErrorFlag==true 
//		HAL_GPIO_TogglePin(SHUTDOWN_OUT_PORT,SHUTDOWN_OUT_PIN);
		if(readPin(SHUTDOWN_IN_PORT,SHUTDOWN_IN_PIN,500) ){
			//printf("get port in ---SHUTDOWN_IN_PIN \r\n");
			
			setbit(StateFeed_promptlys.digit,11); //关机信号
			
			g_shutdownFlag=true;
			//disenableAllMotor();
			osDelay(3000);
			resetbit(StateFeed_promptlys.digit,11);
			HAL_GPIO_WritePin(SHUTDOWN_OUT_PORT,SHUTDOWN_OUT_PIN,GPIO_PIN_SET);
			g_shutdownFlag=false;
	//		//强制关机
	//		osDelay(5000);
	//		if(readPin(SHUTDOWN_IN_PORT,SHUTDOWN_IN_PIN,200)){
	//			osDelay(2000);
	//			HAL_GPIO_WritePin(SHUTDOWN_OUT_PORT,SHUTDOWN_OUT_PIN,GPIO_PIN_SET);
	//		}
		}
		else{
			 //关机信号
			//HAL_GPIO_WritePin(SHUTDOWN_OUT_PORT,SHUTDOWN_OUT_PIN,GPIO_PIN_RESET);
		}
		//linux关机完成IO信号
	//	if(readPin(SHUTDOWN_IN_PORT,SHUUTDOWN_FINISH_PIN,500)){
	//		HAL_GPIO_WritePin(SHUTDOWN_OUT_PORT,SHUTDOWN_OUT_PIN,GPIO_PIN_SET);
	//	}
		
	
		
	}
}
static bool  readPin(GPIO_TypeDef* port, uint16_t pin, uint32_t filter_millisec )
{
	bool  bitstatus = false;
	if(HAL_GPIO_ReadPin(port,pin)){
		osDelay(filter_millisec);
		if(HAL_GPIO_ReadPin(port,pin)){
			bitstatus = true;	    
		}
	}
	return bitstatus;
}

//void adc_DmaInit(void) 
//{
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_Start_DMA(&hadc1, ADCBuf, 7);
//	
//}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //ADC转换完成回调函数(规则通道？)。
//{
//	HAL_ADC_Stop_IT(&hadc1);	
////	adcSum[0] += (float)(ADCBuf[0] - adcAvg[0]);
////	adcAvg[0] = (float)(adcSum[0] / 100);
////	adcVal[0] = (float)(adcAvg[0] * 0.082); 
////	
////	//Driver1
////	adcSum[1] += (float)(ADCBuf[1] - adcAvg[1]);
////	adcAvg[1] = (float)(adcSum[1] / 100);
////	adcVal[1] = (float)(adcAvg[1] * 0.06); 
////	
////	//Driver2
////	adcSum[2] += (float)(ADCBuf[2] - adcAvg[2]);
////	adcAvg[2] = (float)(adcSum[2] / 100);
////	adcVal[2] = (float)(adcAvg[2] * 0.07); 
//	
//	//battery senser
////	if(ADCBuf[6]<6)
////		voltage=(ADCBuf[6])/113.2;
////	else	
//	
//	//RobotStateFrame.elecInfo.batVoltage=(float)(ADCBuf[0])/4096*3.3*167/10; //电阻157K+10K分压？两个电阻各是多少
//	//RobotStateFrame.elecInfo.batVoltage=(float)(ADCBuf[0])/113.3; //实际测出的系数

//	
//	//adcSum[6]=ADCBuf[6]*100;
//	//printf("ADCBuf[0]: %d , ADCBuf[1]: %d , ADCBuf[2]: %d, ADCBuf[3]: %d , ADCBuf[4]: %d , ADCBuf[5]: %d, ADCBuf[6]: %d\r\n",
//		//ADCBuf[0],ADCBuf[1],ADCBuf[2],ADCBuf[3],ADCBuf[4],ADCBuf[5],ADCBuf[6]);
//	//printf("+++++++++voltage:%3.2f   \r\n",RobotStateFrame.elecInfo.batVoltage);
//	//memcpy(&ModData.ADC_v, adcVal,sizeof(adcVal));
//	
//}
/* CAN总线错误回调函数，这里发生CAN总线错误后必须重启接收中断，否则会导致CAN接收异常 >*/

