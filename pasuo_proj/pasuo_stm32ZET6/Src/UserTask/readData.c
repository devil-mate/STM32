#include "readData.h"
#include "cmsis_os.h"
#include "can_handler.h"
#include "common.h"
#include "protocol_type.h"
#include "platform.h"
#include "motion_controller.h"
#include "motor.h"
#include "string.h"
#include "main.h"
//��С���վ���
#define CAPTURE_NUM_MIN 2000

//��ʱ��Ƶ��/��ʱ���ж�Ƶ�� Ϊ f = 84M / Prescaler / Period = 84000 000 / 84 /1000 = 1000Hz��
//MotorInfo_s MotorInfoFeeds;
// tim3 EB1 ������������tim4 eb2 ��ӱ�����
static TaskHandle_t encoderDataTaskkHandle = NULL;
static void encoderDataTask(void *argument);
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
int32_t OverflowCount=0;
extern RobotStateFeed_s    RobotStateFeed;

float milemeterCalculate(int32_t count);

//uint32_t tempCount=0;
void initReadData()
{
    //memset(&MotorInfoFeeds, 0, sizeof(MotorInfoFeeds));
    xTaskCreate(encoderDataTask,         /* ������  */
                "encoderDataTask",       /* ������    */
                512,                         /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                         /* �������  */
                9,                            /* �������ȼ�*/
                &encoderDataTaskkHandle); /* ������  */
}


static void encoderDataTask(void *argument)
{
	//static uint32_t picNum=0;
//	unsigned char chrTemp[30];
//	unsigned char str[100];
	//float  temp;
	
	//int tempCount=0;
	//float tempOdom=0,pictime=0;
	//int tcount=0;
	double totalOdom = 0; //ÿ�ο�������������̣��豸���������ݮ���м���/����

	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	
	uint8_t DirectionA=0, DirectionB=0;
	//realCaptureNumberA��+��-��CaptureNumberA ��+������ʼ���㡣
	uint32_t CaptureNumberA=0;
	int32_t CaptureNumberB=0;
	uint32_t realCaptureNumberA=0,realCaptureNumberB=0;
	static int32_t lastCapNum=0,lastCapNumB=0;
	uint32_t deltaCapNum =0 ,deltaCapNumB =0;
	uint32_t captureCounter=0;
	//���ü�������ʼֵ
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	while (1){
		osDelay(200);
//		DirectionA = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3); 
//		DirectionB = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);
//		CaptureNumberA =__HAL_TIM_GET_COUNTER(&htim3);
//		//����50000������ע�Ᵽ֤��תһ������С��5000.
//		
//		if(CaptureNumberA>50000) {
//			realCaptureNumberA = realCaptureNumberA - (65535-CaptureNumberA);
//		}
//		else{
//			realCaptureNumberA=realCaptureNumberA + CaptureNumberA;
//			taskENTER_CRITICAL();
//			onceTaskInfo_s.picOdomNum = onceTaskInfo_s.picOdomNum+ CaptureNumberA; //ֻ��/����
//			taskEXIT_CRITICAL();
//		}
//		
//		
//		taskENTER_CRITICAL();
//		deltaCapNum = onceTaskInfo_s.picOdomNum -lastCapNum;
//		taskEXIT_CRITICAL();
//		
		//
		
		//CaptureNumberB=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4)+ OverflowCount*65535;
		
		//CaptureNumberB=(uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
		uint16_t tempNumberB = __HAL_TIM_GET_COUNTER(&htim4);
		//+ ǰ������
		if(tempNumberB>5000){
			CaptureNumberB = CaptureNumberB-(65535-tempNumberB);

		}else{
			CaptureNumberB =  CaptureNumberB+ tempNumberB;
			deltaCapNum = cABS(CaptureNumberB-lastCapNum);
			//������������������
			taskENTER_CRITICAL();
			onceTaskInfo_s.picOdomNum = cABS(onceTaskInfo_s.picOdomNum+tempNumberB); //ֻ��/����
			taskEXIT_CRITICAL();
			

		}
		//printf("tempNumberB:%d ,CaptureNumberB: %d\r\n",tempNumberB,CaptureNumberB);
		
		
		
		//printf("deltaCapNum:%d ,lastCapNum: %d\r\n",deltaCapNum,lastCapNum);
		//milemeterCalculate(CaptureNumberB);
		
		//TODO ׼ȷ����
		//ParamConfigs.picDis=300;
		//captureCounter=ParamConfigs.picDis*PLATFORM_GEAR_RATIO/PI/PLATFORM_WHEEL_DIA/DIA_RATE*PULS_NUM; //DIA_RATE
		captureCounter=ParamConfigs.picDis/PI/CODER_WHEEL_DIA*PULS_NUM; //DIA_RATE
		//printf("captureCounter: %d\r\n",captureCounter);
//		printf("CaptureNumberB: %d  , deltaCapNumB:%d tempNumberB:%d, DirectionB=%d\r\n",
//			CaptureNumberB,deltaCapNumB,tempNumberB,DirectionB);
		//	//��¼��̼�Ƶ�ʿ���
		captureCounter = cMAX(captureCounter,CAPTURE_NUM_MIN);
		if(deltaCapNum>=captureCounter){
			//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14,GPIO_PIN_SET); //��������
			HAL_GPIO_WritePin(CAM_GPIO_Port, CAM_Pin,GPIO_PIN_SET); //��������
			float tempOdom = milemeterCalculate(onceTaskInfo_s.picOdomNum); //���
			taskENTER_CRITICAL();
			setbit(RobotStateFeed.digit,10);//��ݮ�ɴ����ź�/�����ź�
			RobotStateFeed.sPicOdom.odom= -tempOdom;
			taskEXIT_CRITICAL();
			
			//RobotStateFeed.realOdom = milemeterCalculate(realCaptureNumberA);
			RobotStateFeed.sPicOdom.time= onceTaskInfo_s.picNum;
			onceTaskInfo_s.picNum++;
			//lastCapNum = onceTaskInfo_s.picOdomNum;
			lastCapNum = CaptureNumberB;
			deltaCapNum=0;
		}
		//else if(deltaCapNum>=captureCounter+10){ 
		else{
			
//			taskENTER_CRITICAL();
//			resetbit(RobotStateFeed.digit,10);
//			taskEXIT_CRITICAL();
			//ѭ������100ms,����<100ms
			HAL_GPIO_WritePin(CAM_GPIO_Port, CAM_Pin,GPIO_PIN_RESET); //��λ
		}
		
		__HAL_TIM_SET_COUNTER(&htim3,0);
		__HAL_TIM_SET_COUNTER(&htim4,0);

//		//TODO----------��ȡ�����Ϣ,

		/***************printf******************/
//		bool tempTrriger =false;
//		
//		taskENTER_CRITICAL();
//		tempTrriger  = RobotStateFeed.digit & 0x400; 	
//		taskEXIT_CRITICAL();
//		printf("captureCounter: %d deltaCapNum:%d trrigerFlag: %d  digit:%d \r\n",
//					captureCounter,deltaCapNum,tempTrriger,RobotStateFeed.digit);

		/***************printf******************/
		
	}

}
float milemeterCalculate(int32_t count){
	static float dis=0;
	//dis=(float)(count)/PLATFORM_GEAR_RATIO*PI*PLATFORM_WHEEL_DIA*DIA_RATE/PULS_NUM;
	dis = (float)(count)/PULS_NUM*PI*CODER_WHEEL_DIA;
	printf("count:%d ,dis: %3.2f\r\n",count,dis);
	return dis;
}
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{

//	static int32_t counter=0,picNum=0;////////
//	static int32_t counter1=0,counter2=0;
//	static int32_t disCounter=0;
////	if(htim->Instance==TIM11){
////		if((HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12))){
////			counter++;
////			disCounter++;

////		}
////	}
////	
//	if(htim->Instance==TIM4){
//		if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)){
//			if(counter<1){
//				counter=1;
//			}			
//			counter--;
//			disCounter--;
//			
//		}
//	}
//	//��¼��̼�Ƶ�ʿ���
//	if(disCounter%1000){
//		RobotStateFeed.sPicOdom.odom= milemeterCalculate(disCounter); //���
//		RobotStateFeed.sPicOdom.time= picNum;	//ʱ��/��ǵ�
//		picNum++;
//	}
//	counter=cABS(counter);
//	if(counter>captureCounter){
//		setbit(RobotStateFeed.digit,10);//��ݮ�ɴ����ź�/�����ź�
//		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14,GPIO_PIN_SET); //��������
//		counter=0;
//	}else{
//		//resetbit(RobotStateFeed.digit,10);//��ݮ�ɴ����ź�/�����ź�
//		//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14,GPIO_PIN_RESET); //��λ
//	}

//	////////////////test////////////
////	if(counter>captureCounter+100){
////		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14,GPIO_PIN_RESET); //��λ
////		resetbit(RobotStateFeed.digit,10);//��ݮ�ɴ����ź�/�����ź�
////		counter=0;
////		
////	}
//	////////////////test////////////
//	//printf("disCounter%d   counter%d\r\n",disCounter,counter);

//	

//}




