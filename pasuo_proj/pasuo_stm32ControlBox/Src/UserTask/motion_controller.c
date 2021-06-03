#include "motion_controller.h"
#include "cmsis_os.h"
#include "platform.h"
#include "protocol_type.h"
#include "stm32f4xx_hal_iwdg.h"

#define  PUSH_BUTTON GPIOE
#define  FORWARD GPIO_PIN_7
#define  EMERGENCY GPIO_PIN_8
#define  BACKWARD GPIO_PIN_9
#define  STOP GPIO_PIN_10

#define  MANUAL_MODE GPIO_PIN_11
#define  AUTO_MODE GPIO_PIN_12
#define  RESET_MODE GPIO_PIN_13





//////////����

static void motionControllerTask(void *argument);
static TaskHandle_t MotionControllerTaskHandle = NULL;
static void setControlDigit();
static void getKey();
static bool key_scan(GPIO_TypeDef *GPIO,uint16_t GPIO_PIN,bool currentState);


	
//////////����

static Flag_s 				localFlag;

static ControlCmd_s localControlCmds;
//static Queue_Message_s localQueueMessage;
extern IWDG_HandleTypeDef hiwdg;
static uint16_t tempDigit = 0;
typedef struct{
	bool autoMode;
	bool resetMode;
	bool mannualMode;
	bool forward;
	bool backward;
	bool stop;
	bool emergency;
}ButtenPush_s;
enum controlDigitEnum{
	manual_mode =	1,
	auto_mode 	=	2,
	reset_mode	=	3,
	temp04		= 	4,
	temp05		=	5,
	forward 	=	6,
	backward	= 	7,
	temp08		= 	8,
	emergency	=	9,
	stop		=	10,
	
};
static ButtenPush_s buttonPush_s;
//float gOutTorque=0.0;

//localFlagʹ��

//ModeStateFSM_e eModeState;
//AutoStateFSM_e eAutoState;
void feedback_mode();
void initMotionController()
{
	
	memset(&localControlCmds,0,sizeof(localControlCmds));
	memset(&buttonPush_s,0,sizeof(buttonPush_s));

    xTaskCreate(motionControllerTask,         /* ������  */
                "MotionControllerTask",       /* ������    */
                2048,                         /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                         /* �������  */
                10,                            /* �������ȼ�*/
                &MotionControllerTaskHandle); /* ������  */
}

static void motionControllerTask(void *argument)
{	
//	InitProtocolFrame(localControlCmds);
//    localControlCmds.funcCode = Func_MoveCmd;
    while (1)
    {	
		//TODO ���Ź�
		osDelay(100);
		getKey();
		setControlDigit(); 

	}
		
			

}
//TODO ö��
void getKey(){

	//����0 �����������ϴ�״̬����1 Ϊ���룬2������
	if(key_scan(PUSH_BUTTON,AUTO_MODE,buttonPush_s.autoMode)){
		buttonPush_s.autoMode= true;
		buttonPush_s.mannualMode= false;
		buttonPush_s.resetMode= false;
	}else if(key_scan(PUSH_BUTTON,RESET_MODE,buttonPush_s.resetMode)){
		buttonPush_s.autoMode= false;
		buttonPush_s.mannualMode= false;
		buttonPush_s.resetMode= true;
	}else {
		buttonPush_s.autoMode= false;
		buttonPush_s.mannualMode= true;
		buttonPush_s.resetMode= false;
	}
	
	//6ǰ��  	//7���ˣ�
	if(key_scan(PUSH_BUTTON,FORWARD,buttonPush_s.forward)){
		buttonPush_s.forward= true;
	}
	else{
		buttonPush_s.forward= false;
	}
	if(key_scan(PUSH_BUTTON,BACKWARD,buttonPush_s.backward)){
		buttonPush_s.backward= true;
	}
	else {
		buttonPush_s.backward= false;
	}

	//9��ͣ
	if(key_scan(PUSH_BUTTON,EMERGENCY,buttonPush_s.emergency)){
		buttonPush_s.emergency= true;
	}else {
		buttonPush_s.emergency= false;
	}
	//10ֹͣ
	if(key_scan(PUSH_BUTTON,STOP,buttonPush_s.stop)){
		buttonPush_s.stop= true;
	}else{
		buttonPush_s.stop= false;
	}

}
void setControlDigit(){
//	taskENTER_CRITICAL();
//	memcpy(&localControlCmds, &controlCmds, sizeof(controlCmds));
//	taskEXIT_CRITICAL();
	
	if(buttonPush_s.autoMode){
		setbit(localControlCmds.digit,2);
		resetbit(localControlCmds.digit,1);
		resetbit(localControlCmds.digit,3);
	}else if(buttonPush_s.resetMode){
		setbit(localControlCmds.digit,3);
		resetbit(localControlCmds.digit,2);
		resetbit(localControlCmds.digit,1);
	}else{
		setbit(localControlCmds.digit,1);
		resetbit(localControlCmds.digit,2);
		resetbit(localControlCmds.digit,3);
	}
	
	//6ǰ�� 
	if(buttonPush_s.forward){
		setbit(localControlCmds.digit,forward);
	}else{
		resetbit(localControlCmds.digit,forward);
	}
	//7���ˣ�
	if(buttonPush_s.backward){
		setbit(localControlCmds.digit,backward);
	}else{
		resetbit(localControlCmds.digit,backward);
	}
	//9��ͣ
	if(buttonPush_s.emergency){
		setbit(localControlCmds.digit,emergency);
	}else{
		resetbit(localControlCmds.digit,emergency);
	}
	//10ֹͣ
	if(buttonPush_s.stop){
		setbit(localControlCmds.digit,stop);
	}else{
		resetbit(localControlCmds.digit,stop);
	}

	taskENTER_CRITICAL();
	controlCmds.digit =localControlCmds.digit;
	//memcpy(&controlCmds, &localControlCmds, sizeof(localControlCmds));
	taskEXIT_CRITICAL();
	//printf("======controlCmds.digit: %d\r\n", controlCmds.digit);
	


}
static bool key_scan(GPIO_TypeDef *GPIO,uint16_t GPIO_PIN,bool currentState){
	//Ӳ����������⵽���뷵��true
	//����0 �����������ϴ�״̬����1 Ϊ���룬2������
	bool returnState = false;
	//���º�̧�� ��� 
	if(HAL_GPIO_ReadPin(GPIO,GPIO_PIN)==GPIO_PIN_RESET && currentState==false){
		osDelay(50);
		if(HAL_GPIO_ReadPin(GPIO,GPIO_PIN)==GPIO_PIN_RESET){
			returnState= true;
		}
		else{
			returnState=currentState;
		}
		
	}
	else if(HAL_GPIO_ReadPin(GPIO,GPIO_PIN)==GPIO_PIN_SET && currentState==true ){
		osDelay(50);
		if(HAL_GPIO_ReadPin(GPIO,GPIO_PIN)==GPIO_PIN_SET){
			returnState= false;
		}
		else{
			returnState=currentState;
		}
		
	}else{
		returnState = currentState;
	}
	return returnState;
}













