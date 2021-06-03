#include "rs485_ultrasonic.h"
#define ULTRASNTIC 1

#if(ULTRASNTIC == 1)
#include <string.h>
#include "cmsis_os.h"
#include "common.h"
#include "protocol_type.h"

//#define calibration 0
//ä������ʼֵ������10mm
#define DisThreshold_min_ultra 20
#define DEV_COUNT 2 
#define MAX_DETECT_DIS 5.00*1000 //mm
#define DESENT_COUNT 10
#define OBSTACAL_COUNT 5

static TaskHandle_t rs485DataTaskHandle = NULL;
static void rs485DataTask(void *argument);

void obstacleDetection(const uint16_t *dis, const uint8_t devCount);
float avgDisFilter(float in, uint8_t depth, uint8_t reset);
float movAvgDisFilter(float in, uint8_t depth, uint8_t reset);
//VL53L0X_DeviceInfo_t vl53l0x_dev_info;//�豸ID�汾��Ϣ

static bool RS485_setAddrCmd();

MovAvgFilter_s DisMovAvgFilter[DEV_COUNT];//0ǰ��1��

static uint16_t RS485_ReadDataCmd(ultra_dev_s *dev,uint16_t len);


//uint8_t UART4_Rx_Buf[MAX_REC_LENGTH] = {0}; //USART4�洢��������

uint8_t  UART4_temp[REC_LENGTH] = {0};       //USART4�������ݻ���

QueueHandle_t      rs485RxMsgQueueHandle = NULL;

bool direcFlag[DEV_COUNT] ={false,false}; 
#define direcFlagF direcFlag[0] 
#define direcFlagB direcFlag[1] 
void initRS485Data()
{

    xTaskCreate(rs485DataTask,         /* ������  */
                "rs485DataTask",       /* ������    */
                512,                         /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                         /* �������  */
                10,                            /* �������ȼ�*/
                &rs485DataTaskHandle); /* ������  */
}


static void rs485DataTask(void *argument)
{
    

    //float  temp;
	//float a[3],w[3],h[3],Angle[3],temperature;
	float tempDistanceF=0,tempDistanceB=0;
	rs485RxMsgQueueHandle=xQueueCreate(1,sizeof(tempDistanceF));
	if(rs485RxMsgQueueHandle ==NULL){
		printf("create rs485RxMsgQueueHandle fail!");
		return;
	}
	
	//osDelay(1000);
//	VL53L0X_Error status = VL53L0X_ERROR_NONE;
//	memset(&sAdjust_Data,0,sizeof(sAdjust_Data));
//	memset(&sAdjust_writeData,0,sizeof(sAdjust_writeData));
//	
//	memset(&vl53l0x_data, 0, sizeof(vl53l0x_data));
//	memset(&vl53l0x_data2, 0, sizeof(vl53l0x_data2));
//	//TODO====
//	initDataStorage(11);//��ʼ��11���������һ����
//	osDelay(2000);
//	//��ʼ��
//	status=initDisSensor();


//	uint8_t sendBuf[3];
//	//sendBuf[0]=0x32; //������
//	sendBuf[0]=0xE8;//��ַ
//	sendBuf[1]=0x02;//�Ĵ���
//	sendBuf[2]=0xBC;//����
	static int8_t ultraDis_CountF=0,ultraDis_CountB=0;
	
	ultra_dev_s Forward_dev;
	ultra_dev_s Backward_dev;
	
	Forward_dev.buff[0]=FORWARD_ADDR;
	Forward_dev.buff[1]=0x02;
	Forward_dev.buff[2]=0xBC;
	
	Backward_dev.buff[0]=BACKWARD_ADDR;
	Backward_dev.buff[1]=0x02;
	Backward_dev.buff[2]=0xBC;
	
	uint16_t ultra_distance[DEV_COUNT];
	//HAL_UART_Receive_IT(&huart4,(uint8_t*)&bRxBuffer,1);//(USART1->DR);	//��ȡ���յ�������
	HAL_UART_Receive_IT(&huart4,(uint8_t *)UART4_temp,REC_LENGTH);
	
	RS485_RX_EN;

	movAvgFilterInit(&DisMovAvgFilter[0]);
	movAvgFilterInit(&DisMovAvgFilter[1]);
/***�޸ĵ�ַ*****/
//	while(!RS485_setAddrCmd()){
//		osDelay(100);
//		RS485_setAddrCmd();

//	};
//	printf("set addr success!\r\n");
//	return ;
/***�޸ĵ�ַ*****/
     while (1)
    {
			
        //osDelay(150);
		//TODO=======��ȡ�ɺ���
		//getDistance(ultra_distance);
//		if(RS485_send_flag==1){
		//ǰ������
		//printf("%x\r\n",55);
		ultra_distance[0] = 0;
		ultra_distance[1] = 0;
		//tempDistance=0;
		if(rs485RxMsgQueueHandle ==NULL){
			printf("rs485RxMsgQueueHandle is Null\r\n");
			continue;
		}
		RS485_ReadDataCmd(&Forward_dev,sizeof(Forward_dev.buff));//���������ݶ�ȡ����
		//0--11m���¶Ȳ�������ʱԼ87ms���ж��л�ȡ����,��ȡ
		if(xQueueReceive(rs485RxMsgQueueHandle,&tempDistanceF,150*portTICK_RATE_MS)!=pdPASS){
			ultraDis_CountF++;
			if(ultraDis_CountF>30){
				//���߱��
				sFlag.laserStateFlagF=true;
				ultraDis_CountF=0;	
			}
		}else{
			sFlag.laserStateFlagF=false;
			ultra_distance[0]=tempDistanceF; //�õ�����		
		}
		
		
		//�󴫸���
		//tempDistance=0;
		RS485_ReadDataCmd(&Backward_dev,sizeof(Backward_dev.buff));
		if(xQueueReceive(rs485RxMsgQueueHandle,&tempDistanceB,150*portTICK_RATE_MS)!=pdPASS){
			ultraDis_CountB++;
			if(ultraDis_CountB>30){
				sFlag.laserStateFlagB=true;
				ultraDis_CountB=0;
			}
			
		}
		else{
			sFlag.laserStateFlagB=0;
			ultra_distance[1]=tempDistanceB; //�õ�����
		}
		obstacleDetection(ultra_distance,DEV_COUNT);

	}



}
static uint16_t RS485_ReadDataCmd(ultra_dev_s *dev,uint16_t len)
{
	//����Ϊ����ģʽ���ߵõ�оƬ�������ͣ���������0x32
	
//		RS485_TX_EN;
//		if(HAL_UART_Transmit(&huart4, dev->buff,len,0xFF)!= HAL_OK){
//			return 0;
//		}
//		//HAL_UART_Transmit(&huart4, dev->buff,len,0xFF);
//		RS485_RX_EN;
	RS485_TX_EN;
	for(uint8_t i=0;i<len;i++){
		if(HAL_UART_Transmit(&huart4, dev->buff,len,0xFF)!= HAL_OK){
			return 0;
		}
		osDelay(2);
	}
	RS485_RX_EN;

	return len;


}
static bool RS485_setAddrCmd()
{
	//����Ϊ����ģʽ���ߵõ�оƬ�������ͣ���������0x32
	osDelay(10);
	uint8_t buff[4];
	buff[0] = 0x9a;
	buff[1] = 0x92;
	buff[2] = 0x9e;
	buff[3] = 0xe6; //�µ�ַ
	
	for (int i = 0;i<4;i++){
		RS485_TX_EN;
		if(HAL_UART_Transmit(&huart4, &buff[i],1,0xFF)!=HAL_OK){
			return false;
		}
		RS485_RX_EN;
		osDelay(10);
	}
	osDelay(100);
	
	
	//printf("\r\n ");
	return true;


}
//static uint16_t USART_ReadData(uint8_t *Buf,uint16_t IdleLen)
//{

////	if (Buf == NULL){
////		
////        return 0;
////	}
////    if ( IdleLen<USBUart.ReadLen)
////    {
////        taskENTER_CRITICAL(); //��ֹUSB�����ж�ͬʱд
////        memcpy(Buf, USBUart.RxBuf, IdleLen);
////        USBUart.ReadLen -= IdleLen;
////        memmove(USBUart.RxBuf, USBUart.RxBuf + IdleLen, USBUart.ReadLen - IdleLen);
////        taskEXIT_CRITICAL();
////        return IdleLen;
////		
////    }
////    else
////    {
////        taskENTER_CRITICAL(); //��ֹUSB�����ж�ͬʱд
////        uint16_t Count = USBUart.ReadLen;
////        memcpy(Buf, USBUart.RxBuf, USBUart.ReadLen);
////        USBUart.ReadLen = 0;
////        taskEXIT_CRITICAL();
////        return Count;
////		
////    }
//}
//float getTemperature(char addr)
//{
//    uint8_t buf[2]; 
//    uint16_t raw,raw1,raw2;
//	float temp=0;
////	IICreadBytes(addr,LM75A_TEMP_OUTH_REG,2,buf); 

//////    buf[0] = iic_read_byte(TEMP_SENSOR_ADDR, 0x00, 1);
//////    buf[1] = iic_read_byte(TEMP_SENSOR_ADDR, 0x00, 0);
////    raw=(((uint16_t)buf[0]<<8)|buf[1]);
////    //raw1=raw | 0x7FFF; 
////    raw2=raw>>5;
////    
//    temp=(int)raw2*0.125;
//    return temp;
//}

//TODO=====
void getDistance(uint16_t dis[DEV_COUNT])
{

}

//float avgDisFilter(float in, uint8_t depth, uint8_t reset)
//{
//	static float sum = 0, mean;
//	if(reset){
//		sum = 0;
//	}
//	else
//	{
//		sum += (in - mean);
//		
//	}
//	mean = sum / depth;
//	
//	return mean;
//}



void obstacleDetection(const uint16_t *dis, const uint8_t devCount)
{
	
	float fliterDis[2]={0.0};
	static int32_t count[2]={0};
	static float tempDis[DEV_COUNT]={0.0};
	static float DisThreshold_max ;
	//taskENTER_CRITICAL();
	
	//taskEXIT_CRITICAL();
	static float lastDis[DEV_COUNT]={0.0};
	static uint8_t direcCount[DEV_COUNT]={0};
	
	
	for(int i=0;i<devCount;i++){
		tempDis[i]=dis[i]/1.0;
		fliterDis[i]=movAvgFilter(&DisMovAvgFilter[i],tempDis[i],3,0);
		//=========����ǰ���ϰ������
		//taskENTER_CRITICAL();
		RobotStateFeed.sObstacleDis.obstacleDis[i]=fliterDis[i];
		//taskEXIT_CRITICAL();
		//ä���ж�,�����½�����,��С��MAX_DETECT_DIS�����ڣ�Ȼ���ִ�����������룬�ж�Ϊä����
		float deltaDis = fliterDis[i]-lastDis[i];
		
		if((fliterDis[i]<=MAX_DETECT_DIS) && deltaDis<0){ //����==0����Զ����ʱ
			direcCount[i]++;
			//printf("direcCount[%d]==%d \r\n",i,direcCount[i]);
			if(direcCount[i]>DESENT_COUNT){
				direcFlag[i]=true;
				
			}
		}else{
			direcCount[i]=0;
		}
		
		if(direcFlag[i]==true){
			if(fliterDis[i]>=MAX_DETECT_DIS){
				fliterDis[i] =0;
				printf("direcFlag[%d] is true,fliterDis[%3.2f]---\r\n",i,fliterDis[i]);
			}
		}
		
		lastDis[i]=fliterDis[i];
		//ʵʱֵ����ֵ�Ƚ�
		DisThreshold_max=cMAX(ParamConfigs.obstacleDis[i],DisThreshold_min_ultra);
		if(( fliterDis[i] <=DisThreshold_max) ){
			count[i]++;
			if(count[i]>=OBSTACAL_COUNT || fliterDis[i]<=0){
				sFlag.limitFlag[i]=true;
				direcFlag[i]=false;
			}
		}
		else{
			count[i]=0;	
			sFlag.limitFlag[i]=false;			
		}		
	}
	
	
	/*****printf******/
	static int cout_x=0;
	cout_x++;
	if(cout_x%10==0){
//		printf(" temDis[0]:	  %3.2f ,	temDis[1]:	%3.2f \r\n ",tempDis[0],tempDis[1]);
//		printf(" ==fliterDis[0]:	%3.2f ,	fliterDis[1]:	%3.2f \r\n ",fliterDis[0],fliterDis[1]);
//    	printf("limitFlag[0]:%d ,limitFlag[1]:%d  filterdis0:%3.2f,   filterdis1:%3.2f, obstacleDis:%3.2f \r\n ",
//		sFlag.limitFlag[0],sFlag.limitFlag[1],fliterDis[0],fliterDis[1],DisThreshold_max);
//		printf("laserStateFlagF:%d,  laserStateFlagB:%d\r\n", sFlag.laserStateFlagF,sFlag.laserStateFlagB);
		cout_x=0;
	}
	/*****printf******/

}


#endif