#include "rs485_ultrasonic.h"
#define ULTRASNTIC 1

#if(ULTRASNTIC == 1)
#include <string.h>
#include "cmsis_os.h"
#include "common.h"
#include "protocol_type.h"

//#define calibration 0
//盲区（初始值），是10mm
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
//VL53L0X_DeviceInfo_t vl53l0x_dev_info;//设备ID版本信息

static bool RS485_setAddrCmd();

MovAvgFilter_s DisMovAvgFilter[DEV_COUNT];//0前，1后

static uint16_t RS485_ReadDataCmd(ultra_dev_s *dev,uint16_t len);


//uint8_t UART4_Rx_Buf[MAX_REC_LENGTH] = {0}; //USART4存储接收数据

uint8_t  UART4_temp[REC_LENGTH] = {0};       //USART4接收数据缓存

QueueHandle_t      rs485RxMsgQueueHandle = NULL;

bool direcFlag[DEV_COUNT] ={false,false}; 
#define direcFlagF direcFlag[0] 
#define direcFlagB direcFlag[1] 
void initRS485Data()
{

    xTaskCreate(rs485DataTask,         /* 任务函数  */
                "rs485DataTask",       /* 任务名    */
                512,                         /* 任务栈大小，单位word，也就是4字节 */
                NULL,                         /* 任务参数  */
                10,                            /* 任务优先级*/
                &rs485DataTaskHandle); /* 任务句柄  */
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
//	initDataStorage(11);//初始化11扇区（最后一个）
//	osDelay(2000);
//	//初始化
//	status=initDisSensor();


//	uint8_t sendBuf[3];
//	//sendBuf[0]=0x32; //请求发送
//	sendBuf[0]=0xE8;//地址
//	sendBuf[1]=0x02;//寄存器
//	sendBuf[2]=0xBC;//命令
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
	//HAL_UART_Receive_IT(&huart4,(uint8_t*)&bRxBuffer,1);//(USART1->DR);	//读取接收到的数据
	HAL_UART_Receive_IT(&huart4,(uint8_t *)UART4_temp,REC_LENGTH);
	
	RS485_RX_EN;

	movAvgFilterInit(&DisMovAvgFilter[0]);
	movAvgFilterInit(&DisMovAvgFilter[1]);
/***修改地址*****/
//	while(!RS485_setAddrCmd()){
//		osDelay(100);
//		RS485_setAddrCmd();

//	};
//	printf("set addr success!\r\n");
//	return ;
/***修改地址*****/
     while (1)
    {
			
        //osDelay(150);
		//TODO=======提取成函数
		//getDistance(ultra_distance);
//		if(RS485_send_flag==1){
		//前传感器
		//printf("%x\r\n",55);
		ultra_distance[0] = 0;
		ultra_distance[1] = 0;
		//tempDistance=0;
		if(rs485RxMsgQueueHandle ==NULL){
			printf("rs485RxMsgQueueHandle is Null\r\n");
			continue;
		}
		RS485_ReadDataCmd(&Forward_dev,sizeof(Forward_dev.buff));//超声波数据读取命令
		//0--11m，温度补偿最大耗时约87ms，中断中获取距离,读取
		if(xQueueReceive(rs485RxMsgQueueHandle,&tempDistanceF,150*portTICK_RATE_MS)!=pdPASS){
			ultraDis_CountF++;
			if(ultraDis_CountF>30){
				//掉线标记
				sFlag.laserStateFlagF=true;
				ultraDis_CountF=0;	
			}
		}else{
			sFlag.laserStateFlagF=false;
			ultra_distance[0]=tempDistanceF; //得到数据		
		}
		
		
		//后传感器
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
			ultra_distance[1]=tempDistanceB; //得到数据
		}
		obstacleDetection(ultra_distance,DEV_COUNT);

	}



}
static uint16_t RS485_ReadDataCmd(ultra_dev_s *dev,uint16_t len)
{
	//设置为发送模式或者得到芯片的请求发送，请求发送用0x32
	
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
	//设置为发送模式或者得到芯片的请求发送，请求发送用0x32
	osDelay(10);
	uint8_t buff[4];
	buff[0] = 0x9a;
	buff[1] = 0x92;
	buff[2] = 0x9e;
	buff[3] = 0xe6; //新地址
	
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
////        taskENTER_CRITICAL(); //防止USB接收中断同时写
////        memcpy(Buf, USBUart.RxBuf, IdleLen);
////        USBUart.ReadLen -= IdleLen;
////        memmove(USBUart.RxBuf, USBUart.RxBuf + IdleLen, USBUart.ReadLen - IdleLen);
////        taskEXIT_CRITICAL();
////        return IdleLen;
////		
////    }
////    else
////    {
////        taskENTER_CRITICAL(); //防止USB接收中断同时写
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
		//=========反馈前后障碍物距离
		//taskENTER_CRITICAL();
		RobotStateFeed.sObstacleDis.obstacleDis[i]=fliterDis[i];
		//taskEXIT_CRITICAL();
		//盲区判断,连续下降次数,在小于MAX_DETECT_DIS距离内，然后又大于了这个距离，判断为盲区。
		float deltaDis = fliterDis[i]-lastDis[i];
		
		if((fliterDis[i]<=MAX_DETECT_DIS) && deltaDis<0){ //不能==0，最远距离时
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
		//实时值和阈值比较
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