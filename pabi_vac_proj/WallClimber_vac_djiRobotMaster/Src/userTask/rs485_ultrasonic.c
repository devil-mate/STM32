#include "rs485_ultrasonic.h"
#define ULTRASNTIC 1
#define SET_ADDR  1

#if(ULTRASNTIC == 1)
#include <string.h>
#include "cmsis_os.h"
#include "common.h"
#include "protocol_type.h"

//#define calibration 0
//盲区（初始值），是10mm
#define DisThreshold_min_ultra 20

#define MAX_DETECT_DIS 5.00*1000 //mm
#define DESENT_COUNT 10
#define OBSTACAL_COUNT 5

extern UART_HandleTypeDef huart8;
static TaskHandle_t rs485DataTaskHandle = NULL;
static void rs485DataTask(void *argument);

void obstacleDetection(const uint16_t *dis, const uint8_t devCount);
float avgDisFilter(float in, uint8_t depth, uint8_t reset);
float movAvgDisFilter(float in, uint8_t depth, uint8_t reset);
//VL53L0X_DeviceInfo_t vl53l0x_dev_info;//设备ID版本信息


//extern RobotStateFeed_s    RobotStateFeed;
MovAvgFilter_s DisMovAvgFilter[DEV_COUNT];//0前，1后

static uint16_t RS485_ReadDataCmd(ultra_dev_s *dev,uint16_t len);
static bool RS485_setAddrCmd(uint8_t addr,uint8_t targetAddr);

//uint8_t UART4_Rx_Buf[MAX_REC_LENGTH] = {0}; //USART4存储接收数据
extern uint8_t  ultrasonic_uart_temp[REC_LENGTH];


QueueHandle_t      rs485RxMsgQueueHandle = NULL;

bool direcFlag[DEV_COUNT] ={false}; 
#define direcFlagF direcFlag[0] 
#define direcFlagB direcFlag[1] 
void initRS485Data()
{

    xTaskCreate(rs485DataTask,         /* 任务函数  */
                "rs485DataTask",       /* 任务名    */
                256,                         /* 任务栈大小，单位word，也就是4字节 */
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
		//printf("create rs485RxMsgQueueHandle fail!");
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
	
	ultra_dev_s ultraDev[DEV_COUNT];
	static int8_t ultraDis_Count[DEV_COUNT]={0};

	
	ultraDev[0].buff[0]=LEFT_FORWARD_ADDR;
	ultraDev[1].buff[0]=RIGHT_FORWARD_ADDR;
	ultraDev[2].buff[0]=RIGHT_BACKWARD_ADDR;
	ultraDev[3].buff[0]=LEFT_BACKWARD_ADDR;
	
	for(uint8_t i=0;i<DEV_COUNT;i++){
		ultraDev[i].buff[1] = 0x02;
		ultraDev[i].buff[2] = 0x0BC;
	}
	

	
	uint16_t ultra_distance[DEV_COUNT]={0};
	//HAL_UART_Receive_IT(&huart8,(uint8_t*)&bRxBuffer,1);//(USART1->DR);	//读取接收到的数据
	HAL_UART_Receive_IT(&huart8,(uint8_t *)ultrasonic_uart_temp,REC_LENGTH);
	
	//RS485_RX_EN;

	movAvgFilterInit(&DisMovAvgFilter[0]);
	movAvgFilterInit(&DisMovAvgFilter[1]);
/***修改地址*****/
#ifdef SET_ADDR
//RS485_setAddrCmd(0xea,0xe6);
RS485_setAddrCmd(0xe8,LEFT_FORWARD_ADDR);
//	while(!RS485_setAddrCmd()){
//		osDelay(100);
//		RS485_setAddrCmd();

//	};
//	printf("set addr success!\r\n");
//	return ;
#endif
/***修改地址*****/
     while (1)
    {

		if(rs485RxMsgQueueHandle ==NULL){
			//printf("rs485RxMsgQueueHandle is Null\r\n");
			continue;
		}
		
		for(uint8_t i=0;i<DEV_COUNT;i++){
			RS485_ReadDataCmd(&ultraDev[i],sizeof(ultraDev[i].buff));//超声波数据读取命令
			//0--11m，温度补偿最大耗时约87ms，中断中获取距离,读取
			if(xQueueReceive(rs485RxMsgQueueHandle,&tempDistanceF,150*portTICK_RATE_MS)!=pdPASS){
				ultraDis_Count[i]++;
				if(ultraDis_Count[i]>30){
					//掉线标记
					sFlag.ultraStateFlag[i]=true;
					//sFlag.ultraStateFlag[left_forward_Index] =true;
					ultraDis_Count[i]=0;	
				}
			}else{
				sFlag.ultraStateFlag[i]=false;
				ultra_distance[i]=tempDistanceF; //得到数据		
			}
		
		
		}
		//obstacleDetection(ultra_distance,DEV_COUNT);

	}



}
static uint16_t RS485_ReadDataCmd(ultra_dev_s *dev,uint16_t len)
{
	if(HAL_UART_Transmit(&huart8, dev->buff,len,0xFF)!= HAL_OK){
			return 0;
	}

	return len;


}
static bool RS485_setAddrCmd(uint8_t addr,uint8_t targetAddr)
{
	
	ultra_dev_s setAddrData[4];
	setAddrData[0].buff[0] = addr;
	setAddrData[0].buff[1] = 0x02;
	setAddrData[0].buff[2] = 0x9a;
	
	setAddrData[1].buff[0] = addr;
	setAddrData[1].buff[1] = 0x02;
	setAddrData[1].buff[2] = 0x92;
	setAddrData[2].buff[0] = addr;
	setAddrData[2].buff[1] = 0x02;
	setAddrData[2].buff[2] = 0x9e;
	setAddrData[3].buff[0] = addr;
	setAddrData[3].buff[1] = 0x02;
	setAddrData[3].buff[2] = targetAddr;

	
	for (int i = 0;i<4;i++){
		RS485_ReadDataCmd(&setAddrData[i],sizeof(setAddrData[i].buff));
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
				//printf("direcFlag[%d] is true,fliterDis[%3.2f]---\r\n",i,fliterDis[i]);
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