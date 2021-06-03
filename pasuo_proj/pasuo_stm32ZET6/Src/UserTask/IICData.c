#include "IICData.h"
#define IIC_LASER 0
#if(IIC_LASER==1)
#include "cmsis_os.h"
#include "can_handler.h"
#include "vl53l0x.h"

#include "common.h"
#include "protocol_type.h"
#include "data_storage.h"

#define calibration 0
//ä������ʼֵ�����Ƿ����ͨ��ƫ��ֵ����Ϊ0��ä��Ҳ��3-4cm
#define DisThreshold_min 100
//int8_t fowardLimit=0,backwardLimit=0;
//int8_t limitFlag[2]={0};
static TaskHandle_t IICDataTaskHandle = NULL;
static void IICDataTask(void *argument);
void obstacleDetection();
float avgDisFilter(float in, uint8_t depth, uint8_t reset);
float movAvgDisFilter(float in, uint8_t depth, uint8_t reset);
//VL53L0X_DeviceInfo_t vl53l0x_dev_info;//�豸ID�汾��Ϣ


u16 Distance_data=0;



VL53L0X_Dev_t vl53l0x_dev;//ǰ���豸I2C���ݲ���
VL53L0X_Dev_t vl53l0x_dev2;//��
VL53L0X_RangingMeasurementData_t vl53l0x_data;//�������ṹ��
VL53L0X_RangingMeasurementData_t vl53l0x_data2;//�������ṹ��
MovAvgFilter_s DisMovAvgFilter[2];//0ǰ��1��
void feedback_laserFlag();


void initIICData()
{

    xTaskCreate(IICDataTask,         /* ������  */
                "IMUDataTask",       /* ������    */
                512,                         /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                         /* �������  */
                2,                            /* �������ȼ�*/
                &IICDataTaskHandle); /* ������  */
}


static void IICDataTask(void *argument)
{
    unsigned char chrTemp[30];
	unsigned char str[100];
    float  temp;
	float a[3],w[3],h[3],Angle[3],temperature;
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	memset(&sAdjust_Data,0,sizeof(sAdjust_Data));
	memset(&sAdjust_writeData,0,sizeof(sAdjust_writeData));
	
	memset(&vl53l0x_data, 0, sizeof(vl53l0x_data));
	memset(&vl53l0x_data2, 0, sizeof(vl53l0x_data2));
	//TODO====
	initDataStorage(11);//��ʼ��11���������һ����
	osDelay(2000);
	//��ʼ��
	status=initDisSensor();

//	while(status!=VL53L0X_ERROR_NONE){
//		status=initDisSensor();
//		osDelay(2000);
//	}

	movAvgFilterInit(&DisMovAvgFilter[0]);
	movAvgFilterInit(&DisMovAvgFilter[1]);
	
//	vl53l0x_reset(&vl53l0x_dev);
	
//	VL53L0X_StartMeasurement(&vl53l0x_dev);
	//vl53l0x_init(&vl53l0x_dev,XSHUT2,vl53l0x_addr2,0);



	int t;
     while (1)
    {
		t++;
        osDelay(20);				
	//    osDelay(10);
	//    temp=getTemperature(LM75A_ADDR0);
	//    osDelay(10);
	//    temp=getTemperature(LM75A_ADDR1);
	//    printf("temp:%f\r\n",temp);
		
		//��ȡ�������
		getDistanceTOF();
		obstacleDetection();
		//=======״̬����
		if(t%20==0){
			feedback_laserFlag();
			t=0;
		}

	}



}
float getTemperature(char addr)
{
    uint8_t buf[2]; 
    uint16_t raw,raw1,raw2;
	float temp=0;
	IICreadBytes(addr,LM75A_TEMP_OUTH_REG,2,buf); 

//    buf[0] = iic_read_byte(TEMP_SENSOR_ADDR, 0x00, 1);
//    buf[1] = iic_read_byte(TEMP_SENSOR_ADDR, 0x00, 0);
    raw=(((uint16_t)buf[0]<<8)|buf[1]);
    //raw1=raw | 0x7FFF; 
    raw2=raw>>5;
    
    temp=(int)raw2*0.125;
    return temp;
}

float getDistanceTOF()
{	
	VL53L0X_Error Status=VL53L0X_ERROR_NONE;//����״̬
	//XSHUT���ܼӵ�dev�ṹ����
	Status=vl53l0x_general_start(&vl53l0x_dev,XSHUT1,&vl53l0x_data);
	static int8_t reconnectCount[2]={0};

	osDelay(30);
	if(Status==VL53L0X_ERROR_NONE){
		sFlag.laserStateFlagF=0;
		reconnectCount[0]=0;
	}
	else{
		reconnectCount[0]++;
		if(reconnectCount[0]>=5){
			sFlag.laserStateFlagF=1;
		}
		vl53l0x_data.RangeMilliMeter=0;//����Ϊ0
		vl53l0x_init(&vl53l0x_dev,XSHUT1,0);
		vl53l0x_set_mode(&vl53l0x_dev,0,&sAdjust_Data.data1);
		osDelay(100);
	}
		
	Status=vl53l0x_general_start(&vl53l0x_dev2,XSHUT3,&vl53l0x_data2);
	if(Status==VL53L0X_ERROR_NONE){
		sFlag.laserStateFlagB=0;
		reconnectCount[0]=0;
	}
	else{
		reconnectCount[1]++;
		if(reconnectCount[1]>=5){
			sFlag.laserStateFlagB=1;
		}
		vl53l0x_data2.RangeMilliMeter=0;//����Ϊ0
		vl53l0x_init(&vl53l0x_dev2,XSHUT3,0);
		vl53l0x_set_mode(&vl53l0x_dev2,0,&sAdjust_Data.data2);
		osDelay(100);		
	}
		//����������ʽ
	//VL53L0X_RangingMeasurementData(&vl53l0x_dev,VL53L0X_RangingMeasurementData);
}
//int8_t setMode(uint8_t addr,uint8_t mode)
//{	
//	VL53L0X_Dev_t *dev;
//	dev->I2cDevAddr = addr;
//	//vl53l0x_set_mode(dev,mode);//���ò���ģʽ

//}
float avgDisFilter(float in, uint8_t depth, uint8_t reset)
{
	static float sum = 0, mean;
	if(reset){
		sum = 0;
	}
	else
	{
		sum += (in - mean);
		
	}
	mean = sum / depth;
	
	return mean;
}



void obstacleDetection()
{
	
	float fliterDis[2]={0.0};
	static int32_t count[2]={0};
	float tempDis[4]={0.0};
	
	tempDis[0]=vl53l0x_data.RangeMilliMeter/1.0;
	tempDis[1]=vl53l0x_data2.RangeMilliMeter/1.0;
	
	//float DisThreshold_max=cMAX(ParamConfigs.obstacleDis,DisThreshold_min);
	for(int i=0;i<2;i++){
		fliterDis[i]=movAvgFilter(&DisMovAvgFilter[i],tempDis[i],5,0);
		//=========����ǰ���ϰ������
		RobotStateFeed.sObstacleDis.obstacleDis[i]=fliterDis[i];
		//=========TODOǰ�󼤹⻵���Ƿ����
		//if(!laserStateFlagF[i])
		
		if((fliterDis[i]>DisThreshold_min && fliterDis[i] <DisThreshold_max) ){
			count[i]++;
			if(count[i]>=5){
				sFlag.limitFlag[i]=1;	
			}
		}
		else{
			count[i]=0;	
			sFlag.limitFlag[i]=0;			
		}		
	}
	
	
	/*****printf******/
	static int cout_x=0;
	cout_x++;
	if(cout_x%10==0){
////		printf("temDis[0]:%3.2f ,   temDis[1]:%3.2f \r\n",tempDis[0],tempDis[1]);
//    	printf("limitFlag[0]:%d ,limitFlag[1]:%d  filterdis0:%3.2f,   filterdis1:%3.2f, obstacleDis:%3.2f \r\n ",
//		sFlag.limitFlag[0],sFlag.limitFlag[1],fliterDis[0],fliterDis[1],DisThreshold_max);
//		printf("laserStateFlagF:%d,  laserStateFlagB:%d\r\n", sFlag.laserStateFlagF,sFlag.laserStateFlagB);
//		cout_x=0;
	}
	/*****printf******/

}
//init mode=0;
VL53L0X_Error initDisSensor()
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	sFlag.laserStateFlagB=0;
	sFlag.laserStateFlagF=0;
	
	vl53l0x_dev.I2cDevAddr = vl53l0x_addr1;//I2C��ַ()
	vl53l0x_dev.comms_type = 1;           //I2Cͨ��ģʽ
	vl53l0x_dev.comms_speed_khz = 400;    //I2Cͨ������
	status=vl53l0x_init(&vl53l0x_dev,XSHUT1,0);
	while(status != VL53L0X_ERROR_NONE ){
		sFlag.laserStateFlagF=1;
		status=vl53l0x_init(&vl53l0x_dev,XSHUT1,0);
		osDelay(2000);
		return  status;
	}
	vl53l0x_set_mode(&vl53l0x_dev,0,&sAdjust_Data.data1);
	osDelay(1000);
	
	vl53l0x_dev2.I2cDevAddr = vl53l0x_addr2;//I2C��ַ()
	vl53l0x_dev2.comms_type = 1;           //I2Cͨ��ģʽ
	vl53l0x_dev2.comms_speed_khz = 400;    //I2Cͨ������
	status=vl53l0x_init(&vl53l0x_dev2,XSHUT3,0);
	while(status != VL53L0X_ERROR_NONE ){
		sFlag.laserStateFlagB=1;
		status=vl53l0x_init(&vl53l0x_dev2,XSHUT3,0);
		osDelay(2000);
		return  status;
	}
	vl53l0x_set_mode(&vl53l0x_dev2,0,&sAdjust_Data.data2);
	osDelay(1000);
	
	return status;
	
}
void feedback_laserFlag()
{
	if(sFlag.laserStateFlagF){
		setbit(RobotStateFeed.digit,0);//��0λ��ǰ������߱�־
	}else{
		resetbit(RobotStateFeed.digit,0);
	}
	
	if(sFlag.laserStateFlagB){
		setbit(RobotStateFeed.digit,1);//��1λ���󼤹���߱�־
	}else{
		resetbit(RobotStateFeed.digit,1);
	}
	
	if(sFlag.limitFlag[0]){
		setbit(RobotStateFeed.digit,5);//��5λ��ǰ���⼫�ޱ�־
	}else{
		resetbit(RobotStateFeed.digit,5);
	}
	
	if(sFlag.laserStateFlagB){
		setbit(RobotStateFeed.digit,6);//��6λ���󼤹⼫�ޱ�־
	}else{
		resetbit(RobotStateFeed.digit,6);
	}

}
#endif