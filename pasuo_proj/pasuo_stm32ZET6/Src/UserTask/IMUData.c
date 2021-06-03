#include "IMUData.h"
#include "cmsis_os.h"
#include "can_handler.h"


static TaskHandle_t IMUDataTaskHandle = NULL;
static void IMUDataTask(void *argument);



void initIMUData()
{
    
    xTaskCreate(IMUDataTask,         /* ������  */
                "IMUDataTask",       /* ������    */
                1024,                         /* ����ջ��С����λword��Ҳ����4�ֽ� */
                NULL,                         /* �������  */
                1,                            /* �������ȼ�*/
                &IMUDataTaskHandle); /* ������  */
}


static void IMUDataTask(void *argument)
{
	unsigned char chrTemp[30];
	unsigned char str[100];
	float  temp;
	int i;
	
	float a[3],w[3],h[3],Angle[3],temperature;


//	vl53l0x_init(&vl53l0x_dev);
    int t;
        while (1)
    {
        osDelay(10);
				//taskENTER_CRITICAL();
				IICreadBytesII(0x50, 0x34, 26,chrTemp);
				//taskEXIT_CRITICAL();
				a[0] = (float)CharToShort(&chrTemp[0])/32768.0*16;
				a[1] = (float)CharToShort(&chrTemp[2])/32768.0*16;
				a[2] = (float)CharToShort(&chrTemp[4])/32768.0*16;
				w[0] = (float)CharToShort(&chrTemp[6])/32768.0*2000;
				w[1] = (float)CharToShort(&chrTemp[8])/32768.0*2000;
				w[2] = (float)CharToShort(&chrTemp[10])/32768.0*2000;
				h[0] = CharToShort(&chrTemp[12]);
				h[1] = CharToShort(&chrTemp[14]);
				h[2] = CharToShort(&chrTemp[16]);
				Angle[0] = (float)CharToShort(&chrTemp[18])/32768.0*180;
				Angle[1] = (float)CharToShort(&chrTemp[20])/32768.0*180;
				Angle[2] = (float)CharToShort(&chrTemp[22])/32768.0*180;
        temperature = (float)CharToShort(&chrTemp[24])/100.0;
        t++;
        if(t%100==0){
					for(i=0;i<3;i++){
            //printf("tempbuf[%d]:%x ",i+18,chrTemp[i+18]);
            printf("Angle[%d]:%3.2f ",i,Angle[i]);
            //vTaskDelay(10);
            t=0;
					}
					printf("temperature:%2.2f ",temperature);
					printf("\r\n\r\n");
        }

    


    
    
    
    }



}

