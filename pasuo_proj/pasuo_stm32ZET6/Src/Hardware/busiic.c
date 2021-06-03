#include "busiic.h"
#include <stdint.h>

typedef unsigned char u8;
typedef unsigned short int  u16;
typedef unsigned int  u32;
//#define VL_IIC_Start(void) IIC_Start(void)
//#define IIC_Send_Byte(uint8_t txd)  IIC_Send_Byte(uint8_t txd)


void ShortToChar(short sData,unsigned char cData[])
{
	cData[0]=sData&0xff;
	cData[1]=sData>>8;
}
short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}

static void Delay(uint32_t count) //us
{
	
    uint8_t i = 0;
    uint8_t delay = 5;
    
    while (delay--)
    {
        i = 10;
        while (i--);
    }
}




/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/


void IIC_Init(void)
{			
	//SDA_OUT();     //sda�����
	IIC_SDA(GPIO_PIN_SET);;	  	  
	IIC_SCL(GPIO_PIN_SET);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA(GPIO_PIN_SET);	  	  
	IIC_SCL(GPIO_PIN_SET);
	
	Delay(10);
 	IIC_SDA(GPIO_PIN_RESET);//START:when CLK is high,DATA change form high to low 
	
	Delay(10);
	IIC_SCL(GPIO_PIN_RESET);//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA(GPIO_PIN_RESET);//STOP:when CLK is high DATA change form low to high
 	
		Delay(5);
	IIC_SCL(GPIO_PIN_SET); 
	IIC_SDA(GPIO_PIN_SET);;//����I2C���߽����ź�
	
		Delay(5);							   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0; 
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA(GPIO_PIN_SET);Delay(1);	
    IIC_SCL(GPIO_PIN_SET);Delay(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
		//Delay(5);
	}  
	IIC_SCL(GPIO_PIN_RESET);//ʱ�����0  
	return 0;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL(GPIO_PIN_RESET);
	SDA_OUT();
	IIC_SDA(GPIO_PIN_RESET);
	Delay(5);
	IIC_SCL(GPIO_PIN_SET);
	Delay(5);
	IIC_SCL(GPIO_PIN_RESET);
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL(GPIO_PIN_RESET);
	SDA_OUT();
	IIC_SDA(GPIO_PIN_SET);
	
		Delay(5);
	IIC_SCL(GPIO_PIN_SET);
		Delay(5);
	IIC_SCL(GPIO_PIN_RESET);
}					 				     

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(uint8_t txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t; 
		SDA_OUT(); 	    
    IIC_SCL(GPIO_PIN_RESET);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>>7==1){
            IIC_SDA(GPIO_PIN_SET);
        }
        else{
            IIC_SDA(GPIO_PIN_RESET);
        }
        
        txd<<=1; 	  
			
		Delay(2);   
		IIC_SCL(GPIO_PIN_SET);
		Delay(5);
		IIC_SCL(GPIO_PIN_RESET);	
		Delay(3);
    }	 
} 	 
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL(GPIO_PIN_RESET); 
        
		Delay(5);
		IIC_SCL(GPIO_PIN_SET);
        receive<<=1;
        if(READ_SDA)receive++;   
		
		Delay(5); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
}



/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
    uint8_t count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1 |0);	   //����д����
	if(IIC_Wait_Ack()){
        IIC_Stop();
        return 1;
    }
	IIC_Send_Byte(reg);   //���͵�ַ
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)|1);  //�������ģʽ	
	IIC_Wait_Ack();
	
//    for(count=0;count<length;count++){
//		 
//		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
//		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
//	}
    while(length)
	{
		if(length==1)*data=IIC_Read_Byte(0);//������,����nACK 
		else *data=IIC_Read_Byte(1);		//������,����ACK  
		length--;
		data++; 
	}  
    IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data){
  
 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
 }
	IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
	
}



////////////////////////////////////////////////////////////////////



u8 VL_IIC_Write_1Byte(u8 SlaveAddress, u8 REG_Address,u8 REG_data)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();//�ͷ�����
		return 1;//ûӦ�����˳�

	}
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();	
	IIC_Send_Byte(REG_data);
	IIC_Wait_Ack();	
	IIC_Stop();

	return 0;
}

//IIC��һ���ֽ�����
u8 VL_IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);//��д����
	if(IIC_Wait_Ack())
	{
		 IIC_Stop();//�ͷ�����
		 return 1;//ûӦ�����˳�
	}		
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();
	IIC_Start(); 
	IIC_Send_Byte(SlaveAddress|0x01);//��������
	IIC_Wait_Ack();
	*REG_data = IIC_Read_Byte(0);
	IIC_Stop();

	return 0;
}

//IICдn�ֽ�����
u8 VL_IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address,u16 len, u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);//��д����
	if(IIC_Wait_Ack()) 
	{
		IIC_Stop();//�ͷ�����
		return 1;//ûӦ�����˳�
	}
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();
	while(len--)
	{
		IIC_Send_Byte(*buf++);//����buff������
		IIC_Wait_Ack();	
	}
	IIC_Stop();//�ͷ�����

	return 0;
	
}

//IIC��n�ֽ�����
u8 VL_IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address,u16 len,u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);//��д����
	if(IIC_Wait_Ack()) 
	{
		IIC_Stop();//�ͷ�����
		return 1;//ûӦ�����˳�
	}
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Send_Byte(SlaveAddress|0x01);//��������
	IIC_Wait_Ack();
	while(len)
	{
		if(len==1)
		{
			*buf = IIC_Read_Byte(0);
		}
		else
		{
			*buf = IIC_Read_Byte(1);
		}
		buf++;
		len--;
	}
	IIC_Stop();//�ͷ�����

	return 0;
	
}

//VL53L0X д�������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_write_multi(u8 address, u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK;

	if(VL_IIC_Write_nByte(address,index,count,pdata))
	{
	   status  = STATUS_FAIL;

	}

	return status;
}


//VL53L0X ���������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_read_multi(u8 address,u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK;

	if(VL_IIC_Read_nByte(address,index,count,pdata))
	{
	  status  = STATUS_FAIL;
	}

	return status;
}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_write_byte(u8 address,u8 index,u8 data)
{
	u8 status = STATUS_OK;

	status = VL53L0X_write_multi(address,index,&data,1);

	return status;
}

//VL53L0X д1������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_write_word(u8 address,u8 index,u16 data)
{
	u8 status = STATUS_OK;
	
	u8 buffer[2];
	
	//��16λ���ݲ�ֳ�8λ
	buffer[0] = (u8)(data>>8);//�߰�λ
	buffer[1] = (u8)(data&0xff);//�Ͱ�λ
	
	if(index%2==1)
	{  
		//����ͨ�Ų��ܴ���Է�2�ֽڶ���Ĵ������ֽ�
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
	}else
	{
		status = VL53L0X_write_multi(address,index,buffer,2);
	}
	
	return status;
}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_write_dword(u8 address,u8 index,u32 data)
{
	
    u8 status = STATUS_OK;

    u8 buffer[4];	
	
	//��32λ���ݲ�ֳ�8λ
	buffer[0] = (u8)(data>>24);
	buffer[1] = (u8)((data&0xff0000)>>16);
	buffer[2] = (u8)((data&0xff00)>>8);
	buffer[3] = (u8)(data&0xff);
	
	status = VL53L0X_write_multi(address,index,buffer,4);
	
	return status;
	
}


//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_read_byte(u8 address,u8 index,u8 *pdata)
{
	u8 status = STATUS_OK;
	 
	status = VL53L0X_read_multi(address,index,pdata,1);
	
	return status;
	 
}

//VL53L0X ��������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_read_word(u8 address,u8 index,u16 *pdata)
{
	u8 status = STATUS_OK;
	
	u8 buffer[2];
	
	status = VL53L0X_read_multi(address,index,buffer,2);
	
	*pdata = ((u16)buffer[0]<<8)+(u16)buffer[1];
	
	return status;
	
}

//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_read_dword(u8 address,u8 index,u32 *pdata)
{
	u8 status = STATUS_OK;
	
	u8 buffer[4];
	
	status = VL53L0X_read_multi(address,index,buffer,4);
	
	*pdata = ((u32)buffer[0]<<24)+((u32)buffer[1]<<16)+((u32)buffer[2]<<8)+((u32)buffer[3]);
	
	return status;
	
}












