#include "busiic2.h"
#include <stdint.h>

typedef unsigned char u8;
typedef unsigned short int  u16;
typedef unsigned int  u32;
//#define VL_IIC_Start(void) IIC_StartII(void)
//#define IIC_Send_ByteII(uint8_t txd)  IIC_Send_ByteII(uint8_t txd)


//void ShortToChar(short sData,unsigned char cData[])
//{
//	cData[0]=sData&0xff;
//	cData[1]=sData>>8;
//}
//short CharToShort(unsigned char cData[])
//{
//	return ((short)cData[1]<<8)|cData[0];
//}

static void DelayII(uint32_t count) //us
{
	
    uint8_t i = 0;
    uint8_t DelayII = 5;
    
    while (DelayII--)
    {
        i = 10;
        while (i--);
    }
}




/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/


void IIC_InitII(void)
{			
	//SDA_OUTII();     //sda�����
	IIC_SDAII(GPIO_PIN_SET);;	  	  
	IIC_SCLII(GPIO_PIN_SET);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_StartII(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
void IIC_StartII(void)
{
	SDA_OUTII();     //sda�����
	IIC_SDAII(GPIO_PIN_SET);	  	  
	IIC_SCLII(GPIO_PIN_SET);
	
	DelayII(10);
 	IIC_SDAII(GPIO_PIN_RESET);//START:when CLK is high,DATA change form high to low 
	
	DelayII(10);
	IIC_SCLII(GPIO_PIN_RESET);//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_StopII(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_StopII(void)
{
	SDA_OUTII();//sda�����
	IIC_SCLII(GPIO_PIN_RESET);
	IIC_SDAII(GPIO_PIN_RESET);//STOP:when CLK is high DATA change form low to high
 	
		DelayII(5);
	IIC_SCLII(GPIO_PIN_SET); 
	IIC_SDAII(GPIO_PIN_SET);;//����I2C���߽����ź�
	
		DelayII(5);							   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Wait_AckII(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
uint8_t IIC_Wait_AckII(void)
{
	uint8_t ucErrTime=0; 
	SDA_INII();      //SDA����Ϊ����  
	IIC_SDAII(GPIO_PIN_SET);DelayII(1);	
    IIC_SCLII(GPIO_PIN_SET);DelayII(1);
	while(READ_SDAII)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_StopII();
			return 1;
		}
		//DelayII(5);
	}  
	IIC_SCLII(GPIO_PIN_RESET);//ʱ�����0  
	return 0;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_AckII(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_AckII(void)
{
	IIC_SCLII(GPIO_PIN_RESET);
	SDA_OUTII();
	IIC_SDAII(GPIO_PIN_RESET);
	DelayII(5);
	IIC_SCLII(GPIO_PIN_SET);
	DelayII(5);
	IIC_SCLII(GPIO_PIN_RESET);
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAckII(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAckII(void)
{
	IIC_SCLII(GPIO_PIN_RESET);
	SDA_OUTII();
	IIC_SDAII(GPIO_PIN_SET);
	
		DelayII(5);
	IIC_SCLII(GPIO_PIN_SET);
		DelayII(5);
	IIC_SCLII(GPIO_PIN_RESET);
}					 				     

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_ByteII(uint8_t txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_ByteII(uint8_t txd)
{                        
    uint8_t t; 
		SDA_OUTII(); 	    
    IIC_SCLII(GPIO_PIN_RESET);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>>7==1){
            IIC_SDAII(GPIO_PIN_SET);
        }
        else{
            IIC_SDAII(GPIO_PIN_RESET);
        }
        
        txd<<=1; 	  
			
		DelayII(2);   
		IIC_SCLII(GPIO_PIN_SET);
		DelayII(5);
		IIC_SCLII(GPIO_PIN_RESET);	
		DelayII(3);
    }	 
} 	 
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IIC_Read_ByteII(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
uint8_t IIC_Read_ByteII(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_INII();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCLII(GPIO_PIN_RESET); 
        
		DelayII(5);
		IIC_SCLII(GPIO_PIN_SET);
        receive<<=1;
        if(READ_SDAII)receive++;   
		
		DelayII(5); 
    }					 
    if (ack)
        IIC_AckII(); //����ACK 
    else
        IIC_NAckII();//����nACK  
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
uint8_t IICreadBytesII(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
    uint8_t count = 0;
	
	IIC_StartII();
	IIC_Send_ByteII(dev<<1 |0);	   //����д����
	if(IIC_Wait_AckII()){
        IIC_StopII();
        return 1;
    }
	IIC_Send_ByteII(reg);   //���͵�ַ
  IIC_Wait_AckII();	  
	IIC_StartII();
	IIC_Send_ByteII((dev<<1)|1);  //�������ģʽ	
	IIC_Wait_AckII();
	
//    for(count=0;count<length;count++){
//		 
//		 if(count!=length-1)data[count]=IIC_Read_ByteII(1);  //��ACK�Ķ�����
//		 	else  data[count]=IIC_Read_ByteII(0);	 //���һ���ֽ�NACK
//	}
    while(length)
	{
		if(length==1)*data=IIC_Read_ByteII(0);//������,����nACK 
		else *data=IIC_Read_ByteII(1);		//������,����ACK  
		length--;
		data++; 
	}  
    IIC_StopII();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint8_t IICwriteBytesII(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
uint8_t IICwriteBytesII(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data){
  
 	uint8_t count = 0;
	IIC_StartII();
	IIC_Send_ByteII(dev<<1);	   //����д����
	IIC_Wait_AckII();
	IIC_Send_ByteII(reg);   //���͵�ַ
	IIC_Wait_AckII();	  
	for(count=0;count<length;count++){
		IIC_Send_ByteII(data[count]); 
		IIC_Wait_AckII(); 
 }
	IIC_StopII();//����һ��ֹͣ����

    return 1; //status == 0;
	
}



////////////////////////////////////////////////////////////////////



u8 VL_IIC_Write_1ByteII(u8 SlaveAddress, u8 REG_Address,u8 REG_data)
{
	IIC_StartII();
	IIC_Send_ByteII(SlaveAddress);
	if(IIC_Wait_AckII())
	{
		IIC_StopII();//�ͷ�����
		return 1;//ûӦ�����˳�

	}
	IIC_Send_ByteII(REG_Address);
	IIC_Wait_AckII();	
	IIC_Send_ByteII(REG_data);
	IIC_Wait_AckII();	
	IIC_StopII();

	return 0;
}

//IIC��һ���ֽ�����
u8 VL_IIC_Read_1ByteII(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
{
	IIC_StartII();
	IIC_Send_ByteII(SlaveAddress);//��д����
	if(IIC_Wait_AckII())
	{
		 IIC_StopII();//�ͷ�����
		 return 1;//ûӦ�����˳�
	}		
	IIC_Send_ByteII(REG_Address);
	IIC_Wait_AckII();
	IIC_StartII(); 
	IIC_Send_ByteII(SlaveAddress|0x01);//��������
	IIC_Wait_AckII();
	*REG_data = IIC_Read_ByteII(0);
	IIC_StopII();

	return 0;
}

//IICдn�ֽ�����
u8 VL_IIC_Write_nByteII(u8 SlaveAddress, u8 REG_Address,u16 len, u8 *buf)
{
	IIC_StartII();
	IIC_Send_ByteII(SlaveAddress);//��д����
	if(IIC_Wait_AckII()) 
	{
		IIC_StopII();//�ͷ�����
		return 1;//ûӦ�����˳�
	}
	IIC_Send_ByteII(REG_Address);
	IIC_Wait_AckII();
	while(len--)
	{
		IIC_Send_ByteII(*buf++);//����buff������
		IIC_Wait_AckII();	
	}
	IIC_StopII();//�ͷ�����

	return 0;
	
}

//IIC��n�ֽ�����
u8 VL_IIC_Read_nByteII(u8 SlaveAddress, u8 REG_Address,u16 len,u8 *buf)
{
	IIC_StartII();
	IIC_Send_ByteII(SlaveAddress);//��д����
	if(IIC_Wait_AckII()) 
	{
		IIC_StopII();//�ͷ�����
		return 1;//ûӦ�����˳�
	}
	IIC_Send_ByteII(REG_Address);
	IIC_Wait_AckII();

	IIC_StartII();
	IIC_Send_ByteII(SlaveAddress|0x01);//��������
	IIC_Wait_AckII();
	while(len)
	{
		if(len==1)
		{
			*buf = IIC_Read_ByteII(0);
		}
		else
		{
			*buf = IIC_Read_ByteII(1);
		}
		buf++;
		len--;
	}
	IIC_StopII();//�ͷ�����

	return 0;
	
}

//VL53L0X д�������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_write_multiII(u8 address, u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OKII;

	if(VL_IIC_Write_nByteII(address,index,count,pdata))
	{
	   status  = STATUS_FAILII;

	}

	return status;
}


//VL53L0X ���������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_read_multiII(u8 address,u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OKII;

	if(VL_IIC_Read_nByteII(address,index,count,pdata))
	{
	  status  = STATUS_FAILII;
	}

	return status;
}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_write_byteII(u8 address,u8 index,u8 data)
{
	u8 status = STATUS_OKII;

	status = VL53L0X_write_multiII(address,index,&data,1);

	return status;
}

//VL53L0X д1������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_write_wordII(u8 address,u8 index,u16 data)
{
	u8 status = STATUS_OKII;
	
	u8 buffer[2];
	
	//��16λ���ݲ�ֳ�8λ
	buffer[0] = (u8)(data>>8);//�߰�λ
	buffer[1] = (u8)(data&0xff);//�Ͱ�λ
	
	if(index%2==1)
	{  
		//����ͨ�Ų��ܴ���Է�2�ֽڶ���Ĵ������ֽ�
		status = VL53L0X_write_multiII(address,index,&buffer[0],1);
		status = VL53L0X_write_multiII(address,index,&buffer[0],1);
	}else
	{
		status = VL53L0X_write_multiII(address,index,buffer,2);
	}
	
	return status;
}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_write_dwordII(u8 address,u8 index,u32 data)
{
	
    u8 status = STATUS_OKII;

    u8 buffer[4];	
	
	//��32λ���ݲ�ֳ�8λ
	buffer[0] = (u8)(data>>24);
	buffer[1] = (u8)((data&0xff0000)>>16);
	buffer[2] = (u8)((data&0xff00)>>8);
	buffer[3] = (u8)(data&0xff);
	
	status = VL53L0X_write_multiII(address,index,buffer,4);
	
	return status;
	
}


//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_read_byteII(u8 address,u8 index,u8 *pdata)
{
	u8 status = STATUS_OKII;
	 
	status = VL53L0X_read_multiII(address,index,pdata,1);
	
	return status;
	 
}

//VL53L0X ��������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_read_wordII(u8 address,u8 index,u16 *pdata)
{
	u8 status = STATUS_OKII;
	
	u8 buffer[2];
	
	status = VL53L0X_read_multiII(address,index,buffer,2);
	
	*pdata = ((u16)buffer[0]<<8)+(u16)buffer[1];
	
	return status;
	
}

//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_read_dwordII(u8 address,u8 index,u32 *pdata)
{
	u8 status = STATUS_OKII;
	
	u8 buffer[4];
	
	status = VL53L0X_read_multiII(address,index,buffer,4);
	
	*pdata = ((u32)buffer[0]<<24)+((u32)buffer[1]<<16)+((u32)buffer[2]<<8)+((u32)buffer[3]);
	
	return status;
	
}












