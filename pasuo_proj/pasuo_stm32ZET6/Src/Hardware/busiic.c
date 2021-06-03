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




/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/


void IIC_Init(void)
{			
	//SDA_OUT();     //sda线输出
	IIC_SDA(GPIO_PIN_SET);;	  	  
	IIC_SCL(GPIO_PIN_SET);
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA(GPIO_PIN_SET);	  	  
	IIC_SCL(GPIO_PIN_SET);
	
	Delay(10);
 	IIC_SDA(GPIO_PIN_RESET);//START:when CLK is high,DATA change form high to low 
	
	Delay(10);
	IIC_SCL(GPIO_PIN_RESET);//钳住I2C总线，准备发送或接收数据 
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA(GPIO_PIN_RESET);//STOP:when CLK is high DATA change form low to high
 	
		Delay(5);
	IIC_SCL(GPIO_PIN_SET); 
	IIC_SDA(GPIO_PIN_SET);;//发送I2C总线结束信号
	
		Delay(5);							   	
}

/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0; 
	SDA_IN();      //SDA设置为输入  
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
	IIC_SCL(GPIO_PIN_RESET);//时钟输出0  
	return 0;  
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
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
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
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

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(uint8_t txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t; 
		SDA_OUT(); 	    
    IIC_SCL(GPIO_PIN_RESET);//拉低时钟开始数据传输
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
   
/**************************实现函数********************************************
*函数原型:		uint8_t IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
}



/**************************实现函数********************************************
*函数原型:		uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
    uint8_t count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1 |0);	   //发送写命令
	if(IIC_Wait_Ack()){
        IIC_Stop();
        return 1;
    }
	IIC_Send_Byte(reg);   //发送地址
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)|1);  //进入接收模式	
	IIC_Wait_Ack();
	
//    for(count=0;count<length;count++){
//		 
//		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
//		 	else  data[count]=IIC_Read_Byte(0);	 //最后一个字节NACK
//	}
    while(length)
	{
		if(length==1)*data=IIC_Read_Byte(0);//读数据,发送nACK 
		else *data=IIC_Read_Byte(1);		//读数据,发送ACK  
		length--;
		data++; 
	}  
    IIC_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:		uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data){
  
 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
 }
	IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
	
}



////////////////////////////////////////////////////////////////////



u8 VL_IIC_Write_1Byte(u8 SlaveAddress, u8 REG_Address,u8 REG_data)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);
	if(IIC_Wait_Ack())
	{
		IIC_Stop();//释放总线
		return 1;//没应答则退出

	}
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();	
	IIC_Send_Byte(REG_data);
	IIC_Wait_Ack();	
	IIC_Stop();

	return 0;
}

//IIC读一个字节数据
u8 VL_IIC_Read_1Byte(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);//发写命令
	if(IIC_Wait_Ack())
	{
		 IIC_Stop();//释放总线
		 return 1;//没应答则退出
	}		
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();
	IIC_Start(); 
	IIC_Send_Byte(SlaveAddress|0x01);//发读命令
	IIC_Wait_Ack();
	*REG_data = IIC_Read_Byte(0);
	IIC_Stop();

	return 0;
}

//IIC写n字节数据
u8 VL_IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address,u16 len, u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);//发写命令
	if(IIC_Wait_Ack()) 
	{
		IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();
	while(len--)
	{
		IIC_Send_Byte(*buf++);//发送buff的数据
		IIC_Wait_Ack();	
	}
	IIC_Stop();//释放总线

	return 0;
	
}

//IIC读n字节数据
u8 VL_IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address,u16 len,u8 *buf)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress);//发写命令
	if(IIC_Wait_Ack()) 
	{
		IIC_Stop();//释放总线
		return 1;//没应答则退出
	}
	IIC_Send_Byte(REG_Address);
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Send_Byte(SlaveAddress|0x01);//发读命令
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
	IIC_Stop();//释放总线

	return 0;
	
}

//VL53L0X 写多个数据
//address:地址
//index:偏移地址
//pdata:数据指针
//count:长度 最大65535
u8 VL53L0X_write_multi(u8 address, u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK;

	if(VL_IIC_Write_nByte(address,index,count,pdata))
	{
	   status  = STATUS_FAIL;

	}

	return status;
}


//VL53L0X 读多个数据
//address:地址
//index:偏移地址
//pdata:数据指针
//count:长度 最大65535
u8 VL53L0X_read_multi(u8 address,u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK;

	if(VL_IIC_Read_nByte(address,index,count,pdata))
	{
	  status  = STATUS_FAIL;
	}

	return status;
}

//VL53L0X 写1个数据(单字节)
//address:地址
//index:偏移地址
//data:数据(8位)
u8 VL53L0X_write_byte(u8 address,u8 index,u8 data)
{
	u8 status = STATUS_OK;

	status = VL53L0X_write_multi(address,index,&data,1);

	return status;
}

//VL53L0X 写1个数据(双字节)
//address:地址
//index:偏移地址
//data:数据(16位)
u8 VL53L0X_write_word(u8 address,u8 index,u16 data)
{
	u8 status = STATUS_OK;
	
	u8 buffer[2];
	
	//将16位数据拆分成8位
	buffer[0] = (u8)(data>>8);//高八位
	buffer[1] = (u8)(data&0xff);//低八位
	
	if(index%2==1)
	{  
		//串行通信不能处理对非2字节对齐寄存器的字节
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
		status = VL53L0X_write_multi(address,index,&buffer[0],1);
	}else
	{
		status = VL53L0X_write_multi(address,index,buffer,2);
	}
	
	return status;
}

//VL53L0X 写1个数据(四字节)
//address:地址
//index:偏移地址
//data:数据(32位)
u8 VL53L0X_write_dword(u8 address,u8 index,u32 data)
{
	
    u8 status = STATUS_OK;

    u8 buffer[4];	
	
	//将32位数据拆分成8位
	buffer[0] = (u8)(data>>24);
	buffer[1] = (u8)((data&0xff0000)>>16);
	buffer[2] = (u8)((data&0xff00)>>8);
	buffer[3] = (u8)(data&0xff);
	
	status = VL53L0X_write_multi(address,index,buffer,4);
	
	return status;
	
}


//VL53L0X 读1个数据(单字节)
//address:地址
//index:偏移地址
//data:数据(8位)
u8 VL53L0X_read_byte(u8 address,u8 index,u8 *pdata)
{
	u8 status = STATUS_OK;
	 
	status = VL53L0X_read_multi(address,index,pdata,1);
	
	return status;
	 
}

//VL53L0X 读个数据(双字节)
//address:地址
//index:偏移地址
//data:数据(16位)
u8 VL53L0X_read_word(u8 address,u8 index,u16 *pdata)
{
	u8 status = STATUS_OK;
	
	u8 buffer[2];
	
	status = VL53L0X_read_multi(address,index,buffer,2);
	
	*pdata = ((u16)buffer[0]<<8)+(u16)buffer[1];
	
	return status;
	
}

//VL53L0X 读1个数据(四字节)
//address:地址
//index:偏移地址
//data:数据(32位)
u8 VL53L0X_read_dword(u8 address,u8 index,u32 *pdata)
{
	u8 status = STATUS_OK;
	
	u8 buffer[4];
	
	status = VL53L0X_read_multi(address,index,buffer,4);
	
	*pdata = ((u32)buffer[0]<<24)+((u32)buffer[1]<<16)+((u32)buffer[2]<<8)+((u32)buffer[3]);
	
	return status;
	
}












