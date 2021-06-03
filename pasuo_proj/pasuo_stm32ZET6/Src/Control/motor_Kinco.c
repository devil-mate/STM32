#include "motor_Kinco.h"
#include "modbus.h"

void initRotaMotor()
{
	uint16_t data=0x0F;
	WriteWordToSlave(MB_REG_CON,data);//使能
	osDelay(10);
	WriteWordToSlave(MB_REG_MODE,3);//控制模式
	osDelay(10);
	WriteDWordToSlave(MB_REG_SPEED,0);//速度0
	osDelay(10);
	//WriteWordToSlave(MB_REG_CON,0x3F);//绝对定位模式
	
	
}

uint8_t resetMotor()
{
	WriteWordToSlave(MB_REG_CON,0x06);//断电失能
	initRotaMotor();
	
}

uint8_t setMotorAngle(float angle)
{
	uint32_t pos;
	pos=angle*10000; //temp
	WriteWordToSlave(MB_REG_POS,pos);//位置
	
}
uint8_t setMotorSpeed_Kinco()
{
}



