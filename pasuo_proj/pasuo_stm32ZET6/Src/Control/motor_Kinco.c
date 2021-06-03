#include "motor_Kinco.h"
#include "modbus.h"

void initRotaMotor()
{
	uint16_t data=0x0F;
	WriteWordToSlave(MB_REG_CON,data);//ʹ��
	osDelay(10);
	WriteWordToSlave(MB_REG_MODE,3);//����ģʽ
	osDelay(10);
	WriteDWordToSlave(MB_REG_SPEED,0);//�ٶ�0
	osDelay(10);
	//WriteWordToSlave(MB_REG_CON,0x3F);//���Զ�λģʽ
	
	
}

uint8_t resetMotor()
{
	WriteWordToSlave(MB_REG_CON,0x06);//�ϵ�ʧ��
	initRotaMotor();
	
}

uint8_t setMotorAngle(float angle)
{
	uint32_t pos;
	pos=angle*10000; //temp
	WriteWordToSlave(MB_REG_POS,pos);//λ��
	
}
uint8_t setMotorSpeed_Kinco()
{
}



