#include "modbus.h"
#include <stdint.h>
#include "rs485_it.h"
//#include "GeneralTIM/bsp_GeneralTIM.h"
//#include "led/bsp_led.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/

// CRC ��λ�ֽ�ֵ��
static const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
// CRC ��λ�ֽ�ֵ��
static const uint8_t auchCRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
/* ��չ���� ------------------------------------------------------------------*/
//extern __IO MSG_TypeDef Rx_MSG ;   // ���ձ���״̬
/* ˽�к���ԭ�� --------------------------------------------------------------*/
void WaitTimeOut(void );
void PrintHexArray(uint8_t *Str,uint16_t Num);
uint32_t ReadData_Parser(void);
/* ������ --------------------------------------------------------------------*/

/** 
  * ��������: Modbus CRC16 У����㺯��
  * �������: pushMsg:������������׵�ַ,usDataLen:���ݳ���
  * �� �� ֵ: CRC16 ������
  * ˵    ��: �������Ǹ�λ��ǰ,��Ҫת�����ܷ���
  */
uint16_t MB_CRC16(uint8_t *pushMsg,uint8_t usDataLen)
{
  uint8_t uchCRCHi = 0xFF;
  uint8_t uchCRCLo = 0xFF;
  uint16_t uIndex;
  while(usDataLen--)
  {
    uIndex = uchCRCLo ^ *pushMsg++;
    uchCRCLo = uchCRCHi^auchCRCHi[uIndex];
    uchCRCHi = auchCRCLo[uIndex];
  }
  return (uchCRCHi<<8|uchCRCLo);
}

/** 
  * ��������: ��N����Ȧ״̬(CoilStatue)
  * �������: _addr:��վ��ַ,_reg:�Ĵ�����ַ,_num:����ȡ������
  * �� �� ֵ: ��
  * ˵    ��: ������ݷ��ͻ�����,Ȼ����
  */
void MB_ReadCoil_01H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x01;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _num >> 8;	  /* ��Ȧ(bit)���� ���ֽ� */
	Tx_Buf[TxCount++] = _num;		      /* ��Ȧ(bit)���� ���ֽ� */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * ��������: д������Ȧ״̬(CoilStatue)
  * �������: _addr:��վ��ַ,_reg:�Ĵ�����ַ,_sta:��д�����Ȧ״̬(0,1)
  * �� �� ֵ: ��
  * ˵    ��: ������ݷ��ͻ�����,Ȼ����
  */
void MB_WriteCoil_05H(uint8_t _addr, uint16_t _reg, uint16_t _sta)
{
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x05;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _sta >> 8;	  /* ��Ȧ(bit)���� ���ֽ� */
	Tx_Buf[TxCount++] = _sta;		      /* ��Ȧ(bit)���� ���ֽ� */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * ��������: ������״̬״̬(InputStatue)
  * �������: _addr:��վ��ַ,_reg:�Ĵ�����ַ,_num:����ȡ����������
  * �� �� ֵ: ��
  * ˵    ��: ������ݷ��ͻ�����,Ȼ����
  */
void MB_ReadInput_02H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x02;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _num >> 8;	  /* ����(Input)���� ���ֽ� */
	Tx_Buf[TxCount++] = _num;		      /* ����(Input)���� ���ֽ� */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * ��������: �����ּĴ���(HoldingRegister)
  * �������: _addr:��վ��ַ,_reg:�Ĵ�����ַ,_num:����ȡ�ļĴ�������
  * �� �� ֵ: ��
  * ˵    ��: ������ݷ��ͻ�����,Ȼ����
  */
void MB_ReadHoldingReg_03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x03;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _num >> 8;	  /* �Ĵ���(16bits)���� ���ֽ� */
	Tx_Buf[TxCount++] = _num;		      /* ���ֽ� */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * ��������: д�������ּĴ���(HoldingRegister)
  * �������: _addr:��վ��ַ,_reg:�Ĵ�����ַ,_data:��д��ļĴ�������
  * �� �� ֵ: ��
  * ˵    ��: ������ݷ��ͻ�����,Ȼ����
  */
void MB_WriteHoldingReg_06H(uint8_t _addr, uint16_t _reg, uint16_t _data)
{
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x06;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _data >> 8;	  /* �Ĵ���(16bits)���� ���ֽ� */
	Tx_Buf[TxCount++] = _data;		    /*  ���ֽ� */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}
void MB_WriteHoldingRegD_06H(uint8_t _addr, uint16_t _reg, uint32_t _data)
{
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x06;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _data >> 24;	  /* �Ĵ���(16bits)���� ���ֽ� */
	Tx_Buf[TxCount++] = _data >> 16;		    /*  ���ֽ� */
	Tx_Buf[TxCount++] = _data >> 8;	  /* �Ĵ���(16bits)���� ���ֽ� */
	Tx_Buf[TxCount++] = _data;		    /*  ���ֽ� */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * ��������: ��N������Ĵ���(InputRegister)
  * �������: _addr:��վ��ַ,_reg:�Ĵ�����ַ,_num:����ȡ�ļĴ�������
  * �� �� ֵ: ��
  * ˵    ��: ������ݷ��ͻ�����,Ȼ����.
  */
void MB_ReadInputReg_04H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x04;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _num >> 8;	  /* �Ĵ���(16bits)���� ���ֽ� */
	Tx_Buf[TxCount++] = _num;		      /*  ���ֽ� */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * ��������: дN�����ּĴ���(HoldingRegister)
  * �������: _addr:��վ��ַ,_reg:�Ĵ�����ַ,_num:��д��ļĴ�������,_databuf:��д��ļĴ�������
  * �� �� ֵ: ��
  * ˵    ��: ������ݷ��ͻ�����,Ȼ����._databuf�ĳ����� >= _num*2
  */
void MB_WriteNumHoldingReg_10H(uint8_t _addr, uint16_t _reg, uint16_t _num,uint8_t *_databuf)
{
  uint16_t i;
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x10;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _num >> 8;	  /* �Ĵ���(16bits)���� ���ֽ� */
	Tx_Buf[TxCount++] = _num;		      /*  ���ֽ� */
  Tx_Buf[TxCount++] = _num<<1;		  /* ���ݸ��� */

  for (i = 0; i < 2 * _num; i++)
  {
		Tx_Buf[TxCount++]  = _databuf[i];		/* ��������ݳ��� */
	}
	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}
/********************************************************************* 
  * ���º���������SKIV100Aϵ��ʸ��ͨ���ͱ�Ƶ��ͨ��
  *
*********************************************************************/

/** 
  * ��������: ��ʱ�ȴ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��������֮��,�ȴ��ӻ���Ӧ,200ms֮������Ϊ��ʱ
  */
void WaitTimeOut(void )
{
  uint16_t TimeOut  = 0;// ͨѶ��ʱ ��λ:ms
  TimeOut = TIME_OVERRUN; // ���峬ʱʱ��Ϊ100ms,��ʵ�ʲ���ʱ��Ϊ200ms
  while(Rx_MSG != MSG_COM)
  {
    HAL_Delay(1);
    if(TimeOut-- == 0)
    {
      if(Rx_MSG != MSG_COM)     // 200ms����û�н������ݣ�����Ϊ��ʱ
      {
        Rx_MSG = MSG_OT;
        break;
      }
    }
  }
}

/** 
  * ��������: ��16���Ƶĸ�ʽ��ӡ��������
  * �������: _RegAddr:�Ĵ�����ַ
  * �� �� ֵ: ��ȡ����16λ����
  * ˵    ��: �ӱ�Ƶ����ȡһ���ֵ�����
  */
void PrintHexArray(uint8_t *Str,uint16_t Num)
{
  uint16_t i = 0;
  for(; i < Num; i++)
  {
    printf("%02X ",Str[i]);
  }
}

/** 
  * ��������: ���ݽ���
  * �������: ��
  * �� �� ֵ: ��ȡ����16/32λ����
  * ˵    ��: �ӽ���ͨ�Ż�������ȡ32λ��Ч���ݳ�����
  */
uint32_t ReadData_Parser()
{
  uint16_t crc_check = 0;
  uint32_t RxData = 0;
  switch(Rx_MSG)
  {
    case MSG_COM:
      /* �յ��������Ĵ�վ���������� */
      if(Rx_Buf[0] != MB_SLAVEADDR)
      {
        printf(" MB_SLAVEADDR Error!\n ");
        //LED2_TOGGLE;
        break; 
      }
      else
      {
        /*����CRC������ */
        crc_check = ( (Rx_Buf[RxCount-1]<<8) | Rx_Buf[RxCount-2] );
        /* CRC У����ȷ */
        if(crc_check == MB_CRC16((uint8_t*)&Rx_Buf,RxCount-2)) 
        {
          /* ͨ�ų��� */
          if(Rx_Buf[1]&0x80)
          {
            printf(" Communication error!\n");
            printf(" Error Code: ");
            PrintHexArray((uint8_t*)&Rx_Buf,8);
            printf("\n");
            //LED2_TOGGLE;
            HAL_Delay(100);
            break; 
          }
          /* ͨ����ȷ ����ȡ���� */
          if(Rx_Buf[2] == 0x02)
            RxData = (Rx_Buf[3]<<8) | Rx_Buf[4];
          else 
            RxData = (Rx_Buf[3]<<24) | (Rx_Buf[4]<<16) | (Rx_Buf[5]<<8) | Rx_Buf[6];
        }
      } 
      break;
      
    case MSG_OT: printf("TimeOut��no response from slave\n");
    default:     printf("Communication failed!\n");
     // LED2_TOGGLE;
      break;
  }
  return RxData;
}
/** 
  * ��������: ��ȡһ���ֵ�����
  * �������: _RegAddr:�Ĵ�����ַ
  * �� �� ֵ: ��ȡ����16λ����
  * ˵    ��: �ӱ�Ƶ����ȡһ���ֵ�����
  */
uint16_t ReadWordFromSlave(uint16_t _RegAddr)
{
  uint16_t RxData = 0;
  Rx_MSG = MSG_IDLE;
  /* ��ȡreg_addr �ļĴ���״̬ */
  MB_ReadHoldingReg_03H(MB_SLAVEADDR, _RegAddr, 0x0001);
  /* �ȴ��ӻ���Ӧ */
  WaitTimeOut();    //�ȴ�ʱ����Ϊ200ms 
  
  RxData = (uint16_t)ReadData_Parser();
  /* ���±��Ϊ����״̬ */
  Rx_MSG = MSG_IDLE;
  return RxData;
}
/** 
  * ��������: ��ȡ2���֣�32bits��������
  * �������: _RegAddr:�Ĵ�����ַ
  * �� �� ֵ: ��ȡ����32λ����
  * ˵    ��: �ӱ�Ƶ����ȡ�����ֵ�����
  */
uint32_t ReadDWordFromSlave(uint16_t _RegAddr)
{
  uint32_t RxData = 0;
  Rx_MSG = MSG_IDLE;
  /* ��ȡreg_addr �ļĴ���״̬ */
  MB_ReadHoldingReg_03H(MB_SLAVEADDR, _RegAddr, 0x0002);
  /* �ȴ��ӻ���Ӧ */
  WaitTimeOut();    // �ȴ�ʱ����Ϊ200ms 
  RxData = (uint32_t)ReadData_Parser();    // 
  
  /* ���±��Ϊ����״̬ */
  Rx_MSG = MSG_IDLE;
  return RxData;
}

/** 
  * ��������: дһ����WORD��16bits��
  * �������: _RegAddr | �Ĵ�����ַ
  *           Data | д�������
  * �� �� ֵ: д�������
  * ˵    ��: дһ���֣�16bits�������豸
  */
uint16_t WriteWordToSlave(uint16_t _RegAddr,uint16_t Data)
{
  Rx_MSG = MSG_IDLE;
  /* д��һ���� */
  MB_WriteHoldingReg_06H(MB_SLAVEADDR,_RegAddr,Data);
  /* �ȴ��ӻ���Ӧ */
  WaitTimeOut();    // �ȴ�ʱ����Ϊ200ms 
  ReadData_Parser();
  /* ���±��Ϊ����״̬ */
  Rx_MSG = MSG_IDLE;
  return Data;
}

uint16_t WriteDWordToSlave(uint16_t _RegAddr,int32_t Data)
{
  Rx_MSG = MSG_IDLE;
	uint8_t temp[4]={0};
	temp[0]= Data >> 8; //��λ���ݸ��ֽ�
	temp[1]= Data; //��λ���ݵ��ֽ�
	temp[2]= Data >> 24; //��λ���ݸ��ֽ�
	temp[3]= Data >> 16;
  /* д��D */
  MB_WriteNumHoldingReg_10H(MB_SLAVEADDR,_RegAddr,2,temp);
  /* �ȴ��ӻ���Ӧ */
  WaitTimeOut();    // �ȴ�ʱ����Ϊ200ms 
  ReadData_Parser();
  /* ���±��Ϊ����״̬ */
  Rx_MSG = MSG_IDLE;
  return Data;
}

/** 
  * ��������: ͨѶ��·���
  * �������: _addr:��վ��ַ,_reg:������,_num:��������
  * �� �� ֵ: ��
  * ˵    ��: ��Ƶ�����й��ܣ����ڶԱ�Ƶ������·��⣬�ӻ���
  *           ����һ�������ݻ���
  */
void MB_ReadHoldingReg_08H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
	uint16_t TxCount = 0;
  uint16_t crc = 0;
	Tx_Buf[TxCount++] = _addr;		    /* ��վ��ַ */
	Tx_Buf[TxCount++] = 0x08;		      /* ������ */	
	Tx_Buf[TxCount++] = _reg >> 8;	  /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _reg;		      /* �Ĵ�����ַ ���ֽ� */
	Tx_Buf[TxCount++] = _num >> 8;	  /* �Ĵ���(16bits)���� ���ֽ� */
	Tx_Buf[TxCount++] = _num;		      /* ���ֽ� */

	crc = MB_CRC16((uint8_t*)&Tx_Buf,TxCount);
  Tx_Buf[TxCount++] = crc;	          /* crc ���ֽ� */
	Tx_Buf[TxCount++] = crc>>8;		      /* crc ���ֽ� */
  RS485_Tx((uint8_t *)&Tx_Buf,TxCount);
}

/** 
  * ��������: ͨ���豸��⹦��
  * �������: _SlaveAddr | �豸��ַ
  * �� �� ֵ: ���豸��ַ��0x00 ��ʾ���豸����
  * ˵    ��: ����ض��豸�Ƿ��Ѿ�����,
  */
void HW_Identify(uint16_t _SlaveAddr)
{
  uint32_t RxData = 0;
  uint32_t Verify = 0x1234;
  Rx_MSG = MSG_IDLE;
  MB_ReadHoldingReg_08H(_SlaveAddr, 0x0000, Verify);
  /* �ȴ��ӻ���Ӧ */
  WaitTimeOut();    // �ȴ�ʱ����Ϊ200ms 
  RxData = (uint32_t)ReadData_Parser();    // 
  RxData = (uint16_t)(RxData >>8);
  if(RxData == Verify)
  {
    /* ��ȷ��Ӧ */
    printf("Device Num: %d Effective Communication.\n",_SlaveAddr);
  }
  else
  {
    /* ����ȷ��Ӧ */
    printf("Ineffective Communication.\n");
    //LED2_TOGGLE;
  }
}
void RS485_Tx(uint8_t *Tx_Buf,uint16_t TxCount)
{
  RS485_TX_MODE();
	HAL_UART_Transmit(&huart3, Tx_Buf, TxCount, 0xffff);
  RS485_RX_MODE();
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
