#ifndef __CRC_H__
#define __CRC_H__
#include <stdio.h>

unsigned short CRC_Cal (unsigned char *crc_buf,unsigned short crc_len);
unsigned char CRC_Verify (unsigned char *p,unsigned short len);
#endif
