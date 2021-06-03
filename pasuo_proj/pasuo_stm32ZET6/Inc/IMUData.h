#ifndef _IMUDATA_H
#define _IMUDATA_H
#include "busiic2.h"

void initIMUData();


#define AX					0x34




void ShortToChar(short sData,unsigned char cData[]);

short CharToShort(unsigned char cData[]);




#endif