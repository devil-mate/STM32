#include "FrameCheck.h"

unsigned char checkCalc(unsigned char *buf, unsigned int len)
{
	unsigned char sum=0;
	for(int n=0 ;n<len;n++)
	{
		sum+=buf[n];
	}	
	return sum;
}