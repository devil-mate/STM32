#include "common.h"

#include <string.h>



int32_t avgFilter(int32_t in, uint8_t depth, uint8_t reset)
{
	static float sum = 0, mean;
	if(reset){
		sum = 0;
	}
	else
	{
		sum += (in - mean);
	}
	mean = sum / depth;
	
	return (int16_t)mean;
}



void movAvgFilterInit(MovAvgFilter_s *MovAvgFilters)
{
	memset(MovAvgFilters,0,sizeof(MovAvgFilter_s));
}

float movAvgFilter(MovAvgFilter_s *MovAvgFilters , float in, uint8_t depth, uint8_t reset)
{
	
	float sum = 0,mean;
	uint8_t number=0;	
	MovAvgFilters->inValue[MovAvgFilters->count++]=in;
	number=cMIN(VSIZE,depth);
	if(MovAvgFilters->count==number){MovAvgFilters->count=0;}
	for(int8_t i=0;i<number;i++){
		sum+=MovAvgFilters->inValue[i];
	}
	mean = sum / number;
	return mean;
}
//TODO =============
//float movAvgDisFilter(float in, uint8_t depth, uint8_t reset)
//{
//	
//	float sum = 0,mean;
//	inValue[count++]=in;
//	if(count==N){count=0;}
//	for(int8_t i=0;i<N;i++){
//		sum+=inValue[i];
//	}
//	mean = sum / N;
//	
//	return mean;
//}