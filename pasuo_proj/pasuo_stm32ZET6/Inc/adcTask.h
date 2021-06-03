#ifndef __ADCTEST_H__
#define __ADCTEST_H__
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void adc_DmaInit(void);
void getAdcValue(void);
void IniAdc(void);

extern float adcVal[7];
#endif