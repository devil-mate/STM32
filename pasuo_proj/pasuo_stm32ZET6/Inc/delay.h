#ifndef _DELAY_H
#define _DELAY_H
#include "cmsis_os.h"
static void delay_ms(int tim)
{
	osDelay(tim);
}
#endif
