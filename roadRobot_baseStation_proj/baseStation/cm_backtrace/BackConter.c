#include "BackConter.h"
#include "cmb_cfg.h"
#define __ENUM_STR(e) #e
#define ENUM_STR(e) __ENUM_STR(e)
unsigned char callConter[CALL_END];
void iniConter()
{
	for(int n =0;n<CALL_END;n++)
		callConter[n]=0;
}

void printConter()
{
	for(int n =0;n<CALL_END;n++)
	{
		cmb_println("callConter %d = %d",n,callConter[n]);
	}
}

