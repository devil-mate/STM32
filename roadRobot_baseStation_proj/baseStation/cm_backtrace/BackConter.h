#ifndef __BACKCONTER_H__
#define __BACKCONTER_H__
typedef enum
{
    USB_SEND = 0,
	  GPGGA_PARSE,
    GPRMC_PARSE,
    GPHDT_PARSE,
		CAN_IT,
		USB_IT,
    CALL_END
} Call_C;
extern unsigned char callConter[CALL_END];
void iniConter();
void printConter();
//void uartPrintConter();
#endif