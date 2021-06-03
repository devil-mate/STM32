#ifndef __USBDATA_H
#define __USBDATA_H
typedef enum
{
	  USB_SYNC=0,
    USB_CMD_REPLY,
    USB_IMU,
    USB_GPS,
    USB_GPS_RAW,
		USB_AUTO_INFO,
		USB_LOG,
		USB_FLY_DATA,
		USB_LIVE ,
} USB_Up_Code;
typedef enum
{
    USB_CMD = 0,
    USB_RTK = 1,
    USB_DATE = 2,
		USB_TIME = 3,
		USB_AUTO_CMD = 4,
		USB_FLY_CMD = 5,
		USB_DOWN_SIZE
} USB_Down_Code; 
#endif