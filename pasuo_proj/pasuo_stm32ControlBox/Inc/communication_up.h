#ifndef __UPPER_COMM_H
#define __UPPER_COMM_H

#include <stdint.h>
#include "cmsis_os.h"




void initCommunication_up(void);

//#define RX_BUF_LEN (1024)

//typedef struct 
//{
//    uint8_t     RxBuf[RX_BUF_LEN];
//    int16_t    RxLen;
//} RxBuf_s;

//#define RxIdleLen(u) (RX_BUF_LEN - u.RxLen)

//uint16_t sendData(uint8_t *Buf, uint16_t Len);
//uint16_t readData(uint8_t *Buf, uint16_t IdleLen);



extern QueueHandle_t      QueueHandle;
#endif
