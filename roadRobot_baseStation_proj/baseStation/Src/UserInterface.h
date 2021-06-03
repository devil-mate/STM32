#ifndef __USERINTERFACE_H__
#define __USERINTERFACE_H__
#include "stm32f2xx_hal.h"
#include "stdint.h"
#include "TaskList.h"
void IniUserInterface();
void CmdMessage(char * cmd , uint16_t len);
int userReplyInf(const char *format, ...);
int IllegalInstruction();
extern TaskPerformer_s * UIPerformer;
#endif