#ifndef __FRAMECHECK_H__
#define __FRAMECHECK_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "FlyProtocolApp.h"
typedef  void (*iapfun)(void);	
#define PORM_ADDR (0x08020000)
#define FLAG_ADDR (0x08120000)
#define ID_ADDR (0x08100000)

void iniUpgrade();
void idleUpgradeTask();
void iap_load_app(unsigned long appxaddr);
#ifdef __cplusplus
}
#endif 
#endif