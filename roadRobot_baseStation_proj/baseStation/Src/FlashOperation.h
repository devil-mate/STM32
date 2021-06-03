#ifndef __FLASHOPER_H
#define __FLASHOPER_H
#include "stm32f2xx_hal.h"
uint8_t bsp_EraseCpuFlash(uint32_t _ulFlashAddr,uint32_t _uFlashSize);
uint8_t bsp_WriteCpuFlash(uint32_t _ulFlashAddr, uint8_t *_ucpSrc, uint32_t _ulSize);
uint8_t bsp_ReadCpuFlash(uint32_t _ulFlashAddr, uint8_t *_ucpDst, uint32_t _ulSize);
#endif