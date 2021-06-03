/*
 * data_storage.h
 *
 *  Created on: 2019Äê11ÔÂ13ÈÕ
 *      Author: lefwin
 */

#ifndef DATA_SAVER_DATA_STORAGE_H_
#define DATA_SAVER_DATA_STORAGE_H_

#include <stdint.h>

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

int8_t initDataStorageAlign(uint8_t sector_index, uint8_t align);

/**
 * align in default
 */
int8_t initDataStorage(uint8_t sector_num);

int8_t saveData(uint8_t* src, uint32_t len);
uint32_t readDataFlash(uint8_t* dest, uint32_t len);
uint32_t readPreviousData(uint8_t* dest, uint32_t len, int32_t prevIndex);

#endif /* DATA_SAVER_DATA_STORAGE_H_ */
