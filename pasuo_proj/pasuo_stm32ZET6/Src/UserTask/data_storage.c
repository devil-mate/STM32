/*
 * data_storage.c
 * 4字节对齐存储，每次存储最后多出4字节本次写入长度信息
 *  Created on: 2019年11月13日
 *      Author: lefwin
 */

#include "data_storage.h"

#include "stm32f4xx_hal.h"
#include <stdio.h>

#define STM32F407xG_

#if defined(STM32F411xE_) || defined(STM32F407xE_)
#define SECTOR_CNT 8
#elif defined STM32F407xG_
#define SECTOR_CNT 12
#endif

#define TAIL_LENGTH 4    //写入数据后添加的尾部，当前为仅添加数据长度

typedef struct {
    uint8_t sector;
    uint8_t size_kb;
    uint32_t startAddr;
    uint32_t endAddr;
    uint32_t virginAddr;
    uint8_t aligin4Bytes;

} DataStorage_s;

static DataStorage_s dataStorage;

static uint8_t STM32F4_SECTOR_SIZE_KB[12] = { 16, 16, 16, 16, 64, 128, 128, 128, 128, 128, 128, 128 };

static int8_t saveDataInByte(uint8_t* src, uint32_t len);
static uint32_t getSectorEndAddr(uint8_t sector);
static uint32_t getSectorVirginAddr(DataStorage_s *pStorage);
static void eraseSector(DataStorage_s* dataStorage);
static int8_t saveDataAlign(uint8_t* src, uint32_t len);
static int8_t saveDataInByte(uint8_t* src, uint32_t len);

static uint32_t getSectorBeginAddr(uint8_t sector) {

    uint32_t sum = 0;
    for (int i = 0; i < sector; ++i) {
        sum += STM32F4_SECTOR_SIZE_KB[i];
    }
    return FLASH_BASE + sum * 1024;
}

static uint32_t getSectorEndAddr(uint8_t sector) {

    return (uint32_t) getSectorBeginAddr(sector) + STM32F4_SECTOR_SIZE_KB[sector] * 1024 - 1;
}

static uint32_t getSectorVirginAddr(DataStorage_s *pStorage) {

    uint32_t last = pStorage->endAddr - 3;

    while (*((uint32_t*) last) == 0xFFFFFFFF && last >= pStorage->startAddr) {
        last -= 4;
    }

    return last + 4;
}

int8_t initDataStorageAlign(uint8_t sector_index, uint8_t align) {

    if (sector_index >= SECTOR_CNT) {
        return -1;
    }

    dataStorage.sector = sector_index;
    dataStorage.size_kb = STM32F4_SECTOR_SIZE_KB[sector_index];
    dataStorage.startAddr = getSectorBeginAddr(sector_index);
    dataStorage.endAddr = getSectorEndAddr(sector_index);
    dataStorage.virginAddr = getSectorVirginAddr(&dataStorage);
    dataStorage.aligin4Bytes = (align != 0);
    return 0;
}

int8_t initDataStorage(uint8_t sector_index) {

    return initDataStorageAlign(sector_index, 1);
}

static void eraseSector(DataStorage_s* dataStorage) {

    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t SectorError;
    pEraseInit.TypeErase = TYPEERASE_SECTORS;
    pEraseInit.Sector = dataStorage->sector;
    pEraseInit.NbSectors = 1;
    pEraseInit.VoltageRange = VOLTAGE_RANGE_3;
    while (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK) {
    }
    dataStorage->virginAddr = dataStorage->startAddr;
}

static void saveInAlign(uint8_t* src, uint32_t len, DataStorage_s* dataStorage) {

    while (len / 4) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dataStorage->virginAddr, *((uint32_t*) (src)));
        dataStorage->virginAddr += 4;
        src += 4;
        len -= 4;
    }
    if (len > 0) {
        uint32_t data = *((uint32_t*) (src)) & (0xFFFFFFFF >> ((4 - len) * 8)); //去掉超出长度的尾部字节，即去掉高位
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dataStorage->virginAddr, data);
        dataStorage->virginAddr += 4;
    }
}

static int8_t saveDataAlign(uint8_t* src, uint32_t len) {

    uint32_t roundLen = (len + 3) / 4 * 4; //圆整到4的整数倍
    if (roundLen > (dataStorage.size_kb * 1024 - TAIL_LENGTH)) { //一次性写入长度不得大于扇区大小-TAIL_LENGTH个字节
        return -1;
    }

    int8_t ret = 0;
    HAL_FLASH_Unlock();
    int32_t freeSpace = (int32_t) (dataStorage.endAddr - dataStorage.virginAddr + 1);
//    printf("needed=%d\t free=%d...\r\n", roundLen + TAIL_LENGTH, freeSpace);
    if ((roundLen + TAIL_LENGTH) > freeSpace) {
        //剩余空间不够，需先格式化
//        printf("erase flash...\r\n");
        eraseSector(&dataStorage);
        ret = 1;
    }

    saveInAlign(src, len, &dataStorage);

    //保存长度值
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dataStorage.virginAddr, len);
    dataStorage.virginAddr += TAIL_LENGTH;

    HAL_FLASH_Lock();

    return ret;
}

static int8_t saveDataInByte(uint8_t* src, uint32_t len) {

    if (len > (dataStorage.size_kb * 1024 - TAIL_LENGTH)) { //一次性写入长度不得大于扇区大小-TAIL_LENGTH个字节
        return -1;
    }

    int8_t ret = 0;
    HAL_FLASH_Unlock();
    int32_t freeSpace = (int32_t) (dataStorage.endAddr - dataStorage.virginAddr + 1);
//    printf("needed=%d\t free=%d...\r\n", len + TAIL_LENGTH, freeSpace);
    if ((len + TAIL_LENGTH) > freeSpace) {
        //剩余空间不够，需先格式化
//        printf("erase flash...\r\n");
        eraseSector(&dataStorage);
        ret = 1;
    }

    int tempLen = len;
    while (tempLen--) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, dataStorage.virginAddr++, *((uint32_t*) (src++)));
    }

    //保存长度值，按字节写入。若按WORD写入，可能会因为地址未对齐导致失败
    for (int i = 0; i < 4; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, dataStorage.virginAddr++, *((uint8_t*) (&len) + i));
    }

    HAL_FLASH_Lock();

//    printf("(virginAddr - 10) = 0x%x\r\n", dataStorage.virginAddr - 10);
    return ret;
}

int8_t saveData(uint8_t* src, uint32_t len) {

    if (dataStorage.aligin4Bytes) {
        return saveDataAlign(src, len);
    }
    return saveDataInByte(src, len);
}

uint32_t readDataFlash(uint8_t* dest, uint32_t len) {

    uint32_t readLen = MIN(*((uint32_t* )(dataStorage.virginAddr - 4)), len);
    uint32_t header;
    if (dataStorage.aligin4Bytes) {
        uint32_t roundLen = (readLen + 3) / 4 * 4; //圆整到4的整数倍
        header = dataStorage.virginAddr - roundLen - TAIL_LENGTH;
    } else {
        header = dataStorage.virginAddr - len - TAIL_LENGTH;
    }
    while (readLen--) {
        *(dest++) = *(uint8_t*) (header++);
    }
    return readLen;
}

uint32_t readPreviousData(uint8_t* dest, uint32_t len, int32_t prevIndex) {

    //TODO unimplemented
    return 0;
}

