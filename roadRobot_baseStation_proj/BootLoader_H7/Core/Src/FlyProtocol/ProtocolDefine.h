#ifndef __PROTOCOLDEFINE_H__
#define __PROTOCOLDEFINE_H__
#include "stdint.h"
#ifdef __cplusplus
extern "C"
{
#endif

#pragma pack(push)
#pragma pack(1)
    typedef struct
    {
        uint16_t head;
        uint16_t ver;
        uint32_t frameLen;
        uint64_t dev_id;
        uint64_t frame_id;
        uint32_t code;
    } ProtocolHead;
		typedef struct{
			 uint32_t mode;
			 uint64_t time;
			 double longitude;
			 double latitude;
			 double altitude;
		} ProtocolHeartbeat;
#pragma pack(pop)

typedef enum
{
    FLY_SEND_FAIL = 1
} Fly_Err_Code; 

#define CLOUD_CMD_HEAD (0x00FF)
#define CLOUD_CMD_RELAY_HEAD (0x00FE)
#define DEV_CMD_HEAD (0x00FB)
#define DEV_RELAY_HEAD (0x00FA)
#define ERR_RELAY_HEAD (0x00F9)

#ifdef __cplusplus
}
#endif
#endif