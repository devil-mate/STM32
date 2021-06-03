#ifndef _MOTOR_FAULHABER_H
#define _MOTOR_FAULHABER_H

#define MOTOR_GROUP_NUM (1)
#define ONLINE_CHECK_TIMEOUT  (5)


#define initData()  {0x23,0x0C,0x20,0x01,0x00,0x00,0x00,0x00} //失能
//#define speedData()  {0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00} //
#define speedData()  {0x23,0x00,0x20,0x01,0x00,0x00,0x00,0x00} //
#define MAX_PWM_LIMIT   (5000)

typedef enum
{
    Func_Reset                          = 0x00,
    Func_SetMode                        = 0x01,
    Func_Cmd_VelocityMode               = 0x04,
    Func_Cmd_CurrVelocityMode           = 0x07,
    Func_Fb_Config                      = 0x0A,
    Func_Fb_CurrVelocity                = 0x0B,
    Func_Fb_Misc                        = 0x0C,
    Func_OnlineCheck                    = 0x0F
} NologyFuncCode;

typedef enum 
{
    OpenLoop_Mode                       =0x01,
    Current_Mode                        =0x02,
    Velocity_Mode                       =0x03,
    Position_Mode                       =0x04,
    Velocity_Position_Mode              =0x05,
    Current_Velocity_Mode               =0x06,
    Current_Position_Mode               =0x07,
    Current_Velocity_Position_Mode      =0x08
} NologyDrvMode;
#endif
