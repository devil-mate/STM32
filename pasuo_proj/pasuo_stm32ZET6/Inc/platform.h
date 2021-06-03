#ifndef __PLATFORM_H_
#define __PLATFORM_H_
#include<stdint.h>
/*电机选择，二选一*/
//#define NOLOGY_MOTOR    0
#define FAULHABER_MOTOR 1
//#define KEYA_MOTOR 0

/*平台结构参数*/
#define PLATFORM_WHEEL_DIA              (60.0f)//直径60mm
#define CODER_WHEEL_DIA              (50.0f)//直径60mm
#define PLATFORM_WHEEL_TREAD            (265.0f) 
#define PLATFORM_MAX_SPEED_M_PER_MIN    (20.0f)
#define PI  3.1415926f
#define	DIA_RATE						(0.8f) // 轮子的一定比例计算实际里程
#define PULS_NUM						(4000) //一圈脉冲数
#define PLATFORM_MAX_OMEGA              (1.0f)
#if (NOLOGY_MOTOR == 1)
    #define PLATFORM_GEAR_RATIO         (82*1.4f)

#elif (FAULHABER_MOTOR == 1)
    #define PLATFORM_GEAR_RATIO         (23*2.0f)
#elif (KEYA_MOTOR == 1)
    #define PLATFORM_GEAR_RATIO         (158*1.0f)

#endif



#endif // !__PLATFORM_H_
