/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   STM3210B-EVAL Evaluation Board
*
* Filename      : bsp.c
* Version       : V1.00
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#define  BSP_MODULE
#include <FreeRTOS.h>
#include <task.h>

#include <bsp.h>
#include <stdio.h>
#include <cm_backtrace.h>
#include "main.h"
/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/
#define  DWT_CR      *(CPU_REG32 *)0xE0001000
#define  DWT_CYCCNT  *(CPU_REG32 *)0xE0001004
#define  DEM_CR      *(CPU_REG32 *)0xE000EDFC
#define  DEM_CR_TRCENA                   (1 << 24)
#define  DWT_CR_CYCCNTENA                (1 <<  0)
/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/**
 * RCC configuration
 */



int fgetc(FILE *f)
{
   
   return 0;
}

//void assert_failed(uint8_t* file, uint32_t line)
//{
//    /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//    /* Infinite loop */
//    cm_backtrace_assert(cmb_get_sp());
//    printf("assert failed at %s:%d \n", file, line);
//    while (1) {
//    }
//}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName )
{
    printf( "OverflowHook in %s \r\n", pcTaskName);  
    while(1);
}

void delay(uint32_t nCount)
{
    for(; nCount!= 0; nCount--);
}

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               BSP_Init()
*
* Description : Initialize the Board Support Package (BSP).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This function SHOULD be called before any other BSP function is called.
*********************************************************************************************************
*/

void  BSP_Init (void)
{

}

