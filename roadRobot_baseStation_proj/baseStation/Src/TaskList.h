#ifndef __TASKLIST_H__
#define __TASKLIST_H__
#include "cmsis_os.h"
#include "stdint.h"
//#define TaskBufSize (5)
typedef struct
{
	TaskFunction_t Function;
	void * Parameter;
}TaskInf_s;
typedef struct
{
	TaskHandle_t MainTaskIF;
	SemaphoreHandle_t  ResumeSemaphore;
	SemaphoreHandle_t  BufMutex;
	QueueHandle_t TaskList;
}TaskPerformer_s;
TaskPerformer_s * CreateTaskPerformer(configSTACK_DEPTH_TYPE StackSize,uint8_t TaskBufSize,char * TaskName);
int8_t AddTask(TaskPerformer_s * TaskPerformerHandle,TaskFunction_t Function,void * Parameter);
int8_t AddTaskFromISR(TaskPerformer_s * TP,TaskFunction_t Function,void * Parameter);
void IniTaskPerformer();


extern TaskPerformer_s * ExePerformer;
#endif