#include "TaskList.h"
//#include "UartDirver.h" 
#include "BackConter.h"
static void TaskPerformerTask(void * argument);
typedef struct
{
	TaskFunction_t Function;
	void * Parameter;
}TaskInfoForList;

TaskPerformer_s * ExePerformer=NULL;
TaskPerformer_s * CreateTaskPerformer(configSTACK_DEPTH_TYPE StackSize,uint8_t TaskBufSize,char * TaskName)
{
	TaskPerformer_s * TaskPerformerHandle;
	TaskPerformerHandle=pvPortMalloc(sizeof(TaskPerformer_s));
	if(TaskPerformerHandle==NULL)
		return NULL;
	
	TaskPerformerHandle->TaskList=xQueueCreate( 10, sizeof(TaskInfoForList) );

	TaskPerformerHandle->ResumeSemaphore=xSemaphoreCreateBinary();
	//TaskPerformerHandle->BufMutex=xSemaphoreCreateMutex();
	TaskPerformerHandle->BufMutex=xSemaphoreCreateBinary();
	
	if(pdPASS!=xTaskCreate( TaskPerformerTask,   
							 TaskName,     	
							 StackSize,               	
							 TaskPerformerHandle,    //²ÎÊý          	
							 tskIDLE_PRIORITY,                 	
							 &(TaskPerformerHandle->MainTaskIF) ))
	{
		vPortFree(TaskPerformerHandle);
		return NULL;
	}	
	return TaskPerformerHandle;
	
}
int8_t AddTask(TaskPerformer_s * TaskPerformerHandle,TaskFunction_t Function,void * Parameter)
{
	TaskInf_s info;
	
	info.Function=Function;
	info.Parameter=Parameter;
	
	xQueueSend( TaskPerformerHandle->TaskList, (const void *) &info, portMAX_DELAY );
	
	return 0;
}

int8_t AddTaskFromISR(TaskPerformer_s * TaskPerformerHandle,TaskFunction_t Function,void * Parameter)
{
	TaskInf_s info;
	BaseType_t  xHigherPriorityTaskWoken = pdFALSE;
	info.Function=Function;
	info.Parameter=Parameter;
	xQueueSendFromISR( TaskPerformerHandle->TaskList, (const void *) &info, &xHigherPriorityTaskWoken );

	return 0;
}

static void TaskPerformerTask(void * argument)
{
	TaskInf_s info;
	BaseType_t QueueReturn;
	TaskPerformer_s * TaskPerformerHandle=(TaskPerformer_s * )argument;
	 while(1)
	 {		 
		 QueueReturn=xQueueReceive( TaskPerformerHandle->TaskList, &info,portMAX_DELAY );
		 if(QueueReturn==pdPASS)
			info.Function(info.Parameter);

	 }	
}
void IniTaskPerformer()
{
	
	ExePerformer=CreateTaskPerformer(512,10,"ExePerformer");
}