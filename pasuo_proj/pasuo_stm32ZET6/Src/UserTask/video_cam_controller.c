#include "cmsis_os.h"
#include <string.h>
#include "common.h"
#include "protocol_type.h"
#include "main.h"
//#include "video_cam_controller.h"

extern TIM_HandleTypeDef videoCamTIM;

static VideoCamCmd_s    LocalVideoCamCmd;
TaskHandle_t VideoCamControllerTaskHandle = NULL;

static void videoCamControllerTask(void *argument);
static void copyGlobal2Local(void);
static void cmd2Pwm(float yaw, float pitch);
static uint16_t angleToDuty(float angle, float span);

void initVideoCamController()
{
    HAL_TIM_PWM_Start(&videoCamTIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&videoCamTIM, TIM_CHANNEL_2);
    
    xTaskCreate(videoCamControllerTask,         /* 任务函数  */
                "VideoCamControllerTask",       /* 任务名    */
                512,                           /* 任务栈大小，单位word，也就是4字节 */
                NULL,                           /* 任务参数  */
                1,                              /* 任务优先级*/
                &VideoCamControllerTaskHandle); /* 任务句柄  */
}

static void videoCamControllerTask(void *argument)
{
    float yaw = 0.0f, pitch = 0.0f;
    uint8_t periodMs = 20;
    while (1)
    {
        osDelay(periodMs);
        
        copyGlobal2Local();
        if(LocalVideoCamCmd.digit & ANGLE_RESET_MASK)
        {
            yaw = 0.0f;
            pitch = 0.0f;
        }
        else
        {
            yaw -= LocalVideoCamCmd.yawAcc / (1000 / periodMs); //方向反向
            pitch += LocalVideoCamCmd.pitchAcc / (1000 / periodMs);
        }

        yaw = yaw > 0 ? cMIN(yaw, MAX_VIDEO_CAM_YAW) : cMAX(yaw, MIN_VIDEO_CAM_YAW);
        pitch = pitch > 0 ? cMIN(pitch, MAX_VIDEO_CAM_PITCH) : cMAX(pitch, MIN_VIDEO_CAM_PITCH);

        cmd2Pwm(yaw + YAW_OFFSET, pitch + PITCH_OFFSET);
    }
}

static void copyGlobal2Local(void)
{
    taskENTER_CRITICAL();
    memcpy(&LocalVideoCamCmd, &VideoCamCmd, sizeof(VideoCamCmd));
    taskEXIT_CRITICAL();
}

static void cmd2Pwm(float yaw, float pitch)
{
  
    uint16_t yawDuty = angleToDuty(yaw, VIDEO_CAM_YAW_MAX_SPAN);
    uint16_t pitchDuty = angleToDuty(pitch, VIDEO_CAM_PITCH_MAX_SPAN);
    
    log_debug("VideoCamCmd yaw:%6.3f, pitch:%6.3f | PwmDuty yaw:%4d, pitch:%4d", yaw, pitch, yawDuty, pitchDuty);

    __HAL_TIM_SET_COMPARE(&videoCamTIM, TIM_CHANNEL_YAW, yawDuty);
    __HAL_TIM_SET_COMPARE(&videoCamTIM, TIM_CHANNEL_PITCH, pitchDuty);
}

static uint16_t angleToDuty(float angle, float span)
{
    return (uint16_t)(angle * (MAX_PWM_DUTY - MIN_PWM_DUTY) / span + (MAX_PWM_DUTY + MIN_PWM_DUTY) / 2);
}
