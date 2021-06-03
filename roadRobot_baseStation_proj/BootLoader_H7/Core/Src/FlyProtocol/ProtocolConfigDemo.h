#ifndef __PROTOCOLCONFIG_H__
#define __PROTOCOLCONFIG_H__
#ifdef __cplusplus
extern "C"
{
#endif
#define RECV_BUF_SIZE (256) //接收缓冲大小
#define SEND_BUF_SIZE (256) //发送缓冲大小
#define RECV_BUF_QUANTITY (2) //接收缓冲个数

#define RELPY_WAIT_SIZE (5) //重发帧跟踪个数
#define RELPY_TIME_LIMIT (5) //重发超时时间，单位为flyProtocolTick()调用间隔
#define RELPY_REPEAT_LIMIT (5) //超时重发最大次数

#ifdef __cplusplus
}
#endif
#endif