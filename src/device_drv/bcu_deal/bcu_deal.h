#ifndef __BCU_DEAL_H__
#define __BCU_DEAL_H__

#include "interface/queue/queue.h"
#include "interface/can/mycan.h"
#include "interface/epoll/myepoll.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>

#include "interface/bms/bms_analysis.h"

#define BCU_CAN_DEVICE_NAME "can2"

#define BCU_CAN_BITRATE 500000
#define BCU_OTA_ID 0x101
extern queue_t Queue_BCURevData; // 分机消息队列，用于epoll接收数据存入，防止处理不过来所以用队列，内部使用
extern queue_t Queue_BCURevData_FD;

bool bcu_Init(void);
int Drv_bcu_can_send(CAN_MESSAGE *pFrame);
int Drv_bcu_canfd_send(CAN_FD_MESSAGE_BUS *pFrame);
int Drv_can_bind_interface(const char *can_name, int bitrate, int *can_fd_ptr,void (*callback)(void *arg));
int Drv_can_auto_recover(const char *can_name, int bitrate, int *can_fd_ptr, void (*callback)(void *arg));
int get_BCU_CAN_FD(void);
#endif