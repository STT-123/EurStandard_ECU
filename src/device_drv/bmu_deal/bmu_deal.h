#ifndef __BMU_DEAL_H__
#define __BMU_DEAL_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>

#include "interface/log/log.h"
#include "interface/bms/bms_analysis.h"
#include "interface/queue/queue.h"
#include "interface/can/mycan.h"
#include "interface/epoll/myepoll.h"

extern queue_t Queue_BMURevData; // 分机消息队列，用于epoll接收数据存入，防止处理不过来所以用队列，内部使用



#define BMU_CAN_DEVICE_NAME "can2"

#define BMU_CAN_BITRATE 500000
#define BMU_OTA_ID 0x102

bool bmu_Init(void);
int Drv_bmu_can_send(CAN_MESSAGE *pFrame);
int Drv_bmu_canfd_send(struct canfd_frame *cansend_data);
#endif