#ifndef __SD_STORE_H__
#define __SD_STORE_H__

#include "stdbool.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

#include <sys/statvfs.h>
#include <dirent.h>
#include <sys/stat.h>
#include <limits.h>
#include <time.h>
#include "interface/bms/bms_simulink/can_fd_message.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"

#define MAX_FILES_IN_FOLDER 80
#define BUFFER_SIZE 100 // 每个环形缓冲区的容量
#define BUFFERED_WRITE_SIZE 1024
#define CHECKSD_TRIGGERING_TIME 60000 * 1000
#define SD_FILE_SIZE (10*1024*1024)
#define SDMAXCAPACITY 90
#define MAX_FRAMES_PER_ID 2
#define STORE_INTERVAL_MS           3000  // 3秒
/*-----*/
#define USB_DEVICE "/dev/mmcblk1p1"
#define USB_MOUNT_POINT "/mnt/sda"
#define UNMOUNT_POINT  "umount /mnt/sda"
typedef struct
{
    double Timestamp; // 相对时间戳
    CAN_FD_MESSAGE msg;
    unsigned char channel;
} CAN_LOG_MESSAGE;

typedef struct
{
    CAN_LOG_MESSAGE buffer[BUFFER_SIZE];
    size_t writeIndex;
    size_t readIndex;
    size_t count;          // 当前缓冲区中的消息数量
    pthread_mutex_t mutex; // 保护缓冲区访问的互斥量
} RingBuffer;

typedef struct
{
    RingBuffer buffers[2];
    int activeBuffer;            // 当前活跃缓冲区索引
    pthread_mutex_t switchMutex; // 用于保护缓冲区切换
} DoubleRingBuffer;

static int  Drv_check_and_update_message(const CAN_FD_MESSAGE *msg);
static void Drv_RTCGetTime(Rtc_Ip_TimedateType *rtcTime);
static int  mount_sdcard_fat32(void);
static int  unmount_sdcard_if_needed(const char *mount_point);
static int  format_sdcard_fat32(const char *device, int *exit_code);
static int judgeTimetoUpdate(struct tm *nowTime);
static int should_store_frame(uint32_t msg_id);
static uint8_t CalculateDLC(uint8_t data_length);
int  SD_Initialize(void);
int  ensure_mount_point(const char *path);
bool sdcard_is_formatting(void);
void Drv_write_to_active_buffer(const CAN_FD_MESSAGE *msg, uint8_t channel);
void Drv_write_buffer_to_file(void);
void checkSDCardCapacity(void);
void sd_storeInit(void);
static int find_id_index(uint32_t id);
int mkdir_log(const char *base_path) ;
bool clean_directory(const char *mount_point);
static int is_valid_date_name(const char *name);
static void Func_DeleteOldestFolder(void);
void checkRootCapacity(void);
#endif
