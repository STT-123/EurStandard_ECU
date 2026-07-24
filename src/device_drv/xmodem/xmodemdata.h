#ifndef __C_OTA_DATAMONITOR_H__
#define __C_OTA_DATAMONITOR_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <ctype.h>
#include <limits.h>
#include <sys/stat.h>

#define SOH 0x01  //xmodem-128协议固定，XModem-128数据帧的数据头
#define STX 0x02  //XModem-128数据帧的数据头
#define EOT 0x04  //文件发起方发送结束，终止传输业务
#define ACK 0x06  //认可响应
#define NAK 0x15  //不认可响应
#define CAN 0x18  //文件接收方终止传输
#define CTRLZ 0xFF  //填充数据


void *lwip_data_TASK(void *param);

extern volatile unsigned long curmsgtimer;

// unsigned int OsIf_GetMilliseconds(void) ;
unsigned short crcGet(unsigned char *ptr, int len);
signed char GetOTAFILEInfo(const unsigned char *databuf, size_t databuf_len,
                           char *name, size_t name_size,
                           int *filesize, int *xmodempacknum);
signed char SaveOtaFile(char *name, unsigned char *buf, int totalpacknum, int curpackno, int datanum);
void delete_files_with_prefix(const char *path, const char *prefix);
#endif
