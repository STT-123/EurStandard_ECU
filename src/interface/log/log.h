#ifndef LOG_H__
#define LOG_H__

#include <stdbool.h>
#include "zlog.h"

/*
    ！！！！！！！！！！！！！！！！！！！！！！！！ 此文件 禁止格式化 ！！！！！！！！！！！！！！！！！！！！！！！！
*/

#define ZLOG_CONF_FILE_PATH "/opt/xcharge/zlog.conf"    // zlog 配置文件地址
#define ZLOG_DATA_FILE_PATH "/mnt/sda/log"          // zlog 日志存放地址

extern zlog_category_t *log_printf;    // 终端
extern zlog_category_t *log_record;    // 通用日志
extern zlog_category_t *log_csv;       // csv文件

// 对外提供的宏定义
#define LOG(fmt, ...)       do { log_output_lock(); if (log_file_output_enabled_locked() && log_record) zlog_info(log_record, fmt, ##__VA_ARGS__); if (log_printf) zlog_info(log_printf, fmt, ##__VA_ARGS__); log_output_unlock(); } while (0)
#define LOG_CSV(fmt, ...)   do { log_output_lock(); if (log_file_output_enabled_locked() && log_csv) zlog_info(log_csv, fmt, ##__VA_ARGS__); if (log_printf) zlog_info(log_printf, fmt, ##__VA_ARGS__); log_output_unlock(); } while (0)

int log_init(void);
bool log_file_output_enabled(void);
bool log_file_output_enabled_locked(void);
void log_output_lock(void);
void log_output_unlock(void);
void log_pause_for_sd_format(void);
int log_resume_after_sd_format(void);
void ascill_to_string(unsigned char *data, unsigned int data_len, char *string_ptr, int string_len);

#endif /* LOG_H__ */
