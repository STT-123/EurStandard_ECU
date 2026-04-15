#ifndef LOG_H__
#define LOG_H__

#include <stdbool.h>
#include "zlog.h"

/*
    ！！！！！！！！！！！！！！！！！！！！！！！！ 此文件 禁止格式化 ！！！！！！！！！！！！！！！！！！！！！！！！
*/

#define ZLOG_CONF_FILE_PATH "/opt/xcharge/zlog.conf"    // zlog 配置文件地址
#define ZLOG_DATA_FILE_PATH "/mnt/sda/log"          // zlog 日志存放地址

zlog_category_t *log_printf;    // 终端
zlog_category_t *log_record;    // 通用日志
zlog_category_t *log_csv;       // csv文件

// 对外提供的宏定义
#define LOG(fmt, ...)       do { if (log_file_output_enabled() && log_record) zlog_info(log_record, fmt, ##__VA_ARGS__); if (log_printf) zlog_info(log_printf, fmt, ##__VA_ARGS__); } while (0)
#define LOG_CSV(fmt, ...)   do { if (log_file_output_enabled() && log_csv) zlog_info(log_csv, fmt, ##__VA_ARGS__); if (log_printf) zlog_info(log_printf, fmt, ##__VA_ARGS__); } while (0)

int log_init(void);
bool log_file_output_enabled(void);
void log_pause_for_sd_format(void);
int log_resume_after_sd_format(void);
void ascill_to_string(unsigned char *data, unsigned int data_len, char *string_ptr, int string_len);

#endif /* LOG_H__ */
