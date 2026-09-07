#ifndef SD_LOG_H
#define SD_LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

/* 单文件 10 MiB，当前文件 + 5 份历史文件，约 60 MiB。 */
#define SD_LOG_MAX_SIZE (10U * 1024U * 1024U)
#define SD_LOG_BACKUP_COUNT 5
#define SD_LOG_QUEUE_COUNT 256
#define SD_LOG_LINE_SIZE 4096

int sd_log_start(void);
void sd_log_stop(void);
/* 返回时已完成正在进行的写入，所有 SD 日志文件均已关闭。 */
void sd_log_pause(bool paused);
void sd_log_enqueue(const char *file, long line, const char *format, va_list args);

#endif
