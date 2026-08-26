#include "log.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>    // 必须包含
#include <sys/stat.h>     // 必须包含，定义了struct stat
#include <pthread.h>
#include <errno.h>
#include <stdio.h>

zlog_category_t *log_printf = NULL;
zlog_category_t *log_record = NULL;
zlog_category_t *log_csv = NULL;

static pthread_mutex_t g_log_output_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool g_log_file_output_enabled = true;
static bool g_log_initialized = false;

void log_output_lock(void)
{
    pthread_mutex_lock(&g_log_output_mutex);
}

void log_output_unlock(void)
{
    pthread_mutex_unlock(&g_log_output_mutex);
}

bool log_file_output_enabled_locked(void)
{
    return g_log_file_output_enabled;
}

bool log_file_output_enabled(void)
{
    bool enabled = true;

    pthread_mutex_lock(&g_log_output_mutex);
    enabled = g_log_file_output_enabled;
    pthread_mutex_unlock(&g_log_output_mutex);

    return enabled;
}

// 0 成功
// -1 配置文件不对
// -2 初始化失败
static int ensure_directory(const char *path, mode_t mode)
{
    struct stat st;

    if (stat(path, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return 0;
        }
        errno = ENOTDIR;
        return -1;
    }

    if (errno != ENOENT) {
        return -1;
    }

    if (mkdir(path, mode) != 0 && errno != EEXIST) {
        return -1;
    }

    return 0;
}

int log_init()
{
    int rc = 0;

    /* 应用日志固定保存在内部存储，不依赖 SD 卡挂载状态。 */
    if (ensure_directory("/userdata", 0755) != 0) {
        printf("create /userdata failed: %s\n", strerror(errno));
        return -1;
    }
    if (ensure_directory("/userdata/xcharge", 0755) != 0) {
        printf("create /userdata/xcharge failed: %s\n", strerror(errno));
        return -1;
    }
    if (ensure_directory(ZLOG_DATA_FILE_PATH, 0755) != 0) {
        printf("create %s failed: %s\n", ZLOG_DATA_FILE_PATH, strerror(errno));
        return -1;
    }

    pthread_mutex_lock(&g_log_output_mutex);

    if (g_log_initialized) {
        g_log_file_output_enabled = true;
        pthread_mutex_unlock(&g_log_output_mutex);
        return 0;
    }

    log_printf = NULL;
    log_record = NULL;
    log_csv = NULL;

    rc = zlog_init(ZLOG_CONF_FILE_PATH);
    if (rc)
    {
        printf("zlog init failed: config=%s, rc=%d\n", ZLOG_CONF_FILE_PATH, rc);
        pthread_mutex_unlock(&g_log_output_mutex);
        return -1;
    }

    log_printf = zlog_get_category("log_printf");
    if (!log_printf)
    {
        printf("get log_printf fail \n");
        zlog_fini();
        log_printf = NULL;
        pthread_mutex_unlock(&g_log_output_mutex);
        return -2;
    }

    log_record = zlog_get_category("log_record");
    if (!log_record)
    {
        printf("get log_record fail \n");
        zlog_fini();
        log_printf = NULL;
        log_record = NULL;
        pthread_mutex_unlock(&g_log_output_mutex);
        return -3;
    }

    log_csv = zlog_get_category("log_csv");
    if (!log_csv)
    {
        printf("get log_csv fail \n");
        zlog_fini();
        log_printf = NULL;
        log_record = NULL;
        log_csv = NULL;
        pthread_mutex_unlock(&g_log_output_mutex);
        return -4;
    }

    g_log_initialized = true;
    g_log_file_output_enabled = true;
    pthread_mutex_unlock(&g_log_output_mutex);
    return 0;
}

/*
 * 十六进制数据转换为字符串并且加空格
 * */
// 将16进制消息，转换为字符串，用于日志存储
// data：原始数据，data_len：原始数据长度
// string_ptr：转换后的字符串，
// string_len：转换后的字符串长度
#include "string.h" // memset
#include <stdio.h>  // snprintf
void ascill_to_string(unsigned char *data, unsigned int data_len, char *string_ptr, int string_len)
{
    unsigned int i;

    // 检查输入参数合法性
    if (!data || !string_ptr || data_len == 0 || string_len <= 0)
    {
        if (string_ptr && string_len > 0)
        {
            string_ptr[0] = '\0'; // 确保输出为空字符串
        }
        return;
    }

    // 计算需要的空间：每个字节转换成 "XX "（3字节），最后去掉末尾空格加'\0'
    int required_len = data_len * 3;
    if (string_len < required_len)
    {
        // 缓冲区不足，只填充能容纳的部分
        data_len = (string_len - 1) / 3;
        if (data_len == 0)
        {
            string_ptr[0] = '\0';
            return;
        }
    }

    char *p = string_ptr;
    for (i = 0; i < data_len; i++)
    {
        // 安全格式化，避免溢出
        int n = snprintf(p, string_len - (p - string_ptr), "%02X ", data[i]);
        if (n < 0 || n >= string_len - (p - string_ptr))
        {
            break; // 写入失败或缓冲区已满
        }
        p += n;
    }

    // 去掉最后一个空格（如果有）
    if (p > string_ptr)
    {
        *(p - 1) = '\0';
    }
    else
    {
        *p = '\0';
    }
}
