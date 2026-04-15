#include "log.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>    // 必须包含
#include <sys/stat.h>     // 必须包含，定义了struct stat
#include <pthread.h>

static pthread_mutex_t g_log_output_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool g_log_file_output_enabled = true;
static bool g_log_initialized = false;
static bool g_log_paused = false;

bool log_file_output_enabled(void)
{
    bool enabled = true;

    pthread_mutex_lock(&g_log_output_mutex);
    enabled = g_log_file_output_enabled;
    pthread_mutex_unlock(&g_log_output_mutex);

    return enabled;
}

void log_pause_for_sd_format(void)
{
    bool should_fini = false;

    pthread_mutex_lock(&g_log_output_mutex);
    g_log_file_output_enabled = false;
    if (!g_log_paused) {
        g_log_paused = true;
        should_fini = g_log_initialized;
        g_log_initialized = false;
    }
    pthread_mutex_unlock(&g_log_output_mutex);

    if (should_fini) {
        zlog_fini();
    }

    log_printf = NULL;
    log_record = NULL;
    log_csv = NULL;
}

int log_resume_after_sd_format(void)
{
    pthread_mutex_lock(&g_log_output_mutex);
    if (!g_log_paused) {
        g_log_file_output_enabled = true;
        pthread_mutex_unlock(&g_log_output_mutex);
        return 0;
    }
    g_log_paused = false;
    pthread_mutex_unlock(&g_log_output_mutex);

    return log_init();
}
// 0 成功
// -1 配置文件不对
// -2 初始化失败
int log_init()
{
    int rc = 0;

    if (F_OK != access(ZLOG_DATA_FILE_PATH, 0))
    {
        system("mkdir " ZLOG_DATA_FILE_PATH); // 创建文件夹
    }

    rc = zlog_init(ZLOG_CONF_FILE_PATH);
    if (rc)
    {
        printf("zlog init failed \n");
        return -1;
    }

    log_printf = zlog_get_category("log_printf");
    if (!log_printf)
    {
        printf("get log_printf fail \n");
        zlog_fini();
        return -2;
    }

    log_record = zlog_get_category("log_record");
    if (!log_record)
    {
        printf("get log_record fail \n");
        zlog_fini();
        return -3;
    }

    log_csv = zlog_get_category("log_csv");
    if (!log_csv)
    {
        printf("get log_csv fail \n");
        zlog_fini();
        return -4;
    }

    pthread_mutex_lock(&g_log_output_mutex);
    g_log_initialized = true;
    g_log_paused = false;
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
