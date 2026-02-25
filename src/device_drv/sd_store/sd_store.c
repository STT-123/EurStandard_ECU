#define _GNU_SOURCE
#include "sd_store.h"
#include "device_drv/abncheck/abncheck.h"
#include "interface/log/log.h"
#include "interface/bms/bms_simulink/CANFDRcvFcn_BCU.h"
#include "interface/bms/bms_simulink/CANRcvFcn_BMU.h"
#include <time.h>
#include <ftw.h>


/*===*/
static struct timespec last_store_time = {0};
// 特定ID触发存储
static uint8_t trigger_store_flag = 0;
static struct timespec trigger_store_time = {0};
/*==*/
DoubleRingBuffer canDoubleRingBuffer;
static bool newFileNeeded = true;
Rtc_Ip_TimedateType initialTime = {0};
Rtc_Ip_TimedateType currentTime = {0};
struct timespec start_tick = {0};
static uint32_t CAN_IDs[] = {
    // 0x1801E410,
    //BCU电池ID 
    0x180110E4,0x180210E4,0x180310E4,0x180410E4,0x1A0110E4,0x1B0110E4,
    //空调ID 
    0x18FF0000,0x18FF0001,0x18FF0002,0x18FF0003,0x18FF0004,
    0x18FF0005,0x18FF0006,0x18FF0007,0x18FF0008,
    0x18FF45F0,0x235,
    0x18FFC13A,0x18FFC13B,0x18FFC13C,0x18FFC13D 
};
#define CAN_ID_HISTORY_SIZE (sizeof(CAN_IDs) / sizeof(CAN_IDs[0]))
 CAN_MESSAGE can_msg_cache[CAN_ID_HISTORY_SIZE] = {0};
 static int frame_count_per_id[CAN_ID_HISTORY_SIZE] = {0};

static struct timeval first_tv = {0, 0};
static int first_time_captured = 0;
/*检查U盘是否可用   0正常 1不正常*/
static char* find_sd_card_simple(void) {
    // 按优先级尝试的设备列表
    const char *devices[] = {
        "/dev/mmcblk1p1",  // 第一优先级：有分区的SD卡
        "/dev/mmcblk1",    // 第二优先级：整个SD卡
        "/dev/mmcblk2p1",  // 第三优先级：其他可能的设备
        "/dev/mmcblk2",
        NULL
    };
    
    for (int i = 0; devices[i] != NULL; i++) {
        // 检查文件是否存在
        if (access(devices[i], F_OK) == 0) {
            LOG("[SD] 找到设备: %s\n", devices[i]);
            return strdup(devices[i]);  // 返回设备路径
        }
    }
    
    // LOG("[SD] 未找到SD卡设备\n");
    return NULL;
}
// 检查设备是否已经挂载
// 最简单实用的SD卡挂载函数
int mount_sdcard_fat32(void) {
    char *device = NULL;
    int ret;
    
    // 先检查/proc/mounts
    FILE *fp = fopen("/proc/mounts", "r");
    if (fp) {
        char line[256];
        while (fgets(line, sizeof(line), fp)) {
            if (strstr(line, USB_MOUNT_POINT)) {
                fclose(fp);
                return 0;  // 已经挂载了，直接返回成功
            }
        }
        fclose(fp);
    }

    // 1. 查找SD卡设备
    device = find_sd_card_simple();
    if (!device) {
        return -1;
    }
    
    // 2. 创建挂载点目录
    // mkdir - 创建目录（最简单的API）
    if (mkdir(USB_MOUNT_POINT, 0755) != 0) {
        if (errno != EEXIST) {  // 如果目录已存在，不算是错误
            LOG("[SD_Card] create /mnt/sda failed: %s\n", strerror(errno));
            free(device);
           return -1;
        }
    }
    
    // 3. 尝试挂载
    // mount - 挂载文件系统（核心API）
    // 先尝试带UTF8支持
    ret = mount(device, USB_MOUNT_POINT, "vfat", 0, "iocharset=utf8");
    
    if (ret != 0) {
        // 如果失败，尝试不带参数
        LOG("[SD_Card]  Mounting with parameters failed. Trying simple mounting...\n");
        ret = mount(device, USB_MOUNT_POINT, "vfat", 0, NULL);
    }
    
    if (ret != 0) {
        LOG("[SD] Mount failed: %s\n", strerror(errno));
        free(device);
        return -1;
    }
    
    LOG("[SD_Card] Mount suceefully: %s -> %s\n", device, USB_MOUNT_POINT);
    free(device);
    return 0;
} 

/**
 * 获取本地时间
*/
static void Drv_RTCGetTime(Rtc_Ip_TimedateType *rtcTime)
{
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);

    rtcTime->year = tm_info->tm_year + 1900;
    rtcTime->month = tm_info->tm_mon + 1;
    rtcTime->day = tm_info->tm_mday;
    rtcTime->hour = tm_info->tm_hour;
    rtcTime->minutes = tm_info->tm_min;
    rtcTime->seconds = tm_info->tm_sec;
}
/**
 * 获取当前时间
 * 
*/

static int GetNowTime(struct tm *nowTime)
{
    static struct tm last_bcu_time = {0}; // 记录上一次有效的BCU时间
    static time_t last_bcu_update = 0;    // 上次BCU时间更新的时间戳（用于超时判断）
    const int BCU_TIMEOUT_SEC = 300;      // 5分钟超时，可调整

    struct tm timeinfo = {0};
    time_t current_time = time(NULL);

    if (get_BCU_TimeYearValue() != 0) 
    {
        // 构造当前BCU时间
        struct tm bcu_tm = {0};
        bcu_tm.tm_year = get_BCU_TimeYearValue() + 100;
        bcu_tm.tm_mon  = get_BCU_TimeMonthValue() - 1;
        bcu_tm.tm_mday = get_BCU_TimeDayValue();
        bcu_tm.tm_hour = get_BCU_TimeHourValue();
        bcu_tm.tm_min  = get_BCU_TimeMinuteValue();
        bcu_tm.tm_sec  = get_BCU_TimeSencondValue(); // 注意拼写 typo? 应为 Second
        bcu_tm.tm_isdst = -1;

        time_t bcu_time_t = mktime(&bcu_tm);
        if (bcu_time_t == (time_t)-1) {
            LOG("[SD Card] WARNING: mktime failed for BCU time\n");
            goto use_local_time;
        }

        // 检查BCU时间是否与上次相同（防陈旧）
        if (memcmp(&bcu_tm, &last_bcu_time, sizeof(struct tm)) == 0) {
            // 时间没变，可能是旧数据
            if (difftime(current_time, last_bcu_update) > BCU_TIMEOUT_SEC) {
                LOG("[SD Card] BCU time unchanged for %d sec, treat as stale", BCU_TIMEOUT_SEC);
                goto use_local_time;
            } else {
                // 时间没变，但在有效期内，继续使用（但不更新系统时间）
                LOG("[SD Card] BCU time unchanged, skip update");
                *nowTime = bcu_tm;
                return 0;
            }
        }

        // 更新记录
        last_bcu_time = bcu_tm;
        last_bcu_update = current_time;

        // 计算时间差
        time_t local_now = time(NULL);
        double time_diff = difftime(bcu_time_t, local_now);
        double abs_diff = fabs(time_diff);
        
        // 只打印超过阈值的情况
        if (abs_diff > 10) {
            LOG("[SD Card] Time difference %.1f > %d seconds, updating from BCU\r\n", 
                abs_diff, 10);
            set_system_time_from_bcu();
        }

        *nowTime = bcu_tm;
        return 0;
    }

    use_local_time:
    {
        // 使用本地时间
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        timeinfo = *tm_info;
        *nowTime = timeinfo;
        LOG("[SD Card] Using local time: %d-%02d-%02d %02d:%02d:%02d",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        return 0;
    }
}
/**
 * 
 * 
 * 使用一个时间创建一个文件路径（文件夹+文件名）
*/ 
static int CreateAscFilePathWithTime(struct tm timeInfo, char *filePath)
{
    // 创建文件夹路径
    char folderPath[256];
    snprintf(folderPath, sizeof(folderPath), "%s/%04d%02d%02d",
             USB_MOUNT_POINT,         // 挂载点
             timeInfo.tm_year + 1900, // 年
             timeInfo.tm_mon + 1,     // 月
             timeInfo.tm_mday);       // 日

    // 创建目录（如果不存在）
    struct stat st = {0};
    if (stat(folderPath, &st) == -1)
    {
        if (mkdir(folderPath, 0777) == -1)
        {
            perror("mkdir failed");
            LOG("[SD Card] [SD Card] mkdir failed");
            return 1;
        }
    }

    // 生成完整文件路径（年月日时分秒）
    sprintf(filePath, "%s/%04d%02d%02d%02d%02d%02d.asc",
            folderPath,              // 文件夹路径
            timeInfo.tm_year + 1900, // 年
            timeInfo.tm_mon + 1,     // 月
            timeInfo.tm_mday,        // 日
            timeInfo.tm_hour,        // 时
            timeInfo.tm_min,         // 分
            timeInfo.tm_sec);        // 秒

    LOG("[SD Card] new asc file path is = %04d%02d%02d%02d%02d%02d. \n",
        timeInfo.tm_year + 1900,
        timeInfo.tm_mon + 1,
        timeInfo.tm_mday,
        timeInfo.tm_hour,
        timeInfo.tm_min,
        timeInfo.tm_sec);

    return 0;
}

static int OpenNowWriteAscFile(const char *filePath, FILE **file) {
    static int failed_count = 0;
    
    if (!filePath || !file) {
        LOG("[SD Card] ERROR: Invalid parameters to OpenNowWriteAscFile\n");
        return 1;
    }
    
    //printf(">>> Attempting to open file: %s\n", filePath);
    
    *file = fopen(filePath, "ab");
    
    if (!*file) {
        int err = errno;
        LOG("[SD Card] ERROR: fopen failed for %s, errno=%d (%s)\n", filePath, err, strerror(err));
        failed_count++;
        
        if (failed_count >= 5) {
            LOG("[SD Card] OpenNowWriteAscFile %s failed %d times, errno=%d\n", filePath, failed_count, err);
           set_emcu_fault(SD_FAULT, SET_ERROR);
        }
        return 1;
    }

    if (failed_count >= 5) {
        LOG("[SD Card] OpenNowWriteAscFile %s recovered after %d failures\n", filePath, failed_count);
        set_emcu_fault(SD_FAULT, SET_RECOVER);
    }
    failed_count = 0;
    // LOG("[SD Card] SUCCESS: File opened successfully: %p\n", (void*)*file);
    return 0;
}

/**
 * 给ASC文件写一固定的时间头
 * 
 * */ 
static int AscFileWriteTimeHeader(FILE *file, struct tm *timeinfo)
{
    LOG("[SD Card] === AscFileWriteTimeHeader START ===\n");
    
    if (file == NULL)
    {
        LOG("[SD Card] CRITICAL ERROR: File pointer is NULL in AscFileWriteTimeHeader\n");
        return -1;
    }
    
    if (timeinfo == NULL)
    {
        LOG("[SD Card] CRITICAL ERROR: timeinfo is NULL in AscFileWriteTimeHeader\n");
        return -1;
    }
    
    // 验证时间字段的合理性
    LOG("[SD Card] Time info: wday=%d, mon=%d, mday=%d, hour=%d, min=%d, sec=%d, year=%d\n",
           timeinfo->tm_wday, timeinfo->tm_mon, timeinfo->tm_mday,
           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
           timeinfo->tm_year);
    
    char header[512] = {0}; // 增大缓冲区确保安全
    const char *weekDays[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    // 检查数组索引边界
    if (timeinfo->tm_wday < 0 || timeinfo->tm_wday > 6) {
        LOG("[SD Card] ERROR: Invalid tm_wday: %d\n", timeinfo->tm_wday);
        return -1;
    }
    if (timeinfo->tm_mon < 0 || timeinfo->tm_mon > 11) {
        LOG("[SD Card] ERROR: Invalid tm_mon: %d\n", timeinfo->tm_mon);
        return -1;
    }

    // 使用安全的 snprintf 一次性构建整个头部
    int total_len = snprintf(header, sizeof(header),
                            "date %s %s %02d %02d:%02d:%02d %s %04d\r\n"
                            "base hex timestamps absolute\r\n"
                            "// version 7.0.0\r\n",
                            weekDays[timeinfo->tm_wday],
                            months[timeinfo->tm_mon],
                            timeinfo->tm_mday,
                            (timeinfo->tm_hour > 12) ? (timeinfo->tm_hour - 12) : 
                             (timeinfo->tm_hour == 0) ? 12 : timeinfo->tm_hour, // 处理 0 点的情况
                            timeinfo->tm_min, 
                            timeinfo->tm_sec,
                            (timeinfo->tm_hour >= 12) ? "PM" : "AM",
                            timeinfo->tm_year + 1900);
    
    LOG("[SD Card] Header length: %d, buffer size: %zu\n", total_len, sizeof(header));
    
    if (total_len < 0) {
        LOG("[SD Card] ERROR: snprintf failed\n");
        return -1;
    }
    
    if ((size_t)total_len >= sizeof(header)) {
        LOG("[SD Card] ERROR: Header too long: %d >= %zu\n", total_len, sizeof(header));
        return -1;
    }
    
    LOG("[SD Card] Header content:\n%s", header);
    
    // 写入文件
    size_t written = fwrite(header, 1, total_len, file);
    // LOG("[SD Card] Bytes written: %zu, expected: %d\n", written, total_len);
    
    if (written != (size_t)total_len)
    {
        LOG("[SD Card] WARNING: Header not fully written to file: %zu != %d\n", written, total_len);
        return -1;
    }
    
    fflush(file); // 确保数据写入磁盘
    LOG("[SD Card] === AscFileWriteTimeHeader COMPLETED SUCCESSFULLY ===\n");
    return 0;
}


// 查找指定CAN ID的历史消息，并与当前消息对比
static int Drv_check_and_update_message(const CAN_FD_MESSAGE *msg)
{

    if (!msg) {
        LOG("[SD Card] ERROR: msg is NULL in Drv_check_and_update_message\n");
        return 0;
    }
    static unsigned int old_BMSWorkMode_value = 0;
    for (int i = 0; i < CAN_ID_HISTORY_SIZE; i++)
    {
        if (can_msg_cache[i].ID == msg->ID)
        {
            if(msg->ID == 0x180110E4)
            {
                /*
                    这段代码的意义：正常存储规则，3S 每个ID的数据存储最多2帧，当检测到get_BCU_SystemWorkModeValue变化时
                    重新开始算3S 计时间器，并触发存储
                */
                if ( get_BCU_SystemWorkModeValue() != old_BMSWorkMode_value)
                {
                    //只要CANFD.DBC 的工作模式位置有变化，这个Dta[]就要变
                    LOG("[CAN Trigger] ID=0x180110E4 Mode changed: 0x%02X -> 0x%02X, triggering storage\n", old_BMSWorkMode_value, msg->Data[0]);                 
                    memset(frame_count_per_id, 0, sizeof(frame_count_per_id));
                    clock_gettime(CLOCK_MONOTONIC, &trigger_store_time);
                    clock_gettime(CLOCK_MONOTONIC, &last_store_time);  // ← 关键：同时重置常规存储时间
                    LOG("[CAN Trigger] Both trigger and normal storage timers reset\n");
                    trigger_store_flag = 1;// 触发存储，重置计数器
                    old_BMSWorkMode_value = get_BCU_SystemWorkModeValue();
                    LOG("WorkMode Changed Time:%d:%d:%d\r\n",get_BCU_TimeHourValue(),get_BCU_TimeMinuteValue(),get_BCU_TimeSencondValue());
                }  
            }

            return 1;
        }
    }

    return 0; // 如果未找到该 CAN ID
}



// 判断当前帧是否应该存储
static int should_store_frame(uint32_t msg_id)
{
    int idx = find_id_index(msg_id);
    if (idx < 0) return 0;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    
    // 检查触发存储条件（优先级更高）
    if (trigger_store_flag) {
        long elapsed_ms = (now.tv_sec - trigger_store_time.tv_sec) * 1000 +
                         (now.tv_nsec - trigger_store_time.tv_nsec) / 1000000;
        
        if (elapsed_ms >= STORE_INTERVAL_MS) {
            trigger_store_flag = 0;
            return 0;
        }
        return (frame_count_per_id[idx] < MAX_FRAMES_PER_ID);// 触发期间也遵守 "每 ID 最多 2 帧"
    }
    
    // 常规3秒存储逻辑
    // 如果是第一次，初始化时间
    if (last_store_time.tv_sec == 0) 
    {
        clock_gettime(CLOCK_MONOTONIC, &last_store_time);
        memset(frame_count_per_id, 0, sizeof(frame_count_per_id));
        // LOG("[CAN Normal] Starting first 3s storage window\n");
        return 1;
    }
    
    // 计算时间差
    long elapsed_ms = (now.tv_sec - last_store_time.tv_sec) * 1000 +
                     (now.tv_nsec - last_store_time.tv_nsec) / 1000000;
    
    // 如果超过3秒，开始新的存储周期
    if (elapsed_ms >= STORE_INTERVAL_MS) {
        // LOG("[CAN Normal] 3s elapsed, starting new window. Previous: %d frames\n", frame_counter);
        memset(frame_count_per_id, 0, sizeof(frame_count_per_id));
        clock_gettime(CLOCK_MONOTONIC, &last_store_time);
        return 1;
    }
    
    return (frame_count_per_id[idx] < MAX_FRAMES_PER_ID);
    return 0;
}

static int judgeTimetoUpdate(struct tm *nowTime)
{
    int ret = 0;
    static int last_year = -1;
    static int last_month = -1;
    static int last_day = -1;
    
    int current_year  = nowTime->tm_year + 1900; // tm_year 是从 1900 开始的偏移
    int current_month = nowTime->tm_mon + 1;     // tm_mon 范围是 0~11
    int current_day   = nowTime->tm_mday;        // tm_mday 范围是 1~31

    // LOG("[SD Card] BCU Current time: %d-%d-%d, nowTime: %d-%d-%d, Last time: %d-%d-%d\n", 
    //     get_BCU_TimeYearValue(), get_BCU_TimeMonthValue(), get_BCU_TimeDayValue(),current_year,current_month,current_day,
    //     last_year, last_month, last_day);
        
    // 如果是首次调用，仅初始化，不触发变更
    if (last_year == -1) {
        last_year  = current_year;
        last_month = current_month;
        last_day   = current_day;
        return 0;
    }

    // 检查年、月、日是否发生变化
    if (current_year != last_year || 
        current_month != last_month || 
        current_day != last_day) {

        LOG("[SD Card] TIME CHANGE DETECTED: %d-%02d-%02d -> %d-%02d-%02d\n",
            last_year, last_month, last_day,
            current_year, current_month, current_day);

        // 更新记录
        last_year  = current_year;
        last_month = current_month;
        last_day   = current_day;
        return 1; // 日期已变更
    }
    
    return ret;
}

static char *Drv_my_strdup(const char *str)
{
    if (!str)
        return NULL;
    char *dup = malloc(strlen(str) + 1);
    if (dup)
        strcpy(dup, str);
    return dup;
}

// 递归删除目录及内容
static int Drv_remove_directory(const char *path)
{
    DIR *dir = opendir(path);
    struct dirent *entry;
    char subPath[512];

    if (!dir)
        return -1;

    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        snprintf(subPath, sizeof(subPath), "%s/%s", path, entry->d_name);

        struct stat statbuf;
        if (stat(subPath, &statbuf) == 0)
        {
            if (S_ISDIR(statbuf.st_mode))
            {
                Drv_remove_directory(subPath); // 递归删除
            }
            else
            {
                unlink(subPath); // 删除文件
            }
        }
    }

    closedir(dir);
    return rmdir(path); // 删除目录
}

static int CompareFolderNames(const void *a, const void *b)
{
    return strcmp(*(const char **)a, *(const char **)b);
}


static int is_valid_date_name(const char *name) {
    if (strlen(name) != 8) return 0;
    for (int i = 0; i < 8; i++) {
        if (name[i] < '0' || name[i] > '9') return 0;
    }
    // 可进一步校验年月日有效性（如月份 01-12，日期合理等）
    return 1;
}

static void Func_DeleteOldestFolder(void)
{
    DIR *dir = opendir(USB_MOUNT_POINT);
    if (!dir) { perror("opendir"); return; }

    char oldest[256] = ""; // 初始为空
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 ||
            strcmp(entry->d_name, "..") == 0 ||
            strcmp(entry->d_name, "19700101") == 0)
            continue;

        char fullPath[512];
        snprintf(fullPath, sizeof(fullPath), "%s/%s", USB_MOUNT_POINT, entry->d_name);

        struct stat st;
        if (stat(fullPath, &st) == 0 && S_ISDIR(st.st_mode)) {
            if (!is_valid_date_name(entry->d_name)) {
                continue; // 跳过非日期文件夹，如 "log"
            }
            if (oldest[0] == '\0' || strcmp(entry->d_name, oldest) < 0) {
                strncpy(oldest, entry->d_name, sizeof(oldest) - 1);
            }
        }
    }
    closedir(dir);

    if (oldest[0] != '\0') {
        char path[512];
        snprintf(path, sizeof(path), "%s/%s", USB_MOUNT_POINT, oldest);
        LOG("[SD Card] Deleting oldest folder: %s\n", path);
        Drv_remove_directory(path);
    }
}

#if 0
// 删除最旧的文件夹（假设文件夹名为日期字符串）
//有BUG，当文件夹太多的时候folders[100]会越界
void Func_DeleteOldestFolder(void)
{
    DIR *dir = opendir(USB_MOUNT_POINT);
    struct dirent *entry;
    char *folders[100];
    int folderCount = 0;

    if (!dir)
    {
        perror("opendir");
        return;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") != 0 &&
            strcmp(entry->d_name, "..") != 0 &&
            strcmp(entry->d_name, "19700101") != 0)
        {

            // 用 stat 判断是不是目录
            char fullPath[512];
            snprintf(fullPath, sizeof(fullPath), "%s/%s", USB_MOUNT_POINT, entry->d_name);

            struct stat st;
            if (stat(fullPath, &st) == 0 && S_ISDIR(st.st_mode))
            {
                folders[folderCount] = Drv_my_strdup(entry->d_name);
                if (!folders[folderCount])
                    break;
                folderCount++;
            }
        }
    }

    printf("[SD Card] Folder Count: %d\n", folderCount);
    closedir(dir);

    if (folderCount > 0)
    {
        qsort(folders, folderCount, sizeof(char *), CompareFolderNames);
        for (int i = 0; i < folderCount; i++)
        {
            LOG("[SD Card] Folder Sorder: %s\r", folders[i]);
        }
        char path[512];
        snprintf(path, sizeof(path), "%s/%s", USB_MOUNT_POINT, folders[0]);

        LOG("[SD Card] Deleting oldest folder: %s\n", path);
        Drv_remove_directory(path);

        for (int i = 0; i < folderCount; i++)
        {
            free(folders[i]);
        }
    }
}
#endif
static void Drv_init_can_id_history(void)
{
    int i = 0;
    // printf("CAN_ID_HISTORY_SIZE = %d\r\n",CAN_ID_HISTORY_SIZE);
    for (i = 0; i < CAN_ID_HISTORY_SIZE; i++)
    {
        can_msg_cache[i].ID = CAN_IDs[i];
        can_msg_cache[i].Length = 64; //
    }
}
// 获取从当天00:00:00开始的秒.毫秒
// 获取UTC时间戳（秒.毫秒）
// 获取Unix时间戳（从1970-01-01开始的秒数）
// 方法1：生成ZXDOC兼容的时间戳（从1900年开始）
double get_relative_timestamp_seconds(void) {
    struct timeval current_tv;
    
    gettimeofday(&current_tv, NULL);
    
    if (!first_time_captured) {
        // 第一次调用，设置基准时间
        first_tv = current_tv;
        first_time_captured = 1;
        return 0.0;
    }
    
    // 计算相对于基准时间的偏移量（秒）
    double diff_sec = (double)(current_tv.tv_sec - first_tv.tv_sec);
    double diff_usec = (double)(current_tv.tv_usec - first_tv.tv_usec);
    
    return diff_sec + diff_usec / 1000000.0;
}

static void Drv_init_double_ring_buffer(DoubleRingBuffer *drb)
{
    for (int i = 0; i < 2; ++i)
    {
        drb->buffers[i].writeIndex = 0;
        drb->buffers[i].readIndex = 0;
        drb->buffers[i].count = 0;
        pthread_mutex_init(&drb->buffers[i].mutex, NULL);
    }
    drb->activeBuffer = 0;
    pthread_mutex_init(&drb->switchMutex, NULL);
}

// 新建文件时清空双缓冲，避免把上一个文件的尾巴写进新文件
static void Drv_reset_timestamp_and_clear_buffers(DoubleRingBuffer *drb)
{
    if (!drb) return;

    pthread_mutex_lock(&drb->switchMutex);
    pthread_mutex_lock(&drb->buffers[0].mutex);
    pthread_mutex_lock(&drb->buffers[1].mutex);

    for (int i = 0; i < 2; ++i) {
        drb->buffers[i].writeIndex = 0;
        drb->buffers[i].readIndex = 0;
        drb->buffers[i].count = 0;
    }

    // 让下一条CAN帧作为新的时间戳基准（从0开始）
    memset(&first_tv, 0, sizeof(first_tv));
    first_time_captured = 0;

    pthread_mutex_unlock(&drb->buffers[1].mutex);
    pthread_mutex_unlock(&drb->buffers[0].mutex);
    pthread_mutex_unlock(&drb->switchMutex);
}
/*=================================外部调用函数========================================*/

void sd_storeInit(void)
{ 
    Drv_init_double_ring_buffer(&canDoubleRingBuffer); // 初始化往sd卡写数据用的双环形缓冲区
    Drv_init_can_id_history();                         // 初始化往SD卡写的can消息的缓存区
}

/**
 * 确认挂载点
*/
int ensure_mount_point(const char *path)
{
    struct stat st;
    if (stat(path, &st) == -1)
    {
        if (mkdir(path, 0777) == -1)
        {
            perror("Failed to create directory\n");
            LOG("[SD Card] Please check the mount point path\n");
            return -1;
        }
        else
        {
            LOG("[SD Card] Succeed to create directory\n");
        }
    }
    else
    {
        LOG("[SD Card] The mount point already exists\n");
    }
    return 0;
}

int mkdir_log(const char *base_path) {
    // 参数检查
    if (base_path == NULL || strlen(base_path) == 0) {
        LOG("[SD Card] Error: base_path is NULL or empty\n");
        return -1;
    }
    
    // 检查路径长度是否安全
    if (strlen(base_path) > 200) {
        LOG("[SD Card] Error: base_path too long\n");
        return -2;
    }
    
    char log_dir[256];
    char file1_path[256];
    char file2_path[256];
    int ret = 0;
    int files_created = 0;
    
    // 构建log目录路径
    int len = snprintf(log_dir, sizeof(log_dir), "%s/log", base_path);
    if (len >= sizeof(log_dir)) {
        LOG("[SD Card] Error: log_dir path too long\n");
        return -3;
    }
    
    // 创建log目录
    if (ensure_mount_point(log_dir) != 0) {
        LOG("[SD Card] Failed to create directory %s\n", log_dir);
        return -4;
    }
    
    LOG("[SD Card] Directory %s created/ensured successfully\n", log_dir);
    
    // 创建第一个日志文件
    len = snprintf(file1_path, sizeof(file1_path), "%s/bat_ecu_exe.log", log_dir);
    if (len < sizeof(file1_path)) {
        FILE *fp1 = fopen(file1_path, "w");
        if (fp1 != NULL) {
            fclose(fp1);
            LOG("[SD Card] Created %s\n", file1_path);
            files_created++;
        } else {
            LOG("[SD Card] Failed to create %s: %s\n", file1_path, strerror(errno));
            ret = -5;
        }
    }
    
    // 创建第二个日志文件
    len = snprintf(file2_path, sizeof(file2_path), "%s/auto_install.log", log_dir);
    if (len < sizeof(file2_path)) {
        FILE *fp2 = fopen(file2_path, "w");
        if (fp2 != NULL) {
            fclose(fp2);
            LOG("[SD Card] Created %s\n", file2_path);
            files_created++;
        } else {
            LOG("[SD Card] Failed to create %s: %s\n", file2_path, strerror(errno));
            ret = -6;
        }
    }
    
    if (files_created == 2) {
        LOG("[SD Card] All log files created successfully\n");
    } else if (files_created == 1) {
        LOG("[SD Card] Warning: Only 1 of 2 log files created\n");
    } else {
        LOG("[SD Card] Error: No log files created\n");
    }
    
    return (files_created == 2) ? 0 : ret;
}

/**
 * 初始化缓存
*/


/*===================================================================================*/
void Drv_write_to_active_buffer(const CAN_FD_MESSAGE *msg, uint8_t channel)
{
    DoubleRingBuffer *drb = &canDoubleRingBuffer;
    uint8_t ret = 0;
    Log_Bcu_Data(msg);
    if (((msg->ID == 0x1cb0e410) && (msg->Data[0] == 0xC9)) ||
        (msg->ID == 0x1cb010e4) || (msg->ID == 0x1823E410) || (msg->ID == 0))
    {
        return;
    }

    ret = Drv_check_and_update_message(msg); // 如果是异常ID（不在缓存中），直接返回，不存储
    if (ret == 0) {
        //LOG("[CAN Filter] Abnormal ID=0x%08lX filtered out\n", (unsigned long)msg->ID);
        return;
    }

    // 检查是否应该存储（包括常规3秒和触发存储）
    if (!should_store_frame(msg->ID)) {
        return;
    }
    // if(msg->ID == 0x1801e410){
    //     printf("should_store_frame  0x1801e410\r\n");
    // }
    pthread_mutex_lock(&drb->switchMutex);
    RingBuffer *activeBuffer = &drb->buffers[drb->activeBuffer];
    pthread_mutex_lock(&activeBuffer->mutex);


    CAN_LOG_MESSAGE *logMsg = &activeBuffer->buffer[activeBuffer->writeIndex];

    logMsg->Timestamp = get_relative_timestamp_seconds(); // 如57093.038

    memcpy(&logMsg->msg, msg, sizeof(CAN_FD_MESSAGE));
    logMsg->channel = channel;

    activeBuffer->writeIndex = (activeBuffer->writeIndex + 1) % BUFFER_SIZE;

    if (activeBuffer->count == BUFFER_SIZE)
    {
        activeBuffer->readIndex = (activeBuffer->readIndex + 1) % BUFFER_SIZE;
    }
    else
    {
        activeBuffer->count++;
    }

    pthread_mutex_unlock(&activeBuffer->mutex);
    pthread_mutex_unlock(&drb->switchMutex);

    //更新计数器
    // ✅ 关键：更新该 ID 的计数器
    int idx = find_id_index(msg->ID);
    if (idx >= 0) {
        frame_count_per_id[idx]++;
    }

}

// 将缓冲区数据写到sd卡
void Drv_write_buffer_to_file(void)
{
    static char filePath[512] = {0}; // 当前使用的文件路径
    int ret = 0;
    // 根据当前时间创建一个文件路径
    struct tm nowTimeInfo = {0};
    DoubleRingBuffer *drb = &canDoubleRingBuffer;

    GetNowTime(&nowTimeInfo);// 获取当前时间

    if (newFileNeeded) {
        // 丢弃旧文件尾巴，重置时间戳基准
        Drv_reset_timestamp_and_clear_buffers(drb);
    }

    pthread_mutex_lock(&drb->switchMutex);// 交换当前使用的缓冲区
    drb->activeBuffer = 1 - drb->activeBuffer;
    pthread_mutex_unlock(&drb->switchMutex);

    // 获取需要写入的缓冲区
    int inactiveBufferIndex = 1 - drb->activeBuffer;
    RingBuffer *inactiveBuffer = &drb->buffers[inactiveBufferIndex];

    // 获取文件互斥锁
    ret = pthread_mutex_lock(&inactiveBuffer->mutex);
    if (ret != 0)
    {
        LOG("[SD Card] write_buffer_to_file end return. \n");
        return;
    }
    
    if (mount_sdcard_fat32() != 0)// 先检查存储器状态 不存在 标记错误 直接退出
    {
        LOG("[SD Card] SD_FAULT\r\n");
        set_emcu_fault(SD_FAULT, SET_ERROR);
        goto QUIT_FLAG;
    }
   

    checkSDCardCapacity();//检测SD 卡容量


    if (newFileNeeded)// 判断是否需要重新创建一个文件开始写
    {
        CreateAscFilePathWithTime(nowTimeInfo, filePath);// 将时间转换为文件路径
        filePath[sizeof(filePath) - 1] = '\0';
    }
  
    FILE *file = NULL;// 打开目标文件

    if (OpenNowWriteAscFile(filePath, &file) == 0  && file != NULL)
    {
    }
    else
    {
        LOG("[SD Card] ERROR: OpenNowWriteAscFile failed for: %s\n", filePath);
        goto QUIT_FLAG; // 打开失败 直接返回
    }
  
    fseek(file, 0, SEEK_END);
    long startFileSize = ftell(file);

    if (newFileNeeded)// 如果是新创建的文件
    {
        LOG("[SD Card] 9. Writing headers for new file\n");
        // 先写入当前文件头
        if (AscFileWriteTimeHeader(file, &nowTimeInfo) != 0) {
            LOG("[SD Card] ERROR: Failed to write time header\n");
        } else {
            //printf("9.1 Time header written\n");
        }
        newFileNeeded = false;// 标记不需要重新创建了      
    }

    // 如果不是新创建的文件 从文件的末尾追加写入
    fseek(file, 0, SEEK_END);
    // printf("sd storage start\r\n");
    while (inactiveBuffer->count > 0)
    {
        CAN_LOG_MESSAGE *logMsg = &inactiveBuffer->buffer[inactiveBuffer->readIndex];

        char dataStr[3 * 64 + 1] = {0};

        int bytes = logMsg->msg.Length;//数据长度

        if (bytes > 64) bytes = 64; // 双保险

        for (int i = 0; i < bytes; ++i) {
            // 每次写 3 个字符："%02X "，最多写入 3*64=192 字符，留了 +1 结尾
            // 这里用 sizeof(dataStr) - i*3 做保护，避免意外越界
            snprintf(&dataStr[i * 3], (size_t)(sizeof(dataStr) - i * 3), "%02X ", logMsg->msg.Data[i]);
        }
        // 2) 方向标记
        const char *dir = (logMsg->channel == 0) ? "Rx" : "Tx";

        uint8_t dlc = CalculateDLC(bytes);

        // 3) 直接一次性写入最终字符串（避免中间缓冲 + memmove）
        char line[BUFFERED_WRITE_SIZE]; // 请把 BUFFERED_WRITE_SIZE 设为 >= 512
        int lineLen = snprintf(
            line, sizeof(line),
            "%.3f CANFD 1 %08lXx %s 0 0 d %d %d %s\r\n",
            logMsg->Timestamp,
            (unsigned long)logMsg->msg.ID,
            dir,
            dlc,
            logMsg->msg.Length,
            dataStr
        );
        if (lineLen < 0 || (size_t)lineLen >= sizeof(line)) 
        {
            // 被截断或出错：可以选择丢弃，或写一条告警，再继续下一条
            // fprintf(stderr, "line truncated or error, drop this frame\n");
            // 安全起见我们直接丢弃这一帧，避免潜在越界
            inactiveBuffer->readIndex = (inactiveBuffer->readIndex + 1) % BUFFER_SIZE;
            inactiveBuffer->count--;
            continue;
        }

        size_t written = fwrite(line, 1, (size_t)lineLen, file);
        (void)written; // 如需可检查 written == (size_t)lineLen

        inactiveBuffer->readIndex = (inactiveBuffer->readIndex + 1) % BUFFER_SIZE;
        inactiveBuffer->count--;

    }
    fflush(file);

    fseek(file, 0, SEEK_END);// 写完之后 计算文件大小
    long fileSize = ftell(file);

    // 创建新文件的两个条件
    // 1. 当前写的文件大小超过10M
    // 2. 系统中不存在当前日志命名的文件夹（日期变化了）
    if ((fileSize > (SD_FILE_SIZE) )|| (judgeTimetoUpdate(&nowTimeInfo))) // 大于10M或者年月日发生变化
    {
        LOG("[SD Card] fileSize = %ld\r\n",fileSize);
        LOG("[SD Card] judgeTimetoUpdate = %d\r\n",judgeTimetoUpdate(&nowTimeInfo));
        newFileNeeded = true; // 下一轮就要创建新文件
    }
    
    fclose(file);// 关闭文件

QUIT_FLAG:
    pthread_mutex_unlock(&inactiveBuffer->mutex);

    return;
}
int SD_Initialize(void)
{
    int res;
    const char *mount_point = USB_MOUNT_POINT;
    char cmd[256];

    // 直接删除所有文件（假设设备已经挂载）
    if(clean_directory(USB_MOUNT_POINT) <= 0){
        LOG("[SD Card] Clean files ERROR\n");       
    }
    LOG("[SD Card] Clean files completed\n");
    chdir(USB_MOUNT_POINT);
    usleep(100 * 1000);
    newFileNeeded = true;
    return 0;
}

// 回调函数：删除每个文件/目录
static int remove_cb(const char *fpath, const struct stat *sb,
                     int typeflag, struct FTW *ftwbuf)
{
    // 注意：nftw 默认是后序遍历（FTW_DEPTH），所以先删子目录内容，再删目录本身
    if (remove(fpath) != 0) {
        // 可选：记录错误，但不要中断整个删除（返回 0 继续）
        perror(fpath);
    }
    return 0; // 继续遍历
}
bool clean_directory(const char *directory)
{
    // 确保路径非空且不是根目录（安全防护）
    if (!directory || strlen(directory) < 2) {
        return false;
    }

    // 使用 nftw 递归删除
    // FTW_DEPTH: 后序遍历（先删子项，再删目录）
    // FTW_PHYS: 不跟随符号链接（避免误删）
    // 64: 文件描述符上限（通常足够）
    int ret = nftw(directory, remove_cb, 64, FTW_DEPTH | FTW_PHYS);
    
    if (ret == -1) {
        perror("nftw");
        return false;
    }
    return true;
}

// 检查SD卡容量并删除旧文件夹的线程任务
void checkSDCardCapacity(void)
{

    struct statvfs stat;
    if (statvfs(USB_MOUNT_POINT, &stat) != 0)
    {
        LOG("[SD Card] Failed to get SD card capacity.\n");
        usleep(CHECKSD_TRIGGERING_TIME);
    }
    uint64_t total = (uint64_t)stat.f_blocks * (uint64_t)stat.f_frsize;
    uint64_t free_space = (uint64_t)stat.f_bavail  * (uint64_t)stat.f_frsize;
    uint64_t used = total - free_space;

    float usage_percent = ((float)used / (float)total) * 100.0f;

    
    // LOG("SD Card total:%d\n", total);
    // LOG("SD Card free_space:%d\n", free_space);
    // LOG("SD Card used:%d\n", used);
    // LOG("SD Card usage_percent:%.2f%%\n", usage_percent);

    if (usage_percent >= SDMAXCAPACITY)
    {
        LOG("[SD Card] SD Card usage_percent:%.2f%% > %d\r\n", usage_percent,SDMAXCAPACITY);
        Func_DeleteOldestFolder();
    }
    usleep(1000*1000);
}

void checkRootCapacity(void)
{
    struct statvfs stat;
    if (statvfs("/", &stat) != 0)
    {
        LOG("[Root] Failed to get root partition capacity.\n");
        usleep(CHECKSD_TRIGGERING_TIME);
        return;
    }
    
    // 使用正确的类型和转换
    uint64_t total = (uint64_t)stat.f_blocks * (uint64_t)stat.f_frsize;
    uint64_t free_space = (uint64_t)stat.f_bavail * (uint64_t)stat.f_frsize;
    uint64_t used = total - free_space;
    
    float usage_percent = (total > 0) ? ((float)used / (float)total) * 100.0f : 0.0f;

    if (usage_percent > 60.0f) {
        LOG("Warning: The root partition usage rate has exceeded 60%%, and the space is tight!");
    }
    if (usage_percent > 80.0f) {
        LOG("Error:   The root partition usage rate has exceeded 80%%, and the space is tight!");
        set_emcu_fault(SD_FAULT, SET_ERROR);
    }
    // 或者用更简单的方法（避免PRIu64）
    // printf("Simple format:\n");
    // printf("  Total: %llu bytes (%.2f GB)\n", 
    //     (unsigned long long)total, total / (1024.0 * 1024 * 1024));
    // printf("  Free:  %llu bytes (%.2f GB)\n", 
    //     (unsigned long long)free_space, free_space / (1024.0 * 1024 * 1024));
    // printf("  Used:  %llu bytes (%.2f GB)\n", 
    //     (unsigned long long)used, used / (1024.0 * 1024 * 1024));
    // printf("  Usage: %.2f%%\n", usage_percent);
}
static uint8_t CalculateDLC(uint8_t data_length) {
    if (data_length <= 8) {
        return data_length;
    } else if (data_length <= 12) {
        return 9;
    } else if (data_length <= 16) {
        return 10;
    } else if (data_length <= 20) {
        return 11;
    } else if (data_length <= 24) {
        return 12;
    } else if (data_length <= 32) {
        return 13;
    } else if (data_length <= 48) {
        return 14;
    } else { // 64字节
        return 15;
    }
}

static int find_id_index(uint32_t id)
{
    for (int i = 0; i < CAN_ID_HISTORY_SIZE; i++) {
        if (CAN_IDs[i] == id) {
            return i;
        }
    }
    return -1; // 非法 ID（应已被 Drv_check_and_update_message 过滤）
}
