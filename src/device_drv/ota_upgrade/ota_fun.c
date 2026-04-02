#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "ota_fun.h"
#include "sd_store.h"
#include "ota_ecu_update.h"
#include "ota_other_update.h"
#include "interface/log/log.h"

#define COPY_BUFFER_SIZE (10*1024 * 1024)  // 10MB buffer
#define BAT_ECU_TARGET_PATH "/opt/xcharge/bat_ecu"

extern ECUStatus ecustatus;
OTAObject g_otactrl ={0};

UpgradeInfo g_max_upgrade = {0};  // 全局保存最大者
static int global_max_index = -1;      // 当前最大 X

// 互斥锁保护非原子变量
pthread_mutex_t ota_mutex = PTHREAD_MUTEX_INITIALIZER;//多读一写适合读写锁

 unsigned int get_ota_deviceID(void) {
    return atomic_load(&g_otactrl.deviceID);
}

 void set_ota_deviceID(unsigned int value) {
    atomic_store(&g_otactrl.deviceID, value);
}

unsigned int get_ota_acOTAFlag(void) {
    return atomic_load(&g_otactrl.acOTAFlag);
}

 void set_ota_acOTAFlag(unsigned int value) {
    atomic_store(&g_otactrl.acOTAFlag, value);
}


 unsigned char get_ota_deviceType(void) {
    return atomic_load(&g_otactrl.deviceType);
}

 void set_ota_deviceType(unsigned char value) {
    atomic_store(&g_otactrl.deviceType, value);
}

 unsigned char get_ota_OTAStart(void) {
    return atomic_load(&g_otactrl.OTAStart);
}

 void set_ota_OTAStart(unsigned char value) {
    atomic_store(&g_otactrl.OTAStart, value);
}

 unsigned char get_ota_multDeviceOTA(void) {
    return atomic_load(&g_otactrl.multDeviceOTA);
}

 void set_ota_multDeviceOTA(unsigned char value) {
    atomic_store(&g_otactrl.multDeviceOTA, value);
}

 unsigned char get_ota_multDeviceOTANum(void) {
    return atomic_load(&g_otactrl.multDeviceOTANum);
}

 void set_ota_multDeviceOTANum(unsigned char value) {
    atomic_store(&g_otactrl.multDeviceOTANum, value);
}

 unsigned char get_ota_UpDating(void) {
    return atomic_load(&g_otactrl.UpDating);
}

 void set_ota_UpDating(unsigned char value) {
    atomic_store(&g_otactrl.UpDating, value);
}

inline unsigned char get_ota_OTAFileType(void) {
    return atomic_load(&g_otactrl.OTAFileType);
}

inline void set_ota_OTAFileType(unsigned char value) {
    atomic_store(&g_otactrl.OTAFileType, value);
}

// ============ 非原子变量的 get/set 函数实现 ============

 const char* get_ota_OTAFilename(void) {
    pthread_mutex_lock(&ota_mutex);
    const char* result = g_otactrl.OTAFilename;
    pthread_mutex_unlock(&ota_mutex);
    return result;
}

 void set_ota_OTAFilename(const char* filename) {
    pthread_mutex_lock(&ota_mutex);
    if (filename) {
        strncpy(g_otactrl.OTAFilename, filename, OTAFILENAMEMAXLENGTH - 1);
        g_otactrl.OTAFilename[OTAFILENAMEMAXLENGTH - 1] = '\0';
    } else {
        g_otactrl.OTAFilename[0] = '\0';
    }
    pthread_mutex_unlock(&ota_mutex);
}

 const char* get_ota_OTAUdsSblFilename(int index) {
    pthread_mutex_lock(&ota_mutex);
    const char* result = NULL;
    if (index >= 0 && index < MAX_FILE_COUNT) {
        result = g_otactrl.OTAUdsSblFilename[index];
    }
    pthread_mutex_unlock(&ota_mutex);
    return result;
}

 void set_ota_OTAUdsSblFilename(int index, const char* filename) {
    pthread_mutex_lock(&ota_mutex);
    if (index >= 0 && index < MAX_FILE_COUNT && filename) {
        strncpy(g_otactrl.OTAUdsSblFilename[index], filename, OTAFILENAMEMAXLENGTH - 1);
        g_otactrl.OTAUdsSblFilename[index][OTAFILENAMEMAXLENGTH - 1] = '\0';
    }
    pthread_mutex_unlock(&ota_mutex);
}

 const char* get_ota_OTAUdsFilename(int index) {
    pthread_mutex_lock(&ota_mutex);
    const char* result = NULL;
    if (index >= 0 && index < MAX_FILE_COUNT) {
        result = g_otactrl.OTAUdsFilename[index];
    }
    pthread_mutex_unlock(&ota_mutex);
    return result;
}

 void set_ota_OTAUdsFilename(int index, const char* filename) {
    pthread_mutex_lock(&ota_mutex);
    if (index >= 0 && index < MAX_FILE_COUNT && filename) {
        strncpy(g_otactrl.OTAUdsFilename[index], filename, OTAFILENAMEMAXLENGTH - 1);
        g_otactrl.OTAUdsFilename[index][OTAFILENAMEMAXLENGTH - 1] = '\0';
    }
    pthread_mutex_unlock(&ota_mutex);
}


// 从 "upgrade123" 提取数字 123，失败返回 -1
static int extract_index(const char* section) {
    if (strncmp(section, "upgrade", 7) != 0) {
        return -1;
    }
    const char* p = section + 7;
    if (*p == '\0') return -1; // 没有数字

    // 检查剩余部分是否全为数字
    for (const char* q = p; *q; q++) {
        if (!isdigit((unsigned char)*q)) {
            return -1;
        }
    }

    return atoi(p);
}


// 修改 find_ota_files_simple 函数
static int find_ota_files_simple(const char *extract_dir, file_type_t file_type,char *conf_path, size_t conf_len, 
                         char *file_path, size_t file_len) {
    FILE *fp;
    char cmd[1024];
    
    // 查找 .conf 文件
    snprintf(cmd, sizeof(cmd), "find \"%s\" -type f \\( -name \"*.conf\" -o -name \"*.CONF\" \\) 2>/dev/null", 
             extract_dir);
    
    LOG("[OTA] Running command: %s\n", cmd);
    fp = popen(cmd, "r");
    if (fp) {
        int found = 0;
        char line[1024];
        while (fgets(line, sizeof(line), fp)) {
            line[strcspn(line, "\n")] = '\0';
            LOG("[OTA] Found conf candidate: %s\n", line);
            if (!found) {
                strncpy(conf_path, line, conf_len - 1);
                conf_path[conf_len - 1] = '\0';
                found = 1;
            }
        }
        pclose(fp);
        if (!found) {
            LOG("[OTA] No conf file found\n");
        }
    } else {
        LOG("[OTA] Failed to execute find command\n");
    }
    
    // 根据文件类型构建查找命令
    const char *ext_patterns = NULL;
    switch (file_type) {
        case FILE_TYPE_DEB:
            ext_patterns = "\\( -name \"*.deb\" -o -name \"*.DEB\" \\)";
            break;
        case FILE_TYPE_BIN:
            ext_patterns = "\\( -name \"*.bin\" -o -name \"*.BIN\" \\)";
            break;
        case FILE_TYPE_IMG:
            ext_patterns = "\\( -name \"*.img\" -o -name \"*.IMG\" \\)";
            break;
        case FILE_TYPE_BAT_ECU:
            ext_patterns = "\\( -name \"bat_ecu\" \\)";
            break;
        case FILE_TYPE_CONF_ONLY:
            // 仅查找conf文件，直接返回
            LOG("[OTA] Final conf_path: %s\n", conf_path);
            return (strlen(conf_path) > 0);
        default:
            LOG("[OTA] Unknown file type: %d\n", file_type);
            return 0;
    }

    snprintf(cmd, sizeof(cmd), "find \"%s\" -type f %s 2>/dev/null", 
            extract_dir, ext_patterns);   

    LOG("[OTA] Running command: %s\n", cmd);
    fp = popen(cmd, "r");
    if (fp) {
        int found = 0;
        char line[1024];
        while (fgets(line, sizeof(line), fp)) {
            line[strcspn(line, "\n")] = '\0';
            LOG("[OTA] Found file candidate: %s\n", line);
            if (!found) {
                strncpy(file_path, line, file_len - 1);
                file_path[file_len - 1] = '\0';
                found = 1;
            }
        }
        pclose(fp);
        if (!found) {
            LOG("[OTA] No deb file found\n");
        }
    } else {
        LOG("[OTA] Failed to execute find command\n");
    }
    
    LOG("[OTA] Final conf_path: %s\n", conf_path);
    LOG("[OTA] Final file_path: %s\n", file_path);
    LOG("[OTA] conf_path valid: %d, file_path valid: %d\n", 
         (strlen(conf_path) > 0), (strlen(file_path) > 0));
    
    return (strlen(conf_path) > 0 && strlen(file_path) > 0);
}


static int compute_file_md5(const char *filepath, char *out_md5) {
    char cmd[512];
    FILE *fp;
    snprintf(cmd, sizeof(cmd), "md5sum \"%s\" | cut -d' ' -f1", filepath);
    fp = popen(cmd, "r");
    if (!fp) return -1;

    if (fgets(out_md5, 33, fp) == NULL) {
        pclose(fp);
        return -1;
    }

    // 去掉换行符
    out_md5[strcspn(out_md5, "\n")] = 0;
    pclose(fp);
    return 0;
}


// 回调函数：只保留 index 最大的 upgrade
static int handler(void* user, const char* section, const char* name,
                   const char* value) {
    (void)user;
    int idx = extract_index(section);
    if (idx < 0) {
        return 1; // 忽略非 upgradeX 的 section
    }

    // 如果当前 section 的 index 更大，就重置结构体
    if (idx > global_max_index) {
        memset(&g_max_upgrade, 0, sizeof(g_max_upgrade));
        strncpy(g_max_upgrade.section, section, sizeof(g_max_upgrade.section) - 1);
        g_max_upgrade.index = idx;
        global_max_index = idx;
    }

    // 只处理当前最大 index 的字段（避免旧数据污染）
    if (idx != global_max_index) {
        return 1;
    }

    #define MATCH(s) strcmp(name, s) == 0
    if (MATCH("old_version")) {
        strncpy(g_max_upgrade.old_version, value, sizeof(g_max_upgrade.old_version) - 1);
    } else if (MATCH("new_version")) {
        strncpy(g_max_upgrade.new_version, value, sizeof(g_max_upgrade.new_version) - 1);
    } else if (MATCH("upgrade_file")) {
        strncpy(g_max_upgrade.upgrade_file, value, sizeof(g_max_upgrade.upgrade_file) - 1);
    } else if (MATCH("md5sum")) {
        strncpy(g_max_upgrade.md5sum, value, sizeof(g_max_upgrade.md5sum) - 1);
    }
    #undef MATCH

    return 1;
}
int unzipfile(char * cp_filepath,unsigned int *error_status, file_type_t file_type){

    char sd_source_file[512] = {'\0'};//SD的路径

    // 步骤1: 检查SD卡的文件是否存在
    if(*error_status == 0)
    {      
        snprintf(sd_source_file, sizeof(sd_source_file), "%s/%s", USB_MOUNT_POINT, get_ota_OTAFilename());
        
        LOG("Source file path: %s\n", sd_source_file);
        // 检查源文件是否存在
        if (access(sd_source_file, F_OK) != 0) {
            LOG("[OTA] source file does not exist: %s\n", sd_source_file);
            *error_status |= 1 << 2;
            goto upcelanup;
        }         
    }

    // 步骤2: 在tmp创建临时文件夹
    char extract_dir[512] = {0};
    snprintf(extract_dir, sizeof(extract_dir), "/tmp/ota_extract_%d", (int)getpid());

    if (access(extract_dir, F_OK) == 0) { // 清理已存在的目录
        LOG("[OTA] Cleaning existing directory: %s\n", extract_dir);
        if (!clean_directory(extract_dir)) {
            LOG("[OTA] Warning: Failed to fully clean directory\n");
        }
    }

    if (mkdir(extract_dir, 0755) != 0 && errno != EEXIST) {
        LOG("[OTA] Failed to create extract dir: %s\n", extract_dir);
        *error_status |= 1 << 3; // 自定义错误位，比如 bit3 表示解压失败
        goto upcelanup;
    }

    // 步骤3: 将SD 卡的文件解压到tmp内
    char tar_cmd[512] = {0};
    snprintf(tar_cmd, sizeof(tar_cmd), "tar -xf \"%s\" -C \"%s\"", sd_source_file, extract_dir);

    LOG("[OTA] Executing tar command: %s\n", tar_cmd);
    int ret = system(tar_cmd);
    if (ret != 0) {
        LOG("[OTA] Tar extraction failed!\n");
        *error_status |= 1 << 3;
        goto upcelanup;
    }

    // 步骤4: 在解压后的文件夹查找文件名为 conf和 deb或bin,压缩文件和解压文件名必须相同
    char base_name[256] = {0};
    char *dot = strrchr(get_ota_OTAFilename(), '.');
    if (dot && strcmp(dot, ".tar") == 0) {
        strncpy(base_name, get_ota_OTAFilename(), dot - get_ota_OTAFilename());
    } else {
        strcpy(base_name, get_ota_OTAFilename()); // fallback
    }

    // 步骤5: 检查conf和file文件是否存在
    char conf_path[512] = {0};
    char file_path[512] = {0};

    if (!find_ota_files_simple(extract_dir, file_type, conf_path, sizeof(conf_path), 
                            file_path, sizeof(file_path))) {
        LOG("[OTA] Could not find required files in extracted archive\n");  
        *error_status |= 1 << 4;
        goto upcelanup;
    }

    LOG("[OTA] Found conf: %s\n", conf_path);
    LOG("[OTA] Found file: %s\n", file_path);

    // 步骤6: 解析conf文件,赋值给max_upgrade
    // =========== 添加这行 ===========
    global_max_index = -1;
    memset(&g_max_upgrade, 0, sizeof(g_max_upgrade));//重置全剧变量，避免污染

    int err = ini_parse(conf_path, handler, NULL);
    if (err < 0) {
        LOG("Unable to read configuration file 'upgrade.conf'\n");
        *error_status |= 1 << 5; // 配置文件异常
        goto upcelanup;
    } else if (err > 0) {
        LOG("There is an error on line %d of the configuration file\n", err);
        *error_status |= 1 << 5; // 配置文件异常
        goto upcelanup;
    }

    if (global_max_index == -1) {
        LOG("No upgradeX configuration found\n");
        *error_status |= 1 << 5; // 配置文件异常
        goto upcelanup;
    }

    // 步骤7: 直接复制ecu文件到/var目录
    if(*error_status == 0)
    {
        // 1. 获取文件路径
        char target_file[512] = {'\0'};           
   
        if (file_type == FILE_TYPE_BAT_ECU) {
            strncpy(target_file, BAT_ECU_TARGET_PATH, sizeof(target_file) - 1);
        } else {
            snprintf(target_file, sizeof(target_file), "%s/%s",  cp_filepath, g_max_upgrade.upgrade_file);
        }

        LOG("[OTA] Copying from: %s\n", file_path);
        LOG("[OTA] Copying to: %s\n", target_file);
    
        // 2. 计算MD5值
        char computed_md5[33] = {0}; // 32 hex + '\0'
        if (compute_file_md5(file_path, computed_md5) != 0) {
            LOG("[OTA] Failed to compute MD5 for %s\n", file_path);
            *error_status |= 1 << 7; // MD5 计算失败
            goto upcelanup;
        }

        // 3. 比较 MD5（转为小写再比较，因为 md5sum 工具输出小写，但配置可能大写）
        char expected_md5[33] = {0};
        strncpy(expected_md5, g_max_upgrade.md5sum, 32);
        expected_md5[32] = '\0';
        for (int i = 0; expected_md5[i]; i++) {
            expected_md5[i] = tolower((unsigned char)expected_md5[i]);
        }
        for (int i = 0; computed_md5[i]; i++) {
            computed_md5[i] = tolower((unsigned char)computed_md5[i]);
        }

        if (strcmp(computed_md5, expected_md5) != 0) {
            LOG("[OTA] MD5 mismatch!\n");
            *error_status |= 1 << 8; // MD5 校验失败
            goto upcelanup;
        }
        // 4. 比较通过,复制文件到/cp_filepath
        LOG("[OTA] MD5 verification passed.\n");

        if (!copy_file(file_path, target_file)) {
            LOG("[OTA] File copy failed!\n");
            *error_status |= 1 << 1; // 自定义错误位
            goto upcelanup;
        }else{
            LOG("[OTA] File copy successful!\n");
        }

        if (file_type == FILE_TYPE_BAT_ECU) {
            if (chmod(target_file, 0755) != 0) {
                LOG("[OTA] Failed to chmod %s: %s\n", target_file, strerror(errno));
                *error_status |= 1 << 1;
                goto upcelanup;
            }
        }
        
    }else{
        goto upcelanup;
    }

    return 1;

upcelanup:
    return -1;
}

int copy_file(const char *src, const char *dst)
{
    int fd_in = -1, fd_out = -1;
    char *buffer = NULL;
    bool success = false;

    // 打开源文件（只读）
    fd_in = open(src, O_RDONLY);
    if (fd_in == -1) {
        LOG("[OTA] Failed to open source file %s: %s\n", src, strerror(errno));
        goto cleanup;
    }

    // 创建目标文件（写入，权限 0644）
    fd_out = open(dst, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd_out == -1) {
        LOG("[OTA] Failed to create target file %s: %s\n", dst, strerror(errno));
        goto cleanup;
    }

    // 分配缓冲区
    buffer = malloc(COPY_BUFFER_SIZE);
    if (!buffer) {
        LOG("[OTA] Out of memory\n");
        goto cleanup;
    }

    // 循环读写
    ssize_t bytes_read;
    while ((bytes_read = read(fd_in, buffer, COPY_BUFFER_SIZE)) > 0) {
        ssize_t written = 0;
        while (written < bytes_read) {
            ssize_t n = write(fd_out, buffer + written, bytes_read - written);
            if (n <= 0) {
                if (errno == EINTR) continue; // 被信号中断，重试
                LOG("[OTA] Write failed: %s\n", strerror(errno));
                goto cleanup;
            }
            written += n;
        }
    }

    if (bytes_read == -1) {
        LOG("[OTA] Read failed: %s\n", strerror(errno));
        goto cleanup;
    }

    // 同步到磁盘（可选，提高可靠性）
    if (fsync(fd_out) != 0) {
        LOG("[OTA] fsync failed: %s\n", strerror(errno));
        // 可选择是否视为失败
    }

    success = true;

cleanup:
    if (buffer) free(buffer);
    if (fd_in != -1) close(fd_in);
    if (fd_out != -1) close(fd_out);

    if (!success && dst) {
        // 复制失败时删除不完整的文件
        unlink(dst);
    }

    return success;
}
