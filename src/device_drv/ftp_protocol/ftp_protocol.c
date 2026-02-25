#include "ftp_protocol.h"
#include "device_drv/sd_store/sd_store.h"
#include "interface/setting/ip_setting.h"
#define DATA_PORT 40900
#define FTP_BUFFER_SIZE 2920 // 2048 网络传输包mtu限制改为1460得倍数
#define TIMEOUT_SECONDS 300000
pthread_mutex_t ftp_file_io_mutex = PTHREAD_MUTEX_INITIALIZER;
static int is_safe_path(const char *path);
static int build_safe_filepath(const FTPState *state, const char *name, char *filepath, size_t filepath_size);

// 安全关闭文件的辅助函数
static void safe_close_file(FILE **file_ptr) {
    if (file_ptr && *file_ptr) {
        fclose(*file_ptr);
        *file_ptr = NULL;
    }
}

// 安全关闭socket的辅助函数
static void safe_close_socket(int *sock_fd) {
    if (sock_fd && *sock_fd >= 0) {
        close(*sock_fd);
        *sock_fd = -1;
    }
}

// 统一的FTP状态清理函数
static void cleanup_ftp_state(FTPState *state) {
    if (!state) return;
    safe_close_socket(&state->data_sock);
    safe_close_socket(&state->control_sock);
    safe_close_file(&state->file);
    state->logged_in = 0;
    state->last_activity = 0;
    state->timeout_pending = 0;
    state->quit_requested = 0;
    state->path[0] = '\0';
}

void send_response(int sock, const char *message)
{
    send(sock, message, strlen(message), 0);
}

// 更新最后活动时间
void update_last_activity(FTPState *state)
{
    state->last_activity = time(NULL);
}


// 改进的超时检查函数
// 改进的超时检查，增加优雅关闭机制
// 添加详细的超时信息
void check_timeouts(FTPState *state) {
    if (!state) return;
    
    time_t current_time = time(NULL);
    time_t time_diff = current_time - state->last_activity;
    
    if (time_diff > TIMEOUT_SECONDS) {    
        // 发送超时通知
        if (state->control_sock >= 0) {
            send(state->control_sock, "421 Service not available, closing control connection.\r\n", 54, 0);
        }
        state->timeout_pending = 1;
    }
    else {
        // 可选：添加调试信息
    }
}

// 处理 USER 命令
static void handle_user_command(FTPState *state, char *args)
{
    update_last_activity(state);
    send_response(state->control_sock, "331 Username OK, need password.\r\n");
}

// 处理 PASS 命令
static void handle_pass_command(FTPState *state, char *args)
{
    update_last_activity(state);
    state->logged_in = 0;

    // 切换到 U 盘挂载目录
    if (chdir(USB_MOUNT_POINT) != 0)
    {
        LOG("Failed to change to USB directory: %s\n", strerror(errno));
        send_response(state->control_sock, "550 Failed to access USB directory.\r\n");
        return;
    }

    if (getcwd(state->path, sizeof(state->path)) == NULL)
    {
        LOG("Failed to get USB directory path: %s\n", strerror(errno));
        send_response(state->control_sock, "550 Failed to access USB directory.\r\n");
        return;
    }

    state->logged_in = 1;
    send_response(state->control_sock, "230 Login successful.\r\n");
}

static void handle_pasv_command(FTPState *state)
{
    update_last_activity(state);

    struct timeval timeout;
    uint32_t host_ip = g_ipsetting.ip; // 主机字节序
    char response[FTP_BUFFER_SIZE] = {0};

    // 拆分为四个字节（从高到低）
    uint8_t a = (host_ip >> 24) & 0xFF;
    uint8_t b = (host_ip >> 16) & 0xFF;
    uint8_t c = (host_ip >>  8) & 0xFF;
    uint8_t d = (host_ip      ) & 0xFF;

    snprintf(response, sizeof(response),
             "227 Entering Passive Mode (%d,%d,%d,%d,%d,%d).\r\n",
             a, b, c, d, DATA_PORT / 256, DATA_PORT % 256);//新端口号

    if (state->data_sock >= 0)
    {
        close(state->data_sock);
        state->data_sock = -1;
    }

    state->data_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (state->data_sock < 0)
    {
        perror("Failed to create data socket");
        return;
    }

    struct sockaddr_in data_addr;
    memset(&data_addr, 0, sizeof(data_addr));
    data_addr.sin_family = AF_INET;
    data_addr.sin_addr.s_addr = INADDR_ANY;
    data_addr.sin_port = htons(DATA_PORT);

    int opt = 1;
    if (setsockopt(state->data_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        perror("setsockopt SO_REUSEADDR");
    }

    if (bind(state->data_sock, (struct sockaddr *)&data_addr, sizeof(data_addr)) < 0)
    {
        perror("Failed to bind data socket");
        close(state->data_sock);
        state->data_sock = -1;
        return;
    }

    if (listen(state->data_sock, 2) < 0)
    {
        perror("Failed to listen on data socket");
        close(state->data_sock);
        state->data_sock = -1;
        return;
    }

    timeout.tv_sec = 60;
    timeout.tv_usec = 0;
    if (setsockopt(state->data_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        perror("Failed to set socket receive timeout");
        close(state->data_sock);
        state->data_sock = -1;
        return;
    }

    send_response(state->control_sock, response);
}

static void handle_port_command(FTPState *state, char *args)
{
    update_last_activity(state);

    int ip[4];
    int port_high, port_low;
    if (!args)
    {
        send_response(state->control_sock, "501 Syntax error in parameters or arguments.\r\n");
        return;
    }

    if (sscanf(args, "%d,%d,%d,%d,%d,%d", &ip[0], &ip[1], &ip[2], &ip[3], &port_high, &port_low) != 6)
    {
        send_response(state->control_sock, "501 Syntax error in parameters or arguments.\r\n");
        return;
    }

    int port = (port_high << 8) | port_low;

    struct sockaddr_in data_addr;
    memset(&data_addr, 0, sizeof(data_addr));
    data_addr.sin_family = AF_INET;
    data_addr.sin_port = htons(port);

    char ip_str[INET_ADDRSTRLEN];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    if (inet_pton(AF_INET, ip_str, &data_addr.sin_addr) != 1)
    {
        fprintf(stderr, "Failed to convert IP address\n");
        send_response(state->control_sock, "501 Invalid IP address.\r\n");
        return;
    }

    // 关闭旧连接（如果有）
    if (state->data_sock >= 0)
    {
        close(state->data_sock);
        state->data_sock = -1;
    }

    // 创建 socket 并尝试连接客户端提供的数据端口
    state->data_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (state->data_sock < 0)
    {
        perror("Failed to create data socket");
        send_response(state->control_sock, "425 Can't open data connection.\r\n");
        return;
    }

    if (connect(state->data_sock, (struct sockaddr *)&data_addr, sizeof(data_addr)) < 0)
    {
        perror("Failed to connect to client data port");
        send_response(state->control_sock, "425 Can't open data connection.\r\n");
        close(state->data_sock);
        state->data_sock = -1;
        return;
    }

    send_response(state->control_sock, "200 PORT command successful.\r\n");
}

static void handle_list_command(FTPState *state, char *args)
{
    update_last_activity(state);
    send_response(state->control_sock, "150 Here comes the directory listing.\r\n");

    int client_data_sock = accept(state->data_sock, (struct sockaddr *)&state->client_addr, &state->client_addr_len);

    if (client_data_sock < 0)
    {
        LOG("Failed to accept data connection: errno=%d, %s\n", errno, strerror(errno));
        send_response(state->control_sock, "425 Can't open data connection.\r\n");
        close(state->data_sock);
        return;
    }

    DIR *dir;
    struct dirent *entry;
    char buffer[FTP_BUFFER_SIZE] = {0};
    char cwd[FTP_BUFFER_SIZE] = {0};

    if (getcwd(cwd, sizeof(cwd)) == NULL)
    {
        LOG("Failed to get current working directory\n");
        send_response(state->control_sock, "550 Failed to get current directory.\r\n");
        close(client_data_sock);
        close(state->data_sock);
        return;
    }

    dir = opendir(cwd);
    if (!dir)
    {
        LOG("Failed to open directory: %s\n", strerror(errno));
        send_response(state->control_sock, "550 Failed to open directory.\r\n");
        close(client_data_sock);
        close(state->data_sock);
        return;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        // Skip hidden files unless "-a" is specified
        if ((!args || strcmp(args, "-a") != 0) && entry->d_name[0] == '.')
        {
            continue;
        }

        char fullpath[FTP_BUFFER_SIZE];
        snprintf(fullpath, sizeof(fullpath), "%s/%s", cwd, entry->d_name);

        struct stat st;
        if (stat(fullpath, &st) == -1)
        {
            LOG("Failed to stat file %s\n", entry->d_name);
            continue;
        }

        if (S_ISDIR(st.st_mode))
        {
            snprintf(buffer, sizeof(buffer), "+/,\t%s\r\n", entry->d_name);
        }
        else
        {
            snprintf(buffer, sizeof(buffer), "+r,s%ld,\t%s\r\n", st.st_size, entry->d_name);
        }

        if (send(client_data_sock, buffer, strlen(buffer), 0) < 0)
        {
            LOG("Failed to send data: %s\n", strerror(errno));
            send_response(state->control_sock, "426 Connection closed; transfer aborted.\r\n");
            closedir(dir);
            close(client_data_sock);
            close(state->data_sock);
            return;
        }

        LOG("sent to client_data_sock %d: %s", client_data_sock, buffer);
    }

    closedir(dir);
    close(client_data_sock);
    close(state->data_sock);
    state->data_sock = -1;

    send_response(state->control_sock, "226 Directory send OK.\r\n");
}

/*
 * ftp 开始的时候设置标志位，
 * 	然后里边调用各自的接口去实现文件的关闭
 * 	并且不允许再次去写文件。
 *
 */
static bool ftp_read_flag = false;
void set_ftp_read_file_flag(bool flag)
{
    static bool flag_last = false;
    if (flag_last == flag)
        return;

    ftp_read_flag = flag;
    flag_last = ftp_read_flag;
}

bool get_ftp_read_file_flag()
{
    return ftp_read_flag;
}

// 在所有涉及文件IO的函数中添加完整的锁保护
static void handle_retr_command(FTPState *state, char *filename) {
    int client_data_sock = -1;
    int result = -1;
    char filepath[512] = {0};

    if (build_safe_filepath(state, filename, filepath, sizeof(filepath)) != 0) {
        send_response(state->control_sock, "550 Invalid filename.\r\n");
        return;
    }
    
    pthread_mutex_lock(&ftp_file_io_mutex);
    
    do {
        // 接受数据连接
        client_data_sock = accept(state->data_sock,
                                (struct sockaddr *)&state->client_addr,
                                &state->client_addr_len);
        if (client_data_sock < 0) {
            LOG("Failed to accept data connection: %s\n", strerror(errno));
            send_response(state->control_sock, "425 Can't open data connection.\r\n");
            break;
        }

        send_response(state->control_sock, "150 Opening data connection.\r\n");

        LOG("Loading file: %s\n", filepath);

        // 打开文件
        state->file = fopen(filepath, "rb");
        if (!state->file) {
            LOG("Failed to open file: %s\n", strerror(errno));
            send_response(state->control_sock, "550 File not found.\r\n");
            break;
        }

        // 传输文件数据
        char buffer[FTP_BUFFER_SIZE];
        size_t bytes_read;
        result = 0;

        while ((bytes_read = fread(buffer, 1, sizeof(buffer), state->file)) > 0) {
            update_last_activity(state);
            
            if (send(client_data_sock, buffer, bytes_read, 0) < 0) {
                LOG("Failed to send data: %s\n", strerror(errno));
                result = -1;
                break;
            }
        }
        
    } while(0);

    // 清理资源
    safe_close_file(&state->file);
    safe_close_socket(&client_data_sock);
    safe_close_socket(&state->data_sock);
    
    pthread_mutex_unlock(&ftp_file_io_mutex);

    // 发送最终响应
    if (result == 0) {
        send_response(state->control_sock, "226 Transfer complete.\r\n");
    } else {
        send_response(state->control_sock, "426 Connection closed; transfer aborted.\r\n");
    }
}

static void handle_stor_command(FTPState *state, char *filename) {
    int client_data_sock = -1;
    FILE *file = NULL;
    int result = -1;
    char filepath[512] = {0};

    if (build_safe_filepath(state, filename, filepath, sizeof(filepath)) != 0) {
        send_response(state->control_sock, "550 Invalid filename.\r\n");
        return;
    }
    
    send_response(state->control_sock, "150 Opening data connection.\r\n");

    do {
        // 接受数据连接
        client_data_sock = accept(state->data_sock, 
                                (struct sockaddr *)&state->client_addr, 
                                &state->client_addr_len);
        if (client_data_sock < 0) {
            LOG("Failed to accept data connection: %s\n", strerror(errno));
            send_response(state->control_sock, "425 Can't open data connection.\r\n");
            break;
        }

        // 创建文件
        file = fopen(filepath, "wb");
        if (!file) {
            LOG("Failed to create file: %s\n", strerror(errno));
            send_response(state->control_sock, "550 Failed to create file.\r\n");
            break;
        }

        // 接收并写入数据
        char buffer[FTP_BUFFER_SIZE];
        ssize_t bytes_received;
        result = 0;

        while ((bytes_received = recv(client_data_sock, buffer, sizeof(buffer), 0)) > 0) {
            update_last_activity(state);

            size_t bytes_written = fwrite(buffer, 1, bytes_received, file);
            if (bytes_written != bytes_received) {
                LOG("Failed to write data to file: %s\n", strerror(errno));
                result = -1;
                break;
            }
        }
        
    } while(0);

    // 清理资源
    safe_close_file(&file);
    safe_close_socket(&client_data_sock);
    safe_close_socket(&state->data_sock);

    // 发送最终响应
    if (result == 0) {
        send_response(state->control_sock, "226 Transfer complete.\r\n");
    } else {
        send_response(state->control_sock, "426 Connection closed; transfer aborted.\r\n");
    }
}

static void handle_mget_command(FTPState *state, char *args)
{
    char filepath[512] = {0};

    if (build_safe_filepath(state, args, filepath, sizeof(filepath)) != 0)
    {
        send_response(state->control_sock, "550 Invalid filename.\r\n");
        return;
    }

    send_response(state->control_sock, "150 Opening data connection.\r\n");

    int client_data_sock = accept(state->data_sock, (struct sockaddr *)&state->client_addr, &state->client_addr_len);
    if (client_data_sock < 0)
    {
        LOG("Failed to accept data connection: %s\n", strerror(errno));
        send_response(state->control_sock, "425 Can't open data connection.\r\n");
        close(state->data_sock);
        return;
    }

    FILE *file = fopen(filepath, "rb"); // Open the file for reading
    if (!file)
    {
        LOG("Failed to open file: %s\n", strerror(errno));
        send_response(state->control_sock, "550 File not found.\r\n");
        close(client_data_sock);
        close(state->data_sock);
        return;
    }

    char buffer[FTP_BUFFER_SIZE];
    size_t bytes_read;

    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0)
    {
        update_last_activity(state);
        if (send(client_data_sock, buffer, bytes_read, 0) < 0)
        {
            LOG("Failed to send data: %s\n", strerror(errno));
            send_response(state->control_sock, "426 Connection closed; transfer aborted.\r\n");
            fclose(file);
            state->file = NULL; 
            state->file = NULL; 
            close(client_data_sock);
            close(state->data_sock);
            return;
        }
    }

    fclose(file);
    state->file = NULL; 
    state->file = NULL; 
    close(client_data_sock);
    close(state->data_sock);
    state->data_sock = -1;

    send_response(state->control_sock, "226 Transfer complete.\r\n");
}

static void handle_pwd_command(FTPState *state)
{
    char cwd[FTP_BUFFER_SIZE];

    if (getcwd(cwd, sizeof(cwd)) != NULL)
    {
        char response[FTP_BUFFER_SIZE];
        snprintf(response, sizeof(response), "257 \"%s\" is the current directory.\r\n", cwd);
        send_response(state->control_sock, response);
    }
    else
    {
        send_response(state->control_sock, "550 Failed to get current directory.\r\n");
    }
}

static void handle_syst_command(FTPState *state)
{
    update_last_activity(state);
    send_response(state->control_sock, "215 UNIX Type: L8\r\n");
}

static void handle_cdup_command(FTPState *state)
{
    update_last_activity(state);

    // 获取当前目录
    char cwd[FTP_BUFFER_SIZE];
    if (getcwd(cwd, sizeof(cwd)) == NULL)
    {
        LOG("Failed to get current directory\n");
        send_response(state->control_sock, "550 Failed to get current directory.\r\n");
        return;
    }

    // 改变目录到上级目录
    if (chdir("..") != 0)
    {
        LOG("Failed to change to parent directory\n");
        send_response(state->control_sock, "550 Failed to change directory.\r\n");
        return;
    }

    if (getcwd(cwd, sizeof(cwd)) == NULL)
    {
        LOG("Failed to get current directory after CDUP\n");
        chdir(USB_MOUNT_POINT);
        strncpy(state->path, USB_MOUNT_POINT, sizeof(state->path) - 1);
        state->path[sizeof(state->path) - 1] = '\0';
        send_response(state->control_sock, "550 Failed to change directory.\r\n");
        return;
    }

    if (!is_safe_path(cwd))
    {
        chdir(state->path[0] ? state->path : USB_MOUNT_POINT);
        send_response(state->control_sock, "550 Access denied.\r\n");
        return;
    }

    strncpy(state->path, cwd, sizeof(state->path) - 1);
    state->path[sizeof(state->path) - 1] = '\0';
    send_response(state->control_sock, "200 Directory successfully changed.\r\n");
}

// 路径安全检查函数
static int is_safe_path(const char *path) {
    // 检查NULL指针
    if (!path) return 0;
    
    // 检查路径是否以允许的前缀开始
    if (strncmp(path, USB_MOUNT_POINT, strlen(USB_MOUNT_POINT)) != 0) {
        return 0;
    }
    
    // 检查路径中是否包含危险字符序列
    if (strstr(path, "..") != NULL) {
        return 0;
    }
    
    return 1;
}

static int build_safe_filepath(const FTPState *state, const char *name, char *filepath, size_t filepath_size)
{
    int len = 0;

    if (!state || !name || !filepath || filepath_size == 0) {
        return -1;
    }
    if (name[0] == '\0') {
        return -1;
    }
    if (name[0] == '/' || strstr(name, "..") != NULL) {
        return -1;
    }

    len = snprintf(filepath, filepath_size, "%s/%s", state->path, name);
    if (len < 0 || (size_t)len >= filepath_size) {
        return -1;
    }

    if (!is_safe_path(filepath)) {
        return -1;
    }
    return 0;
}

// 改进的CWD命令处理
static void handle_cwd_command(FTPState *state, const char *args) {
    update_last_activity(state);
    
    if (!args || strlen(args) == 0) {
        send_response(state->control_sock, "501 Syntax error in parameters or arguments.\r\n");
        return;
    }

    char target_path[512] = {0};

    // 处理绝对路径
    if (args[0] == '/') {
        if (strcmp(args, "/") == 0) {
            strncpy(target_path, USB_MOUNT_POINT, sizeof(target_path) - 1);
        } else {
            // 显式拼接 USB_MOUNT_POINT 和 args
            snprintf(target_path, sizeof(target_path), "%s%s", USB_MOUNT_POINT, args);
        }
    } else {
        // 处理相对路径
        LOG("[FTP] CWD state->path =  %s\r\n",state->path);
        snprintf(target_path, sizeof(target_path), "%s/%s", state->path, args);
    }

    // 安全检查
    if (!is_safe_path(target_path)) {
        LOG("CWD rejected: unsafe path (%s)\n", target_path);
        send_response(state->control_sock, "550 Access denied.\r\n");
        return;
    }

    // 尝试切换目录
    if (chdir(target_path) == 0) {
        if (getcwd(state->path, sizeof(state->path)) != NULL) {
            state->path[sizeof(state->path) - 1] = '\0';
            LOG("Changed to directory: %s\n", state->path);
            send_response(state->control_sock, "250 Directory changed.\r\n");
        } else {
            // 回退到安全目录
            chdir(USB_MOUNT_POINT);
            strncpy(state->path, USB_MOUNT_POINT, sizeof(state->path) - 1);
            send_response(state->control_sock, "550 Internal error.\r\n");
        }
    } else {
        LOG("Failed to change directory: %s (target_path=%s)\n", strerror(errno), target_path);
        send_response(state->control_sock, "550 Failed to change directory.\r\n");
    }
}


static void handle_type_command(FTPState *state, char *args)
{
    if (args && strcmp(args, "A") == 0)
    {
        send_response(state->control_sock, "200 Type set to A.\r\n");
    }
    else if (args && strcmp(args, "I") == 0)
    {
        send_response(state->control_sock, "200 Switching to Binary mode.\r\n");
    }
    else
    {
        send_response(state->control_sock, "504 Command not implemented for that parameter.\r\n");
    }
}


static void handle_size_command(FTPState *state, char *filename)
{
    update_last_activity(state);

    if (!state->logged_in)
    {
        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
        return;
    }

    if (!filename || strlen(filename) == 0)
    {
        send_response(state->control_sock, "501 Missing filename.\r\n");
        return;
    }

    char filepath[512] = {0};
    if (build_safe_filepath(state, filename, filepath, sizeof(filepath)) != 0)
    {
        send_response(state->control_sock, "550 Invalid filename.\r\n");
        return;
    }

    struct stat st;
    if (stat(filepath, &st) != 0)
    {
        LOG("SIZE: File not found - %s (%s)\n", filepath, strerror(errno));
        send_response(state->control_sock, "550 File not found.\r\n");
        return;
    }

    if (!S_ISREG(st.st_mode))
    {
        send_response(state->control_sock, "550 Not a regular file.\r\n");
        return;
    }

    char response[64];
    snprintf(response, sizeof(response), "213 %ld\r\n", (long)st.st_size);
    send_response(state->control_sock, response);
}

// 改进的QUIT命令处理
static void handle_quit_command(FTPState *state) {
    // 先发送响应
    if (state->control_sock >= 0) {
        send(state->control_sock, "221 Goodbye.\r\n", 14, 0);
    }
    
    // 标记准备退出，但不立即清理
    state->quit_requested = 1;
    
    // 等待一小段时间确保响应发送完成
    usleep(100000); // 100ms
}

void init_ftp_state(FTPState *state) {
    if (!state) return;
    
    state->control_sock = -1;
    state->data_sock = -1;
    state->file = NULL;
    state->logged_in = 0;
    state->last_activity = time(NULL);
    state->path[0] = '\0';
    state->timeout_pending = 0;
    state->quit_requested = 0;
}

int handle_ftp_commands(FTPState *state) {
    if (!state) return -1;
    
    char buffer[FTP_BUFFER_SIZE];
    ssize_t bytes_received;
    update_last_activity(state);

    while (1) {
        // 检查是否有待处理的超时
        if (state->timeout_pending) {
            LOG("Processing pending timeout\n");
            cleanup_ftp_state(state);
            return -3; // 超时退出
        }
        
        // 检查超时（但不立即清理）
        check_timeouts(state);
        
        // 检查socket有效性
        if (state->control_sock < 0) {
            LOG("Control socket closed\n");
            return -4;
        }
        
        // 设置较短的接收超时
        struct timeval timeout;
        timeout.tv_sec = 60;  // 1秒超时
        timeout.tv_usec = 0;
        
        if (setsockopt(state->control_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            LOG("Failed to set socket timeout: %s\n", strerror(errno));
        }
        
        bytes_received = recv(state->control_sock, buffer, sizeof(buffer) - 1, 0);// 接收命令
        
        if (bytes_received > 0) {
            // 重置超时标记
            state->timeout_pending = 0;
            update_last_activity(state);
            
            buffer[bytes_received] = '\0';
            LOG("Received command: %s", buffer);

            // 命令处理逻辑...
            char *command = strtok(buffer, " \r\n");
            char *args = strtok(NULL, "\r\n");

            if (command) 
            {
                //处理 FTP 命令
                if (strcmp(command, "USER") == 0) //用户名
                {
                    handle_user_command(state, args);
                }
                else if (strcmp(command, "PASS") == 0)//密码
                {
                    handle_pass_command(state, args);
                }
                else if (strcmp(command, "SIZE") == 0)
                {
                    handle_size_command(state, args);
                } 
                else if (strcmp(command, "PASV") == 0)//主动传输
                {
                    handle_pasv_command(state);
                }
                else if (strcmp(command, "PORT") == 0)
                {
                    handle_port_command(state, args);
                }
                else if (strcmp(command, "LIST") == 0)
                {
                    if (state->logged_in)
                    {
                        handle_list_command(state, args);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else if (strcmp(command, "RETR") == 0)//下载文件
                {
                    if (state->logged_in)
                    {
                        handle_retr_command(state, args);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else if (strcmp(command, "STOR") == 0)//上传指令
                {
                    if (state->logged_in)
                    {
                        handle_stor_command(state, args);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else if (strcmp(command, "MGET") == 0)
                {
                    if (state->logged_in)
                    {
                        handle_mget_command(state, args);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else if (strcmp(command, "PWD") == 0)//列出目录
                {
                    if (state->logged_in)
                    {
                        handle_pwd_command(state);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else if (strcmp(command, "CWD") == 0)//cd导航到目标目录
                {
                    if (state->logged_in)
                    {
                        handle_cwd_command(state, args);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else if (strcmp(command, "TYPE") == 0)//决定文件格式
                {
                    handle_type_command(state, args);
                }
                else if (strcmp(command, "SYST") == 0)
                {
                    handle_syst_command(state);
                }
                else if (strcmp(command, "NLST") == 0)
                {
                    if (state->logged_in)
                    {
                        handle_list_command(state, args);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else if (strcmp(command, "CDUP") == 0)
                {
                    if (state->logged_in)
                    {
                        handle_cdup_command(state);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else if (strcmp(command, "QUIT") == 0)
                {
                    if (state->logged_in)
                    {
                        handle_quit_command(state);
                    }
                    else
                    {
                        send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
                    }
                }
                else
                {
                    send_response(state->control_sock, "502 Command not implemented.\r\n");
                }

                // 在命令处理后检查是否需要退出
                if (state->quit_requested) {
                    LOG("Client requested quit\n");
                    break; // 退出 while(1) 循环
                }

            }
        }
        else if (bytes_received == 0) {
            LOG("Client disconnected\n");
            cleanup_ftp_state(state);
            return 0; // 客户端正常断开
        }
        else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 超时，继续循环检查
                continue;
            } else if (errno == EBADF) {
                LOG("Bad file descriptor detected\n");
                cleanup_ftp_state(state);
                return -5; // 文件描述符错误
            } else {
                LOG("recv error: %s\n", strerror(errno));
                cleanup_ftp_state(state);
                return -2; // 其他错误
            }
        }
    }

    cleanup_ftp_state(state);
    return 0;
}

// int handle_ftp_commands(FTPState *state)
// {
//     char buffer[FTP_BUFFER_SIZE];
//     ssize_t bytes_received;
//     update_last_activity(state);

//     for (;;)
//     {
//         // 使用非阻塞的 recv 来避免阻塞
//         while ((bytes_received = recv(state->control_sock, buffer, sizeof(buffer) - 1, 0)) > 0)
//         {
//             check_timeouts(state);         // 检查超时
            
//             buffer[bytes_received] = '\0'; // 确保字符串以 \0 结束
//             LOG("Received command: %s\r", buffer);

//             // 分割命令和参数
//             char *command = strtok(buffer, " \r\n");
//             char *args = strtok(NULL, "\r\n");

//             // 处理 FTP 命令
//             if (strcmp(command, "USER") == 0) //用户名
//             {
//                 handle_user_command(state, args);
//             }
//             else if (strcmp(command, "PASS") == 0)//密码
//             {
//                 handle_pass_command(state, args);
//             }
//             else if (strcmp(command, "SIZE") == 0)
//             {
//                 handle_size_command(state, args);
//             } 
//             else if (strcmp(command, "PASV") == 0)//主动传输
//             {
//                 handle_pasv_command(state);
//             }
//             else if (strcmp(command, "PORT") == 0)
//             {
//                 handle_port_command(state, args);
//             }
//             else if (strcmp(command, "LIST") == 0)
//             {
//                 if (state->logged_in)
//                 {
//                     handle_list_command(state, args);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else if (strcmp(command, "RETR") == 0)//下载文件
//             {
//                 if (state->logged_in)
//                 {
//                     handle_retr_command(state, args);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else if (strcmp(command, "STOR") == 0)//上传指令
//             {
//                 if (state->logged_in)
//                 {
//                     handle_stor_command(state, args);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else if (strcmp(command, "MGET") == 0)
//             {
//                 if (state->logged_in)
//                 {
//                     handle_mget_command(state, args);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else if (strcmp(command, "PWD") == 0)//列出目录
//             {
//                 if (state->logged_in)
//                 {
//                     handle_pwd_command(state);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else if (strcmp(command, "CWD") == 0)//cd导航到目标目录
//             {
//                 if (state->logged_in)
//                 {
//                     handle_cwd_command(state, args);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else if (strcmp(command, "TYPE") == 0)//决定文件格式
//             {
//                 handle_type_command(state, args);
//             }
//             else if (strcmp(command, "SYST") == 0)
//             {
//                 handle_syst_command(state);
//             }
//             else if (strcmp(command, "NLST") == 0)
//             {
//                 if (state->logged_in)
//                 {
//                     handle_list_command(state, args);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else if (strcmp(command, "CDUP") == 0)
//             {
//                 if (state->logged_in)
//                 {
//                     handle_cdup_command(state);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else if (strcmp(command, "QUIT") == 0)
//             {
//                 if (state->logged_in)
//                 {
//                     handle_quit_command(state);
//                 }
//                 else
//                 {
//                     send_response(state->control_sock, "530 Please login with USER and PASS.\r\n");
//                 }
//             }
//             else
//             {
//                 send_response(state->control_sock, "502 Command not implemented.\r\n");
//             }
//         }

//         if (bytes_received == 0)
//         {
//             close(state->control_sock);
//             close(state->data_sock);
//             return 0; // 客户端断开连接
//         }
//         else if (bytes_received < 0)
//         {
//             if (errno == EAGAIN || errno == EWOULDBLOCK)
//             {
//                 close(state->control_sock);
//                 close(state->data_sock);
//                 return -1; // 非阻塞错误
//             }
//             else
//             {
//                 close(state->control_sock);
//                 close(state->data_sock);
//                 return -2; // 其他错误
//             }
//         }
//     }
// }


