#ifndef __C_FTPSERVER_HANDLE_H__
#define __C_FTPSERVER_HANDLE_H__


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <limits.h>
typedef struct {
	uint32_t last_activity;
    int control_sock;
    int data_sock;
    int logged_in;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    char path[255+8];
    char newpathname[128];
    FILE *file;
    uint8_t file_interrupt_flag;
    // 新增状态字段
    int timeout_pending;    // 待处理的超时
    int quit_requested;     // QUIT请求标记
} FTPState;

void init_ftp_state(FTPState *state) ;
static void handle_user_command(FTPState *state, char *args);
static void handle_pass_command(FTPState *state, char *args);
static void handle_pasv_command(FTPState *state);
static void handle_port_command(FTPState *state, char *args);
static void handle_list_command(FTPState *state, char *args);
static void handle_retr_command(FTPState *state, char *filename);
static void handle_stor_command(FTPState *state, char *filename);
static void handle_mget_command(FTPState *state, char *args);
static void handle_pwd_command(FTPState *state);
static void handle_syst_command(FTPState *state);
static void handle_cdup_command(FTPState *state);
static void handle_cwd_command(FTPState *state, const char *args);
static void handle_size_command(FTPState *state, char *filename);
static void handle_quit_command(FTPState *state);
static void handle_type_command(FTPState *state, char *args);

void set_ftp_read_file_flag(bool flag);
bool get_ftp_read_file_flag();
void send_response(int sock, const char *message);
int handle_ftp_commands(FTPState *state);
#endif
