#include "ftp_task.h"
#include "ftp_protocol.h"
#include "interface/log/log.h"
#define FTP_VERSION "FTP Server 1.0.0"

int ftp_port = 21;
pthread_t ftp_service_thread_id;

static void *ftp_service_thread_func(void *arg)
{
    int port = *(int *)arg;
    int server_sock;
    struct sockaddr_in server_addr;
    int opt = 1;
    struct timeval timeout;

    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0){
        LOG("[FTP] Failed to create socket\n");
        return NULL;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    // Set socket options
    if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0){
        close(server_sock);
        return NULL;
    }

    // Bind socket to address
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){
        LOG("[FTP] Failed to bind socket\n");
        close(server_sock);
        return NULL;
    }

    // Start listening for incoming connections
    if (listen(server_sock, 2) < 0){
        close(server_sock);
        return NULL; 
    }

    while (1)
    {
        FTPState state = {0};
        init_ftp_state(&state);
        // Accept client connections
        state.client_addr_len = sizeof(state.client_addr);
        state.control_sock = accept(server_sock, (struct sockaddr *)&state.client_addr, &state.client_addr_len);
        if (state.control_sock < 0)
        {
            LOG("[FTP] Failed to accept connection\n");
            sleep(1);
            continue;
        }

        // Set receive timeout for control socket
        timeout.tv_sec = 60;
        timeout.tv_usec = 0;
        if (setsockopt(state.control_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
        {
            close(state.control_sock);
            sleep(1);
            continue;
        }

        // Initialize state
        state.logged_in = 0;
        state.data_sock = -1;

        // Send greeting message
        char response[256];
        snprintf(response, sizeof(response), "220 %s ready.\r\n", FTP_VERSION);
        send_response(state.control_sock, response);

        int result = handle_ftp_commands(&state);
        // 主线程只需要处理连接级别的清理
        if (state.control_sock >= 0) {
            close(state.control_sock);
            state.control_sock = -1;
        }
    }

    close(server_sock);
}

void FtpServiceThreadCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&ftp_service_thread_id, NULL, ftp_service_thread_func, &ftp_port);
        if (ret != 0)
        {
            LOG("[FTP] Create ftp_service_thread_func Failed, [%s]. \n", strerror(ret));
            sleep(1);
        }
    } while (ret != 0);
}