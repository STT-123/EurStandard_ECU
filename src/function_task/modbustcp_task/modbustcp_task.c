#define _GNU_SOURCE
#include "modbustcp_task.h"
#include <errno.h>
#include <time.h>
#include "interface/log/log.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/setting/ip_setting.h"
#include "interface/bms/bms_analysis.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"
#include "device_drv/bcu_deal/bcu_deal.h"

// modbus服务器信息
modbus_t *ctx = NULL;
modbus_mapping_t *g_mb_mapping = NULL;
char modbus_ip[16] = IP_ADDRESS;
const uint16_t REGISTERS_START_ADDRESS = 0x3000; // 寄存器起始地址
extern unsigned short g_ota_flag;
uint16_t *modbusBuff = NULL;
pthread_t NetConfig_TASKHandle = 0;
pthread_t NetConfig_TASKHandle_TEST = 0;
static int timeout_flag = 0;
extern pthread_mutex_t modbus_reg_mutex;

#define CONN_RESET_LOG_WINDOW_SEC 5

static void close_socket_quickly(int socket_fd)
{
    struct linger linger_opt;

    if (socket_fd < 0) {
        return;
    }

    linger_opt.l_onoff = 1;
    linger_opt.l_linger = 0;
    setsockopt(socket_fd, SOL_SOCKET, SO_LINGER, &linger_opt, sizeof(linger_opt));
    shutdown(socket_fd, SHUT_RDWR);
    close(socket_fd);
}

static int count_connected_clients(const fd_set *refset, int server_socket, int fdmax)
{
    int socket_fd;
    int client_count = 0;

    if (refset == NULL) {
        return 0;
    }

    for (socket_fd = 0; socket_fd <= fdmax; socket_fd++) {
        if (FD_ISSET(socket_fd, refset) && socket_fd != server_socket) {
            client_count++;
        }
    }

    return client_count;
}

static void log_connected_clients(const fd_set *refset, int server_socket, int fdmax)
{
    int socket_fd;
    int client_count = 0;
    char socket_list[128] = {0};
    int offset = 0;

    if (refset == NULL) {
        LOG("[ModbusTcp] Current connected clients: NULL, sockets: [NULL]\r");
        return;
    }

    offset += snprintf(socket_list + offset, sizeof(socket_list) - offset, "[");

    for (socket_fd = 0; socket_fd <= fdmax; socket_fd++) {
        if (FD_ISSET(socket_fd, refset) && socket_fd != server_socket) {
            client_count++;
            offset += snprintf(socket_list + offset,
                               sizeof(socket_list) - offset,
                               "%s%d",
                               (client_count > 1) ? ", " : "",
                               socket_fd);
            if (offset >= (int)sizeof(socket_list)) {
                break;
            }
        }
    }

    if (offset < (int)sizeof(socket_list)) {
        snprintf(socket_list + offset, sizeof(socket_list) - offset, "]");
    } else {
        socket_list[sizeof(socket_list) - 2] = ']';
        socket_list[sizeof(socket_list) - 1] = '\0';
    }

    LOG("[ModbusTcp] Current connected clients: %d, sockets: %s",
        client_count, socket_list);
}

static void close_all_client_sockets(fd_set *refset, int server_socket, int *fdmax)
{
    int socket_fd;

    if (refset == NULL || fdmax == NULL) {
        return;
    }

    for (socket_fd = 0; socket_fd <= *fdmax; socket_fd++) {
        if (FD_ISSET(socket_fd, refset) && socket_fd != server_socket) {
            LOG("[ModbusTcp] Closing connection on socket %d\n", socket_fd);
            FD_CLR(socket_fd, refset);
            close_socket_quickly(socket_fd);
        }
    }

    while (*fdmax > 0 && !FD_ISSET(*fdmax, refset)) {
        (*fdmax)--;
    }

    log_connected_clients(refset, server_socket, *fdmax);
}

static void log_conn_reset_limited(int socket_fd)
{
    static time_t window_start = 0;
    static unsigned int suppressed_count = 0;
    time_t now = time(NULL);

    if (window_start == 0) {
        window_start = now;
    }

    if ((now - window_start) >= CONN_RESET_LOG_WINDOW_SEC) {
        if (suppressed_count > 0) {
            LOG("[ModbusTcp] Connection reset by peer repeated %u times in last %d seconds\n",
                suppressed_count, CONN_RESET_LOG_WINDOW_SEC);
            suppressed_count = 0;
        }
        window_start = now;
        LOG("[ModbusTcp] Connection closed or error on socket %d: Connection reset by peer\n", socket_fd);
        return;
    }

    if (suppressed_count == 0) {
        LOG("[ModbusTcp] Connection closed or error on socket %d: Connection reset by peer\n", socket_fd);
    } else {
        suppressed_count++;
        return;
    }

    suppressed_count = 1;
}

int get_timeout_flag(void)
{
    return timeout_flag;
}

void *ModbusTCPServerTask(void *arg)
{
    int rc;
    unsigned int i;
    fd_set refset;  //所有监听的socket）
    fd_set rdset;   //select检查的socket
    int fdmax;      // 最大文件描述符值
    int server_socket;  // 服务器监听socket
    int master_socket;  // 客户端连接socket

    if (g_ipsetting.flag == 1 && g_ipsetting.ip != 0)// 创建modbus服务端
    {
        sprintf(modbus_ip, "%d.%d.%d.%d", (g_ipsetting.ip >> 24) & 0xFF, (g_ipsetting.ip >> 16) & 0xFF, (g_ipsetting.ip >> 8) & 0xFF, g_ipsetting.ip & 0xFF);
    }

    for (;;)
    {
        struct timeval timeout;
        int need_restart = 0;

        // IP未真正生效时不启动监听，避免bind失败后线程直接退出
        while (check_and_fix_ip(MODBUS_ETH_NUM) != 0) {
            LOG("[ModbusTcp] IP is not ready, retry after 1s\n");
            sleep(1);
        }
        LOG("[ModbusTcp] IP setting successed\n");

        ctx = modbus_new_tcp(modbus_ip, 502);//新建一个tcp服务端
        if (ctx == NULL) {
            LOG("[ModbusTcp] modbus_new_tcp failed\n");
            sleep(1);
            continue;
        }
        LOG("[ModbusTcp] ctx =%p ,modbus ip =%s \r\n", (void *)ctx, modbus_ip);

        // 创建寄存器映射，只创建保持寄存器
        g_mb_mapping = modbus_mapping_new_start_address(0, 0, 0, 0, REGISTERS_START_ADDRESS, REGISTERS_NB, 0, 0);
        if (g_mb_mapping == NULL){
            LOG("[ModbusTcp] Failed to allocate the mapping: %s \r\n", modbus_strerror(errno));
            modbusBuff = NULL;
            if (ctx != NULL) {
                modbus_free(ctx);
                ctx = NULL;
            }
            sleep(1);
            continue;
        }

        modbusBuff = g_mb_mapping->tab_registers;// 全局 外部在用

        for (i = 0; i < 2; i++)// 填充部分信息
        {
            g_mb_mapping->tab_registers[i] = 10;
        }
        g_mb_mapping->tab_registers[MDBUS_ADDR_PRODUCTION - REGISTERS_START_ADDRESS] = LOGO;       // 智充
        g_mb_mapping->tab_registers[MDBUS_ADDR_ECU_VERSION - REGISTERS_START_ADDRESS] = ECU_VERSION; // 版本号
        //测试使用，由于测试过程中BCU和BMU的CAN接反了，所以出现过两个硬件状态，所以软件要改动一下，稳定出场之后就之有一个硬件CAN状态了
        if(BCU_CAN_DEVICE_NAME == "can2"){
            g_mb_mapping->tab_registers[MDBUS_ADDR_ECU_VERSION - REGISTERS_START_ADDRESS] = (ECU_VERSION + 0x100); // 版本号
        }
        server_socket = modbus_tcp_listen(ctx, NB_CONNECTION);// 开启监听
        if (server_socket < 0)
        {
            LOG("[ModbusTcp] modbus_tcp_listen failed: %s, retry after 1s\n", modbus_strerror(errno));
            modbusBuff = NULL;
            if (g_mb_mapping != NULL) {
                modbus_mapping_free(g_mb_mapping);
                g_mb_mapping = NULL;
            }
            if(ctx != NULL) {
                modbus_free(ctx);
                ctx = NULL;
            }
            sleep(1);
            continue;
        }
        FD_ZERO(&refset); //初始化集合为NULL
        FD_SET(server_socket, &refset); // 将服务器socket加入集合
        fdmax = server_socket;

        while (!need_restart)
        {
            timeout.tv_sec = 10; // 设置select超时时间为10秒
            timeout.tv_usec = 0;

            rdset = refset;// 复制参考集合到读集合
            //一刀切，10内没有任何新连接或数据则关闭所有客户端
            int sel = select(fdmax + 1, &rdset, NULL, NULL, &timeout);
            if (sel < 0) {
                LOG("[ModbusTcp] select error: %s\n", strerror(errno));
                timeout_flag = 0;
                need_restart = 1;
                break;
            }
            else if(sel == 0)
            {
                timeout_flag = 1;// select超时处理
                LOG("[ModbusTcp] select timeout for 10s, closing all client sockets\n");
                close_all_client_sockets(&refset, server_socket, &fdmax);
            }
            else{
                timeout_flag = 0;
            }

            // 遍历所有可能的socket
            for (master_socket = 0; master_socket <= fdmax; master_socket++)
            {
                if (FD_ISSET(master_socket, &rdset))// 检查socket是否就绪
                {
                    if (master_socket == server_socket) // 服务器进入就绪状态，有客户端要连接
                    {
                        socklen_t addrlen;// 处理新连接请求
                        struct sockaddr_in clientaddr;
                        int newfd;

                        addrlen = sizeof(clientaddr);
                        memset(&clientaddr, 0, sizeof(clientaddr));
                        // 接受新连接
                        newfd = accept(server_socket, (struct sockaddr *)&clientaddr, &addrlen); //任意客户端连接出错，客户端都关了
                        if (newfd == -1)
                        {
                            LOG("[ModbusTcp] Server accept() error: %s\n", strerror(errno));  // 出错时清理所有连接
                            close_all_client_sockets(&refset, server_socket, &fdmax);
                            need_restart = 1;
                            break;
                        }
                        else
                        {
                            FD_SET(newfd, &refset); // 成功接受连接,添加到监控总集合
                            if (newfd > fdmax)  // 更新最大fd
                            {
                                fdmax = newfd;// 更新最大fd
                            }
                            LOG("[ModbusTcp] New connection from %s:%d on socket %d, current clients: %d\r\n",
                                inet_ntoa(clientaddr.sin_addr),
                                clientaddr.sin_port,
                                newfd,
                                count_connected_clients(&refset, server_socket, fdmax));
                                log_connected_clients(&refset, server_socket, fdmax);
                        }
                    }
                    else
                    {
                        // 处理已连接客户端的请求
                        unsigned char query[MODBUS_TCP_MAX_ADU_LENGTH];
                        modbus_set_socket(ctx, master_socket); // 设置当前socket到modbus上下文

                        rc = modbus_receive(ctx, query);// 接收Modbus请求
                        if (rc != -1)
                        {
                            modbus_write_reg_deal(ctx, query, rc); // 写寄存器处理
                            pthread_mutex_lock(&modbus_reg_mutex);
                            modbus_reply(ctx, query, rc, g_mb_mapping); // 回复寄存器
                            pthread_mutex_unlock(&modbus_reg_mutex);
                        }
                        else
                        {
                            if (errno == ECONNRESET) {
                                log_conn_reset_limited(master_socket);
                            } else {
                                LOG("[ModbusTcp] Connection closed or error on socket %d: %s\n", master_socket, modbus_strerror(errno));
                            }
                            close_socket_quickly(master_socket); // 关闭连接
                            FD_CLR(master_socket, &refset); // 从集合移除

                            // 若该 socket 是当前最大值，更新 fdmax
                            if (master_socket == fdmax)
                            {
                                while (fdmax > 0 && !FD_ISSET(fdmax, &refset))
                                {
                                    fdmax--;
                                }
                            }
                            log_connected_clients(&refset, server_socket, fdmax);
                        }
                    }
                }
            }
        }

        for (master_socket = 0; master_socket <= fdmax; master_socket++) {
            if (FD_ISSET(master_socket, &refset)) {
                close_socket_quickly(master_socket);
            }
        }
        modbusBuff = NULL;
        if (g_mb_mapping != NULL) {
            modbus_mapping_free(g_mb_mapping);
            g_mb_mapping = NULL;
        }
        if (ctx != NULL) {
            modbus_free(ctx);
            ctx = NULL;
        }
        sleep(1);
    }
}


/**
 * reamdme:
 * 本C文件包含ModusTCP连接人物和OTA文件升级处理任务
 * ModbusTCPServer：新建ModusTCP连接，监听读取寄存器和写寄存器的功能，要是ota则由TcpServerExample处理
 * TcpServerExample：OTA文件升级处理任务
*/
void modbusTcpServerTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&NetConfig_TASKHandle, NULL, ModbusTCPServerTask, NULL);
        if (ret != 0){
            LOG("[ModbusTcp] Failed to create NETConfigTask thread : %s", strerror(ret));
            sleep(1);
        }else{
            LOG("[ModbusTcp] NETConfigTask thread created successfully.\r\n");
        }
    } while (ret != 0); 
}
