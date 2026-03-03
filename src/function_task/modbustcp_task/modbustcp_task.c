#define _GNU_SOURCE
#include "modbustcp_task.h"
#include "interface/log/log.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/setting/ip_setting.h"
#include "interface/bms/bms_analysis.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"


// modbus服务器信息
modbus_t *ctx = NULL;
modbus_mapping_t *g_mb_mapping = NULL;
unsigned char modbus_ip[16] = IP_ADDRESS;
const uint16_t REGISTERS_START_ADDRESS = 0x3000; // 寄存器起始地址
extern unsigned short g_ota_flag;
uint16_t *modbusBuff = NULL;
pthread_t NetConfig_TASKHandle = 0;
pthread_t NetConfig_TASKHandle_TEST = 0;
static int timeout_flag = 0;
extern pthread_mutex_t modbus_reg_mutex;

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

    // 使用新的组合函数，既设置临时IP又更新永久配置
    int ret = set_ip_address(MODBUS_ETH_NUM, modbus_ip);
    if (ret != 0) {
        LOG("[ModbusTcp] IP setting failed, but continued to attempt to create Modbus context\n");
    } else {
        LOG("[ModbusTcp] IIP setting successed\n");
    }
    
    ctx = modbus_new_tcp(modbus_ip, 502);//新建一个tcp服务端
    if (ctx == NULL) { LOG("ctx = NULL"); return NULL; }
    LOG("[ModbusTcp] ctx =%d ,modbus ip =%s \r\n", ctx, modbus_ip);

    // 创建寄存器映射，只创建保持寄存器
    g_mb_mapping = modbus_mapping_new_start_address(0, 0, 0, 0, REGISTERS_START_ADDRESS, REGISTERS_NB, 0, 0);
    if (g_mb_mapping == NULL){
        LOG("[ModbusTcp] Failed to allocate the mapping: %s \r\n", modbus_strerror(errno));
        modbusBuff = NULL;
        if (ctx != NULL) {
            modbus_free(ctx);
            ctx = NULL;
        }
        return NULL;
    }

    modbusBuff = g_mb_mapping->tab_registers;// 全局 外部在用

    for (i = 0; i < 2; i++)// 填充部分信息
    {
        g_mb_mapping->tab_registers[i] = 10;
    }
    g_mb_mapping->tab_registers[MDBUS_ADDR_PRODUCTION - REGISTERS_START_ADDRESS] = LOGO;       // 智充
    g_mb_mapping->tab_registers[MDBUS_ADDR_ECU_VERSION - REGISTERS_START_ADDRESS] = ECU_VERSION; // 版本号

    server_socket = modbus_tcp_listen(ctx, NB_CONNECTION);// 开启监听
    if (server_socket < 0)
    {
        LOG("[ModbusTcp] modbus_tcp_listen failed: %s\n", modbus_strerror(errno));
        modbusBuff = NULL;
        if (g_mb_mapping != NULL) {
            modbus_mapping_free(g_mb_mapping);
            g_mb_mapping = NULL;
        }
        if(ctx != NULL) {
            modbus_free(ctx);
            ctx = NULL;
        }
        return NULL;
    }

    FD_ZERO(&refset); //初始化集合为NULL
    FD_SET(server_socket, &refset); // 将服务器socket加入集合
    fdmax = server_socket;
    struct timeval timeout;

    for (;;)
    {
        timeout.tv_sec = 10; // 设置select超时时间为10秒
        timeout.tv_usec = 0;

        rdset = refset;// 复制参考集合到读集合    
        //一刀切，10内没有任何新连接或数据则关闭所有客户端
        int sel = select(fdmax + 1, &rdset, NULL, NULL, &timeout);
        if (sel < 0) {
            LOG("[ModbusTcp] select error: %s\n", strerror(errno));
            timeout_flag = 0;
            usleep(1000);
            continue;
        }
        else if(sel == 0)       
        {      
            timeout_flag = 1;// select超时处理
             // 清理所有客户端连接
            for (master_socket = 0; master_socket <= fdmax; master_socket++){
                if (FD_ISSET(master_socket, &refset) && master_socket != server_socket){ //判断当前fd是否为refset中的集合
                    LOG("[ModbusTcp] Closing connection on socket %d\n", master_socket);
                    close(master_socket); // 关闭服务器套接字
                    FD_CLR(master_socket, &refset);// 从集合中移除
                    if (master_socket == fdmax){ fdmax--;}  // 更新最大文件描述符
                }
            }
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
                        for (master_socket = 0; master_socket <= fdmax; master_socket++)
                        {
                            if (FD_ISSET(master_socket, &refset) && master_socket != server_socket)
                            {
                                // LOG("[ModbusTcp] Closing connection on socket %d\n", master_socket);
                                close(master_socket); // 关闭服务器套接字
                                FD_CLR(master_socket, &refset);
                                if (master_socket == fdmax)
                                {
                                    fdmax--;
                                }
                            }
                        }
                        usleep(1000);
                    }
                    else
                    {                     
                        FD_SET(newfd, &refset); // 成功接受连接,添加到监控总集合
                        if (newfd > fdmax)  // 更新最大fd
                        {
                            fdmax = newfd;// 更新最大fd
                        }
                        LOG("[ModbusTcp] New connection from %s:%d on socket %d \r\n", inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
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
                        LOG("[ModbusTcp] Connection closed or error on socket %d: %s\n", master_socket, modbus_strerror(errno));                       
                        close(master_socket); // 关闭连接
                        FD_CLR(master_socket, &refset); // 从集合移除

                        // 若该 socket 是当前最大值，更新 fdmax
                        if (master_socket == fdmax)
                        {
                            while (fdmax > 0 && !FD_ISSET(fdmax, &refset))
                            {
                                fdmax--;
                            }
                        }
                    }
                }
            }
        }
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
