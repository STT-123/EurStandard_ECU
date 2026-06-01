#define _GNU_SOURCE
// #include "C_OTAStateMonitor.h"
#include "xmodemstate.h"
#include "interface/modbus/modbus_defines.h"
#include "xmodemlisten.h"
#include "function_task/modbustcp_task/modbustcp_task.h"
#include "xmodemdata.h"
#include "interface/bms/bms_analysis.h"
#include "interface/log/log.h"

struct timespec AC_OTA_lastCheckTick = {0};

unsigned short sblfilenumber = 0xFFFF; // SBL文件数量大小
unsigned short appfilenumber = 0xFFFF; // app文件数量大小
int SBl_index = 0;
int APP_index = 0;

int otasock = -1;
int otasock1 = -1;

static int otafileret = -1; // 初始化为自定义值（-1 表示未初始化）

extern unsigned char XmodemSendCFlag;
extern volatile unsigned long prvmsgtimer;

unsigned char clientConnected = 0;
unsigned char XmodemServerReceiveSOH = 0;
unsigned char XmodemServerReceiveEOT = 0;
unsigned char XmodemServerReceiveFileEnd = 0;
unsigned char XmodemServerEnd = 0;

pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;

FILE *OTAfil = NULL;

unsigned int OsIf_GetMilliseconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec * 1000 + now.tv_nsec / 1000000);
}


void CloseXModemServer(void)
{
    pthread_t data_tid = 0;
    pthread_t listen_tid = 0;
    int join_data = 0;
    int join_listen = 0;

    pthread_mutex_lock(&task_mutex);
    xmodem_server_stopping = 1;
    if (LwIPTCPDataTaskRunning)
    {
        data_tid = LwIPTCPDataTaskHandle;
        LwIPTCPDataTaskRunning = 0;
        LwIPTCPDataTaskHandle = 0;
        join_data = 1;
    }
    if (LwIPTCPListenTaskRunning)
    {
        listen_tid = LwIPTCPListenTaskHandle;
        LwIPTCPListenTaskRunning = 0;
        LwIPTCPListenTaskHandle = 0;
        join_listen = 1;
    }
    pthread_mutex_unlock(&task_mutex);

    if (otasock1 >= 0)
    {
        shutdown(otasock1, SHUT_RDWR);
        close(otasock1);
        otasock1 = -1;
    }

    if (otasock >= 0)
    {
        shutdown(otasock, SHUT_RDWR);
        close(otasock);
        otasock = -1;
    }

    if (join_data)
    {
        pthread_cancel(data_tid);
        pthread_join(data_tid, NULL); // 等待线程回收资源
    }

    if (join_listen)
    {
        pthread_cancel(listen_tid);
        pthread_join(listen_tid, NULL); // 等待线程回收资源
    }

    if (OTAfil != NULL)
    {
        fclose(OTAfil);
        OTAfil = NULL;
    }

    prvmsgtimer = 0;
    curmsgtimer = 0;
    clientConnected = 0;
    XmodemSendCFlag = 0;
    otasock1 = -1;
    otasock = -1;
    XmodemServerReceiveSOH = 0;
    otafileret = 30;
    // XmodemServerEnd = 0;
    setXmodemServerEnd(0);
    pthread_mutex_lock(&task_mutex);
    xmodem_server_stopping = 0;
    pthread_mutex_unlock(&task_mutex);
}

signed char CheckXModemClient(void)
{
    if (clientConnected)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

unsigned char getClientConnected(void)
{
    return clientConnected;
}

void setClientConnected(unsigned char value)
{
    clientConnected = value;
}

unsigned char getXmodemSendCFlag(void)
{
    return XmodemSendCFlag;
}

void setXmodemSendCFlag(unsigned char value)
{
    XmodemSendCFlag = value;
}

unsigned char getXmodemServerReceiveSOH(void)
{
    return XmodemServerReceiveSOH;
}

void setXmodemServerReceiveSOH(unsigned char value)
{
    XmodemServerReceiveSOH = value;
}

unsigned char getXmodemServerReceiveEOT(void)
{
    return XmodemServerReceiveEOT;
}

void setXmodemServerReceiveEOT(unsigned char value)
{
    XmodemServerReceiveEOT = value;
}

unsigned char getXmodemServerReceiveFileEnd(void)
{
    return XmodemServerReceiveFileEnd;
}

void setXmodemServerReceiveFileEnd(unsigned char value)
{
    XmodemServerReceiveFileEnd = value;
}

unsigned char getXmodemServerEnd(void)
{
    return XmodemServerEnd;
}

void setXmodemServerEnd(unsigned char value)
{
    XmodemServerEnd = value;
}
