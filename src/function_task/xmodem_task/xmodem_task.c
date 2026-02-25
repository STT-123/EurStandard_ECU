#include "xmodem_task.h"
#include "function_task/modbustcp_task/modbustcp_task.h"
#include "interface/log/log.h"
#include "device_drv/xmodem/xmodemstate.h"
#include "interface/bms/bms_analysis.h"
#include "interface/modbus/modbus_defines.h"
#include "device_drv/xmodem/xmodemlisten.h"
#include "device_drv/xmodem/xmodemdata.h"
#include "device_drv/ota_upgrade/ota_fun.h"


// #include "Net_Config_Task.h"
pthread_t TcpServerExample_TASKHandle = 0;
unsigned char XmodemSendCFlag = 0;
volatile unsigned long prvmsgtimer = 0;
pthread_t *pLwIPTCPDataTaskHandle = NULL;
pthread_t *pLwIPTCPListenTaskHandle = NULL;

void *XmodemCommTask(void *arg)
{
    unsigned int startTime = 0;
    unsigned short curotaCtrregval = 0xFFFF;
    unsigned short prvotaCtrregval = 0xFFFF;
    static unsigned char step = 0;
    static unsigned char times = 0;
    unsigned short times1 = 0;

    LOG("[Xmodem] TcpServerExample task!\n");
    signal(SIGPIPE, SIG_IGN);
    step = 1;
    while (1)
    {
        if (get_ota_UpDating() == 1)
        {
            set_TCU_PowerUpCmd(BMS_POWER_UPDATING);//打印太频繁
        }
        if (g_otactrl.OTAStart == 0 && (sblfilenumber != 0) && ((GetTimeDifference_ms(AC_OTA_lastCheckTick)) >= 30000))
        {
            SBl_index = 0;
            APP_index = 0;
        }

        int ret = get_modbus_reg_val(OTACTRLREGADDR, &curotaCtrregval);
        ret = get_modbus_reg_val(AC_SBL_OTAFILENUMBER, &sblfilenumber);
        ret = get_modbus_reg_val(AC_APP_OTAFILENUMBER, &appfilenumber);
        //  LOG("[Xmodem] curotaCtrregval %d sblfilenumber %d appfilenumber %d\n", curotaCtrregval, sblfilenumber, appfilenumber);

        if (ret < 0)
        {
            //LOG("[Xmodem] ret %d get modbus reg val error\n", ret);
        }
        else
        {
            if ((curotaCtrregval != prvotaCtrregval) && (get_ota_OTAStart() == 0))
            {
                LOG("[Xmodem] ota curotaCtrregval 0x%x prvotaCtrregval 0x%x\n", curotaCtrregval, prvotaCtrregval);

                if (curotaCtrregval == 0x0000)
                {
                    set_modbus_reg_val(OTASTATUSREGADDR, 0);
                    set_modbus_reg_val(OTAPPROGRESSREGADDR, 0);
                    CloseXModemServer();
                    setXmodemServerEnd(0);
                    step = 2;
                }

                if (curotaCtrregval == 0x0001)//上位机发送升级文件，curotaCtrregval值会被改变
                {
                    // 临界区保护：确保任务创建不会出现竞态条件
                    LOG("[Xmodem] TcpServerExample task malloc!\n");
                    pthread_mutex_lock(&task_mutex); // 锁住临界区

                    if (pLwIPTCPListenTaskHandle == NULL)
                    {
                        pLwIPTCPListenTaskHandle = malloc(sizeof(pthread_t));
                        pLwIPTCPDataTaskHandle = malloc(sizeof(pthread_t));

                        if (pLwIPTCPListenTaskHandle == NULL || pLwIPTCPDataTaskHandle == NULL)
                        {
                            LOG("[Xmodem] TcpServerExample task malloc failed!\n");
                            while (1)
                            {
                                usleep(1000 * 10000);
                            }
                        }

                        *pLwIPTCPDataTaskHandle = NULL;
                        *pLwIPTCPListenTaskHandle = NULL;
                        // LOG("[Xmodem] TcpServerExample task malloc suceess!\n");
                        // 创建监听任务
                        int ret = pthread_create(pLwIPTCPListenTaskHandle, NULL, Lwip_Listen_TASK, NULL);
                        if (ret != 0)
                        {
                            LOG("[Xmodem] Failed to create Lwip_Listen xTaskCreate! thread : %s", strerror(ret));
                            sleep(1);
                        }
                        else
                        {
                            LOG("[Xmodem] success to create Lwip_Listen xTaskCreate! thread");
                        }

                        if (ret == 0)
                        {
                            set_modbus_reg_val(OTASTATUSREGADDR, 1);
                        }
                    }

                    pthread_mutex_unlock(&task_mutex); // 解锁临界区
                    startTime = OsIf_GetMilliseconds();
                    while (1)
                    {
                        unsigned int currentTime = OsIf_GetMilliseconds();
                        ret = CheckXModemClient();
                        unsigned int time = currentTime - startTime;

                        if (ret == 0)
                        {
                            while (1)
                            {
                                char c = 'C';
                                LOG("[Xmodem] write tcp message C to client! otasock1:  %d!\n", otasock1);
                                signed char ret = write(otasock1, &c, 1);
                                LOG("[Xmodem] write tcp message C to client ret %d!\n", ret);
                                LOG("[Xmodem] otasock1 %d!\n", otasock1);

                                if (ret == 1)
                                {
                                    LOG("[Xmodem] write tcp message C to client!\n");
                                    XmodemSendCFlag = 1;
                                }

                                unsigned int RvTime = OsIf_GetMilliseconds();
                                while (1)
                                {
                                    unsigned int RvPassTime = OsIf_GetMilliseconds() - RvTime;

                                    if (getXmodemServerReceiveSOH())
                                    {
                                        LOG("[Xmodem] Received SOH!\n");
                                        break;
                                    }

                                    if (RvPassTime >= 2000)
                                    {
                                        LOG("[Xmodem] Received SOH timeout!\n");
                                        times++;
                                        LOG("[Xmodem] times %d!\n", times);
                                        break;
                                    }

                                    usleep(100000); // 100ms
                                }
                                LOG("[Xmodem] getXmodemServerReceiveSOH()  = %d!\n", getXmodemServerReceiveSOH());
                                if (getXmodemServerReceiveSOH())
                                {
                                    times = 0;
                                    LOG("[Xmodem] Received SOH!\n");
                                    break;
                                }

                                if (times >= 30)
                                {
                                    times = 0;
                                    set_ota_UpDating(0);
                                    LOG("[Xmodem] Write C 30 times over time!");
                                    break;
                                }
                            }

                            break;
                        }
                        else
                        {
                            if (time >= 60000)
                            {
                                setXmodemServerEnd(1);
                                set_modbus_reg_val(OTASTATUSREGADDR, 0);
                                set_modbus_reg_val(OTAPPROGRESSREGADDR, 0);
                                LOG("[Xmodem] Without client connect over 60s, close xmodem tcp server!\n");
                                LOG("[Xmodem] Without client connect over 60s, close xmodem tcp server!");
                                break;
                            }
                        }

                        usleep(100 * 1000); // 100ms
                    }
                }

                if (prvotaCtrregval == 0x0001 && curotaCtrregval == 0x0008)
                {
                    // 暂未用到
                }

                if (prvotaCtrregval == 0x0008 && curotaCtrregval == 0x0080)
                {
                    set_modbus_reg_val(OTASTATUSREGADDR, OTASTARTRUNNING);
                    set_ota_OTAStart(1);
                }

                // 0x0000 -> !0x0001
                if (prvotaCtrregval == 0 && step == 2 && curotaCtrregval != 0x0001)//检查curotaCtrregval没有按顺序变化
                {
                    setXmodemServerEnd(1);
                    set_modbus_reg_val(OTASTATUSREGADDR, 0);
                    set_modbus_reg_val(OTAPPROGRESSREGADDR, 0);
                    LOG("[Xmodem] prvotaCtrregval == 0 && step == 2 && curotaCtrregval != 0x0001\n");
                }
            }
            if (getXmodemServerEnd() == 1)
            {
                LOG("[Xmodem] XmodemServerEnd\n");
                CloseXModemServer();
                setXmodemServerEnd(0);
            }

            prvotaCtrregval = curotaCtrregval;
        }

        if (curotaCtrregval == 0x0001)
        {
            if (prvmsgtimer == curmsgtimer)
            {
                times1++;
            }
            else
            {
                times1 = 0;
            }

            prvmsgtimer = curmsgtimer;

            if (times1 >= 400)
            {
                if (get_timeout_flag() == 1)
                {
                    set_ota_UpDating(0);
                    set_modbus_reg_val(OTASTATUSREGADDR, 0);
                }
                prvmsgtimer = curmsgtimer = 0;
                CloseXModemServer();
                // XmodemServerEnd = 0;
                setXmodemServerEnd(0);
            }
        }

        usleep(10 * 1000); // 10ms
    }
}


void xmodemCommTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&TcpServerExample_TASKHandle, NULL, XmodemCommTask, NULL);
        if (ret != 0){
            LOG("[Xmodem] Failed to create TcpServerExampleTask thread : %s", strerror(ret));
            sleep(1);
        } else{
            LOG("[Xmodem] TcpServerExampleTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}