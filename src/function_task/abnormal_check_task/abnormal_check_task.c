#include "abnormal_check_task.h"
#include "device_drv/abncheck/abncheck.h"
#include "interface/bms/bms_analysis.h"
#include "interface/bms/bms_simulink/CANFDRcvFcn_BCU.h"
#include "interface/bms/bms_simulink/CANRcvFcn_BMU.h"
#include <pthread.h>
#include "interface/log/log.h"
#include "ip_setting.h"
pthread_t AnormalDetectionTask_TASKHandle = 0;


void *AbnormalDetection(void *arg)
{
    /**
     * 目前检测的故障只有、CAN0通道、SD卡
    */
   sleep(2);
    while (1)
    {
        /* code */
        check_bcu_rx_timeout();//CAN0 通道检测
        PHYlinktate(); //网口流量活动检测
        get_BCU_FaultInfo(get_BCU_FaultInfoLv4Value(),get_BCU_FaultInfoLv3Value(),get_BCU_FaultInfoLv2Value());
        ECUfault_process(); // 各种故障检测
        can_monitor_fun();//CAN 通道 通道检测
        check_and_fix_ip(MODBUS_ETH_NUM);//检测ip地址是否被修改并自动更正
        checkRootCapacity();//检测系统盘空间是否被耗尽
        sleep(1);
    }
}


void abnormalDetectionTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&AnormalDetectionTask_TASKHandle, NULL, AbnormalDetection, NULL);
        if (ret != 0)
        {
            LOG("[Check] Failed to create AnormalDetectionTask thread : %s", strerror(ret));
            sleep(1);
        }
        else
        {
            LOG("[Check] AnormaletectionTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}