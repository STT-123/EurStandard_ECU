#include "otaupgrad_task.h"
#include "device_drv/ota_upgrade/ota_other_update.h"
#include "device_drv/ota_upgrade/ota_xcp_update.h"
#include "device_drv/ota_upgrade/ota_uds_update.h"
#include "device_drv/ota_upgrade/ota_fun.h"
#include <pthread.h>
#include "interface/log/log.h"
#include "device_drv/ota_upgrade/ota_ecu_update.h"
#include "interface/bms/bms_analysis.h"
#include "device_drv/xmodem/xmodemstate.h"
#include "interface/modbus/modbus_defines.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/bmu_deal/bmu_deal.h"
pthread_t OTAUpgrad_TASKHandle = 0;
volatile unsigned int CurrentOTADeviceCanID = 0x1821FF10;
unsigned short g_ota_flag = 0;

void *ota_Upgrade_Task(void *arg)
{
    CurrentOTADeviceCanID = ACPOTACANID;
    unsigned char ECUOtaFlag = 0;
    unsigned char ACPOtaFlag = 0;
    unsigned char DCDCOtaFlag = 0;
    unsigned char ACOtaFlag = 0;
    unsigned char BCUOtaFlag = 0;
    unsigned char BMUOtaFlag = 0;
    unsigned char ReOtaFlag = 0;
    char matched_filename[256] = {0};

#if 0
    sleep(10);
    //BMU
    // set_ota_OTAFilename("XC_BMU_V302.tar");
    // set_ota_deviceType(BMU);
    // set_ota_deviceID(0x1821FF10) ;

    //BCU
    set_ota_OTAFilename("XC_BCU_V501.tar");
    set_ota_deviceType(BCU);
    set_ota_deviceID(BCUOTACANID) ;//BCU
    //ECU
    // set_ota_OTAFilename("XC_ECU_V123.tar");
    // set_ota_deviceType(ECU);
    // set_ota_deviceID(0) ;//ECU

    set_ota_OTAStart(1) ;
    LOG("[OTA] get_ota_OTAFilename() : %s\r\n",get_ota_OTAFilename());
    LOG("[OTA] get_ota_deviceID(): %x\r\n",get_ota_deviceID());
#endif
    while (1)
    {
        //获取ota标识
        get_modbus_reg_val(OTASTATUSREGADDR, &g_ota_flag);
        if(1 == get_ota_OTAStart())
        {
            set_modbus_reg_val(OTASTATUSREGADDR, OTASTARTRUNNING);//0124.升级状态
            if (get_ota_deviceType() == ECU)
            {
                LOG("[OTA] get_ota_deviceType(): %u\r\n", get_ota_deviceType());
                set_modbus_reg_val(OTAPPROGRESSREGADDR, 0); // 0124
                ECUOtaFlag = 0;
                while(ECUOtaFlag <3)
                {
                     set_ota_OTAStart(1);
                     ECU_OTA();
                     if(ecustatus.ErrorReg == 0)
                     {
                        LOG("[OTA] CAN ID 0x%x BCU OTA success!\r\n", get_ota_deviceID());
                        break;
                     }
                     else
                     {
                        ECUOtaFlag++;
                        LOG("[OTA] CAN ID 0x%x BCU OTA failed, retry count: %d\r\n", get_ota_deviceID(), ECUOtaFlag);
                     }

                }
                if(ECUOtaFlag >= 3){
                    set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                    sleep(5);//这个延时不能删除，不然上位机不显示升级失败，直接变为升级完成
                }
                FinshhECUOtaAndCleanup();
            }
            else if (get_ota_deviceType() == ACP ||get_ota_deviceType() == DCDC)
            {

                usleep(1000 * 1000);
                ACPDCDC_OTA();

                if (independentStatus.ErrorReg != 0 && get_ota_OTAStart() == 0)
                {
                    if (get_ota_deviceType() == ACP)
                    {
                        ACPOtaFlag++;
                        if (ACPOtaFlag < 3)
                        {

                            CurrentOTADeviceCanID = ACPOTACANID;
                            set_ota_deviceID(ACPOTACANID);
                            set_ota_OTAStart(1) ;
                            independentStatus.ErrorReg = 0;
                            LOG("[OTA] ACP OTA failed, error ACPOtaFlag count:  %d\r\n", ACPOtaFlag);
                            continue;
                        }

                        else
                        {
                            FinishACPOtaAndCleanup();
                            continue;
                        }
                    }
                    else if (get_ota_deviceType() == DCDC)
                    {
                        DCDCOtaFlag++;
                        if (DCDCOtaFlag < 3)
                        {

                            CurrentOTADeviceCanID = DCDCOTACANID;
                            set_ota_deviceID(DCDCOTACANID);
                            set_ota_OTAStart(1) ;
                            independentStatus.ErrorReg = 0;
                            LOG("[OTA] DCDC OTA failed, error ACPOtaFlag count:  %d\r\n", DCDCOtaFlag);
                            continue;
                        }

                        else
                        {
                            FinishDCDCOtaAndCleanup();
                            continue;
                        }
                    }
                }
                else if (independentStatus.DeviceProgramOkFlag)
                {
                    independentStatus.DeviceProgramOkFlag = 0; // 需要添加
                    LOG("[OTA] CAN ID 0x%x ACP OTA success!\r\n", get_ota_deviceID());
                    LOG("[OTA] CAN ID 0x%x ACP OTA success!\r\n", get_ota_deviceID());
                    if (get_ota_deviceType() == ACP)
                    {
                        FinishACPOtaAndCleanup();
                    }
                    else if (get_ota_deviceType() == DCDC)
                    {
                        FinishDCDCOtaAndCleanup();
                    }
                }
            }
            else if (get_ota_deviceType() == AC)
            {
                // usleep(1000*1000);
                UDS_OTA();
                if (udsstatus.ErrorReg != 0 && get_ota_OTAStart() == 0)
                {

                    ACOtaFlag++;
                    if (ACOtaFlag < 3)
                    {

                        CurrentOTADeviceCanID = ACOTACANID;
                        set_ota_deviceID(ACPOTACANID);
                        set_ota_OTAStart(1) ;
                        udsstatus.ErrorReg = 0;
                        LOG("[OTA] ACP OTA failed, error ACPOtaFlag count:  %d\r\n", ACOtaFlag);
                        continue;
                    }
                    else
                    {
                        set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                        FinishACOtaAndCleanup();
                        continue;
                    }
                }
                else if (udsstatus.DeviceProgramOkFlag)
                {
                    udsstatus.DeviceProgramOkFlag = 0; // 需要添加
                    LOG("[OTA] CAN ID 0x%x ACP OTA success!\r\n", get_ota_deviceID());
                    LOG("[OTA] CAN ID 0x%x ACP OTA success!\r\n", get_ota_deviceID());
                    if (get_ota_deviceType() == AC)
                    {
                        FinishACOtaAndCleanup();
                    }
                }
            }
            else if (get_ota_deviceType() == BCU || get_ota_deviceType() == BMU)
            {
                LOG("[OTA] BCU or BMU OTA start!\r\n");
                LOG("[OTA] get_ota_deviceType() ==  : %u\r\n", get_ota_deviceType());
                if (get_ota_deviceType() == BCU)//0x1cb0110e4
                {
                    for (unsigned int i = 0; i < 5; i++){
                        set_OTA_XCPConnect(255);//设置跳转到BOOT的条件,OTA_XCPConnect为0xFF才会跳转到BOOT
                        LOG("[OTA] set_OTA_XCPConnect\r\n");
                        CANFDSendFcn_BCU_step();
                        usleep(200*1000);
                    }                 

                    // 主业务判断：检查CAN2是否就绪
                    if (!is_bcu_can_ready()) {
                        LOG("[OTA] CAN2 not ready, waiting...\n");// 可以等待几秒或直接报错
                        int wait_count = 0;
                        while (!is_bcu_can_ready() && wait_count < 10) {
                            usleep(500000); // 500ms
                            can_monitor_fun();//检查CAN 状态
                            wait_count++;
                        }
                    }

                    BCUOtaFlag = 0;
                    if (is_bcu_can_ready())
                    {

                        restart_can_interface_enhanced(BCU_CAN_DEVICE_NAME);
                        sleep(2);
                        while(BCUOtaFlag < 3)
                        {
                            set_ota_OTAStart(1);
                            queue_clear(&Queue_BCURevData);//情况缓存消息队列

                            XCP_OTA(BCUOtaFlag);
                            if (xcpstatus.ErrorReg == 0)
                            {
                                LOG("[OTA] CAN ID 0x%x BCU OTA success!\r\n", get_ota_deviceID());
                                set_modbus_reg_val(OTAPPROGRESSREGADDR, 100);//0124,升级进度
                                set_modbus_reg_val(OTASTATUSREGADDR, OTASUCCESS);
                                break;
                            }
                            else
                            {
                                BCUOtaFlag++;
                                sleep(5);
                                LOG("[OTA] CAN ID 0x%x BCU OTA failed, retry count: %d\r\n", get_ota_deviceID(), BCUOtaFlag);
                            }
                        }
                        if(BCUOtaFlag >= 3){
                            set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                            sleep(5);//这个延时不能删除，不然上位机不显示升级失败，直接变为升级完成
                            LOG("[OTA] CAN ID 0x%x BCU OTA Failed \r\n");
                        }
                    }else{
                        LOG("[OTA] CAN2 is not ready\r\n");
                    }
                }
                else if (get_ota_deviceType() == BMU)
                {
                    unsigned int total_steps = BMUMAXNUM;  // 0到14共15次
                    unsigned int start_percent = 7;
                    unsigned int end_percent = 100;
                    
                    can_monitor_fun();//检查CAN 状态
                    // 主业务判断：检查CAN3是否就绪
                    if (!is_bmu_can_ready()) {
                        LOG("[OTA] CAN3 not ready, waiting...\n");
                        // 可以等待几秒或直接报错
                        int wait_count = 0;
                        while (!is_bmu_can_ready() && wait_count < 10) {
                            usleep(500000); // 500ms
                            wait_count++;
                        }
                    }
                    if (is_bmu_can_ready())
                    {
                        BMUOtaFlag = 0;
                        unsigned int percentage = 0;
                        restart_can_interface_enhanced(BMU_CAN_DEVICE_NAME);
                        sleep(2);
                        for (int i = 0; i < BMUMAXNUM; i++)//BMUMAXNUM
                        {
                            ReOtaFlag = 0;
                            LOG("[OTA] BMU OTA start! i:%d, ReOtaFlag:%d ,BMUOtaFlag: %d\r\n", i,ReOtaFlag,BMUOtaFlag);
                            while (ReOtaFlag < 3)
                            {
                                CurrentOTADeviceCanID = (0x1821D << 12) | ((i + 1) << 8) | 0x10;
                                set_ota_deviceID(CurrentOTADeviceCanID);
                                LOG("[OTA] Start OTA try %d, CAN ID 0x%x BMU %d\r\n", ReOtaFlag + 1, get_ota_deviceID());
                                LOG("[OTA] get_ota_deviceID() ==  : %x\r\n", get_ota_deviceID());                  
                                XCP_OTA(i+ReOtaFlag);

                                if (xcpstatus.ErrorReg == 0)
                                {
                                    LOG("[OTA] CAN ID 0x%x BMU OTA success!\r\n", get_ota_deviceID());
                                    break;
                                }
                                else
                                {
                                    ReOtaFlag++;
                                    BMUOtaFlag++;
                                    sleep(5);
                                    LOG("[OTA] CAN ID 0x%x BMU OTA failed, retry count: %d\r\n", get_ota_deviceID(), ReOtaFlag);
                                    continue;
                                }
                            }
                            if(xcpstatus.ErrorReg == 0)
                            {
                                sleep(2);
                                //这段代码是，一共15个BMU，每ota完一个增加7%的进度
                                percentage = start_percent + (end_percent - start_percent) * i / (total_steps - 1);
                                set_modbus_reg_val(OTAPPROGRESSREGADDR, percentage); // 0124, upgrade progress,BCU直接写升级进度，BMU 由于有15个，不在这里写进度
                                LOG("[OTA] STEP %2d: %3d%%\n", i, percentage);
                            }else{
                                LOG("[OTA] CAN ID 0x%x BMU OTA failed\r\n", get_ota_deviceID());
                            }       
                        }
                        if((percentage == 100) && (BMUOtaFlag < 3)){
                            LOG("[OTA] BMU OTA SUCCEDD\r\n");
                            set_modbus_reg_val(OTASTATUSREGADDR, OTASUCCESS);
                        }else{
                            set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                            sleep(5);//这个延时不能删除，不然上位机不显示升级失败，直接变为升级完成
                            LOG("[OTA] BMU failed number > 1,BMUOtaFlag = %d \r\n",BMUOtaFlag);

                        }
                    }else{
                        LOG("[OTA] CAN3 is not ready\r\n");
                    }                    
                }
                FinshhBCUBMUOtaAndCleanup();    
            }
        }
        usleep(10 * 1000);
    }
}
void ota_Upgrade_TaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&OTAUpgrad_TASKHandle, NULL, ota_Upgrade_Task, NULL);
        if (ret != 0)
        {
            LOG("[OTA] Failed to create SerialLedTask thread : %s", strerror(ret));
            sleep(1);
        }
        else
        {
            LOG("[OTA] SerialLedTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}

