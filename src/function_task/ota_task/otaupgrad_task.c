#include "otaupgrad_task.h"
#include "device_drv/ota_upgrade/ota_other_update.h"
#include "device_drv/ota_upgrade/ota_xcp_update.h"
#include "device_drv/ota_upgrade/ota_uds_update.h"
#include "sd_store.h"
#include "device_drv/ota_upgrade/ota_fun.h"
#include <pthread.h>
#include "interface/log/log.h"
#include "device_drv/ota_upgrade/ota_ecu_update.h"
#include "interface/bms/bms_analysis.h"
#include "device_drv/xmodem/xmodemstate.h"
#include "interface/modbus/modbus_defines.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/bmu_deal/bmu_deal.h"
#include "device_drv/abncheck/abncheck.h"
#include "interface/bms/bms_simulink/CANFDSendFcn_BCU.h"
#include <ctype.h>
pthread_t OTAUpgrad_TASKHandle = 0;
volatile unsigned int CurrentOTADeviceCanID = 0x1821FF10;
unsigned short g_ota_flag = 0;
static int parse_bcu_target_version_h(const char *filename)
{
    const char *ver = NULL;

    if (filename == NULL) {
        return -1;
    }
    if (strstr(filename, "BCU") == NULL) {
        return -1;
    }

    ver = strchr(filename, 'V');
    if ((ver == NULL) || !isdigit((unsigned char)*(ver + 1))) {
        return -1;
    }

    return *(ver + 1) - '0';
}

static int has_file_extension(const char *filename, const char *ext)
{
    size_t filename_len;
    size_t ext_len;

    if (filename == NULL || ext == NULL) {
        return 0;
    }

    filename_len = strlen(filename);
    ext_len = strlen(ext);
    if (filename_len < ext_len) {
        return 0;
    }

    return strcmp(filename + filename_len - ext_len, ext) == 0;
}

static int validate_bcu_ota_version_before_boot(void)
{
    const char *filename = get_ota_OTAFilename();
    int ota_ver_h = parse_bcu_target_version_h(filename);
    int bcu_ver_h = get_BCU_Version_H();

    if (ota_ver_h < 0) {
        LOG("[OTA] Invalid BCU OTA file name, cannot parse version before boot: %s\r\n",
            filename ? filename : "(null)");
        xcpstatus.ErrorReg = OTA_ERR_VERSION_MISMATCH;
        xcpstatus.ErrorDeviceID = get_ota_deviceID();
        return -1;
    }

    if (bcu_ver_h == 0) {
        LOG("[OTA] BCU version high byte is 0, treat as unknown and allow OTA. file=%s, ota_ver_h=%d\r\n",
            filename, ota_ver_h);
        return 0;
    }

    if (ota_ver_h != bcu_ver_h) {
        LOG("[OTA] BCU OTA version mismatch before boot. file=%s, ota_ver_h=%d, bcu_ver_h=%d\r\n",
            filename, ota_ver_h, bcu_ver_h);
        xcpstatus.ErrorReg = OTA_ERR_VERSION_MISMATCH;
        xcpstatus.ErrorDeviceID = get_ota_deviceID();
        return -1;
    }

    LOG("[OTA] BCU OTA version check passed before boot. file=%s, ota_ver_h=%d, bcu_ver_h=%d\r\n",
        filename, ota_ver_h, bcu_ver_h);
    return 0;
}

static int precheck_bcu_ota_package_before_boot(void)
{
    const char *filename = get_ota_OTAFilename();
    char source_file[512] = {'\0'};
    int ret;

    memset(&xcpstatus, 0, sizeof(xcpstatus));

    if (!ota_filename_is_safe(filename)) {
        xcpstatus.ErrorReg = OTA_ERR_SOURCE_PACKAGE_MISSING;
        xcpstatus.ErrorDeviceID = get_ota_deviceID();
        LOG("[OTA] Invalid BCU OTA file name before boot precheck: %s\r\n",
            filename ? filename : "(null)");
        return -1;
    }

    if (has_file_extension(filename, ".bin")) {
        char ready_file[512] = {'\0'};

        snprintf(source_file, sizeof(source_file), "%s/%s", OTA_INCOMING_PATH, filename);
        if (access(source_file, F_OK) != 0) {
            xcpstatus.ErrorReg = OTA_ERR_SOURCE_PACKAGE_MISSING;
            xcpstatus.ErrorDeviceID = get_ota_deviceID();
            LOG("[OTA] Direct BCU bin does not exist before boot precheck: %s\r\n", source_file);
            return -1;
        }

        snprintf(ready_file, sizeof(ready_file), "%s/%s", OTA_READY_PATH, filename);
        if (ensure_ota_storage_dirs() != 0 || !copy_file(source_file, ready_file)) {
            xcpstatus.ErrorReg = OTA_ERR_COPY_TARGET_FAILED;
            xcpstatus.ErrorDeviceID = get_ota_deviceID();
            LOG("[OTA] Failed to stage direct BCU bin: %s -> %s\r\n", source_file, ready_file);
            return -1;
        }

        memset(&g_max_upgrade, 0, sizeof(g_max_upgrade));
        strncpy(g_max_upgrade.upgrade_file, filename, sizeof(g_max_upgrade.upgrade_file) - 1);
        LOG("[OTA] Direct BCU bin precheck mode: %s\r\n", source_file);
    } else {
        ret = unzipfile(OTA_READY_PATH, (unsigned int *)&xcpstatus.ErrorReg, FILE_TYPE_BIN);
        if (ret < 0) {
            LOG("[OTA] BCU OTA package precheck failed before boot. file=%s, ErrorReg=0x%x\r\n",
                filename, xcpstatus.ErrorReg);
            xcpstatus.ErrorDeviceID = get_ota_deviceID();
            return -1;
        }
    }

    if (validate_bcu_ota_version_before_boot() != 0) {
        LOG("[OTA] BCU OTA version precheck failed before boot. file=%s, ErrorReg=0x%x\r\n",
            filename, xcpstatus.ErrorReg);
        return -1;
    }

    LOG("[OTA] BCU OTA package precheck passed before boot. file=%s, upgrade_file=%s\r\n",
        filename, g_max_upgrade.upgrade_file);
    return 0;
}

void *ota_Upgrade_Task(void *arg)
{
    if (ensure_ota_storage_dirs() != 0) {
        LOG("[OTA] Local OTA storage initialization failed: %s\r\n", OTA_ROOT_PATH);
    }

    CurrentOTADeviceCanID = ACPOTACANID;
    unsigned char ECUOtaFlag = 0;
    unsigned char ACPOtaFlag = 0;
    unsigned char DCDCOtaFlag = 0;
    unsigned char ACOtaFlag = 0;
    unsigned char BCUOtaFlag = 0;
    unsigned char BMUOtaFlag = 0;
    unsigned char ReOtaFlag = 0;
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
                        LOG("[OTA] CAN ID 0x%x ECU OTA success!\r\n", get_ota_deviceID());
                        break;
                     }
                     else
                     {
                        ECUOtaFlag++;
                        LOG("[OTA] ecustatus.ErrorReg  = %d\r\n", ecustatus.ErrorReg);
                        LOG("[OTA] CAN ID 0x%x ECU OTA failed, retry count: %d\r\n", get_ota_deviceID(), ECUOtaFlag);
                     }

                }
                if(ECUOtaFlag >= 3){
                    set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                    sleep(5);//这个延时不能删除，不然上位机不显示升级失败，直接变为升级完成
                }
                int ecu_ota_success = (ecustatus.ErrorReg == 0);
                FinshhECUOtaAndCleanup();
                if (ecu_ota_success)
                {
                    system("sync"); // 确保OTA目标文件和临时目录清理都已落盘
                    LOG("[OTA] ECU OTA cleanup completed, rebooting system...\n");
                    system("reboot");
                }
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
                        set_ota_deviceID(ACOTACANID);
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
						LOG("[OTA] Hold AC OTA success status and 100%% progress for 5 seconds.\r\n");
						sleep(5);
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
                    if (precheck_bcu_ota_package_before_boot() != 0)
                    {
                        set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                        LOG("[OTA] Stop BCU OTA before boot jump because package precheck failed. file=%s, ErrorReg=0x%x\r\n",
                            get_ota_OTAFilename(), xcpstatus.ErrorReg);
                        sleep(5);
                        FinshhBCUBMUOtaAndCleanup();
                        continue;
                    }

                    for (unsigned int i = 0; i < 5; i++){
                        set_OTA_XCPConnect(255);//设置跳转到BOOT的条件,OTA_XCPConnect为0xFF才会跳转到BOOT
                        LOG("[OTA] set_OTA_XCPConnect\r\n");
                        CANFDSendFcn_BCU_step();
                        usleep(200*1000);
                    }                 
                    set_OTA_XCPConnect(0);//不能删***不然会触发OTA成功之后再次进入OTA导致失败的情况
                    // 主业务判断：检查BCU是否就绪
                    if (!is_bcu_can_ready()) {
                        LOG("[OTA] BCU not ready, waiting...\n");// 可以等待几秒或直接报错
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
                        while(BCUOtaFlag < 5)
                        {
                            set_ota_OTAStart(1);
                            queue_clear(&Queue_BCURevData);//情况缓存消息队列

                            XCP_OTA(BCUOtaFlag);
                            if (xcpstatus.ErrorReg == 0)
                            {
                                LOG("[OTA] CAN ID 0x%x BCU OTA success!\r\n", get_ota_deviceID());
                                set_modbus_reg_val(OTAPPROGRESSREGADDR, 100);//0124,升级进度
                                set_modbus_reg_val(OTASTATUSREGADDR, OTASUCCESS);
                                sleep(5); // 保持成功状态，确保上位机能够读到
                                break;
                            }
                            else
                            {
                                BCUOtaFlag++;
                                sleep(5);
                                LOG("[OTA] CAN ID 0x%x BCU OTA failed, retry count: %d\r\n", get_ota_deviceID(), BCUOtaFlag);
                            }
                        }
                        if(BCUOtaFlag >= 5){
                            set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                            sleep(5);//这个延时不能删除，不然上位机不显示升级失败，直接变为升级完成
                            LOG("[OTA] xcpstatus.ErrorReg  = %d\r\n", xcpstatus.ErrorReg);
                            LOG("[OTA] CAN ID  0x%x BCU OTA Failed \r\n", get_ota_deviceID());
                        }
                    }else{
                        LOG("[OTA] bcu is not ready\r\n");
                    }
                }
                else if (get_ota_deviceType() == BMU)
                {
                    unsigned int total_steps = BMUMAXNUM;  // 0到14共15次
                    unsigned int start_percent = 7;
                    unsigned int end_percent = 100;
                    
                    can_monitor_fun();//检查CAN 状态
                    // 主业务判断：检查bmu是否就绪
                    if (!is_bmu_can_ready()) {
                        LOG("[OTA] bmu not ready, waiting...\n");
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
                            while (ReOtaFlag < 5)
                            {
                                CurrentOTADeviceCanID = (0x1821D << 12) | ((i + 1) << 8) | 0x10;
                                set_ota_deviceID(CurrentOTADeviceCanID);
                                LOG("[OTA] Start OTA try %d, CAN ID 0x%x BMU %d\r\n",
                                    ReOtaFlag + 1, get_ota_deviceID(), i + 1);
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
                                    LOG("[OTA] xcpstatus.ErrorReg  = %d\r\n", xcpstatus.ErrorReg);
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
                        if((percentage == 100) && (BMUOtaFlag < 5)){
                            LOG("[OTA] BMU OTA SUCCEDD\r\n");
                            set_modbus_reg_val(OTASTATUSREGADDR, OTASUCCESS);
                        }else{
                            set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                            sleep(5);//这个延时不能删除，不然上位机不显示升级失败，直接变为升级完成
                            LOG("[OTA] BMU failed number > 1,BMUOtaFlag = %d \r\n",BMUOtaFlag);

                        }
                    }else{
                        LOG("[OTA] bmu is not ready\r\n");
                    }                    
                }
                FinshhBCUBMUOtaAndCleanup();    
            }
            else
            {
                LOG("[OTA] Invalid OTA context: OTAStart=%d, deviceType=%u, deviceID=0x%x, file=%s\r\n",
                    get_ota_OTAStart(), get_ota_deviceType(), get_ota_deviceID(),
                    get_ota_OTAFilename());
                LOG("[OTA] Abort invalid OTA state to avoid endless OTASTARTRUNNING loop.\r\n");
                set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
                set_ota_OTAStart(0);
                set_ota_UpDating(0);
                sleep(5);
                set_modbus_reg_val(OTASTATUSREGADDR, OTAIDLE);
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
