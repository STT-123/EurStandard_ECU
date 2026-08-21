#include "ota_ecu_update.h"
#include "interface/log/log.h"
#include "interface/ini/ini.h"
#include "interface/bms/bms_analysis.h"
#include "interface/modbus/modbus_defines.h"
#include "device_drv/xmodem/xmodemdata.h"
#include "device_drv/ota_upgrade/ota_fun.h"
#include "device_drv/sd_store/sd_store.h"
#include "interface/bms/bms_simulink/CANFDSendFcn_BCU.h"

#define APP_PATH  "/opt/xcharge"  

ECUStatus ecustatus = {0};

void ECU_OTA(void)
{
    LOG("[OTA] ECU_OTA start!, get_ota_OTAStart():%d\r\n", get_ota_OTAStart());
    if (!get_ota_OTAStart()) return;

    memset(&ecustatus, 0, sizeof(ECUStatus));//异常状态
    LOG("[OTA] get_ota_deviceType() : %d \r\n", get_ota_deviceType());
    LOG("[OTA] can id 0x%x device ota start!\r\n", get_ota_deviceID());
    set_modbus_reg_val(OTASTATUSREGADDR, OTASTARTRUNNING);

    if( (get_ota_deviceID() == 0) && (get_ota_deviceType() == ECU))
    {
        set_modbus_reg_val(OTAPPROGRESSREGADDR, 10); // 进度10%
        
        unsigned int bat_error = 0;
        int ret = unzipfile(APP_PATH, &bat_error, FILE_TYPE_BAT_ECU);
        if (ret > 0) {
            ecustatus.ErrorReg = 0;
            LOG("[OTA] bat_ecu upgrade file detected, replaced %s/bat_ecu\n", APP_PATH);
        } else if (bat_error == OTA_ERR_ARCHIVE_CONTENT_MISSING) {
            LOG("[OTA] bat_ecu not found in archive, fallback to deb flow\n");
            ret = unzipfile("/var",(unsigned int *)&ecustatus.ErrorReg,FILE_TYPE_DEB);
        } else {
            ecustatus.ErrorReg = bat_error;
        }
        
        if(ret < 0){
            set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED); 
            LOG("ecustatus.ErrorReg = %d\r\n",ecustatus.ErrorReg);
            return;
        }else{
            set_modbus_reg_val(OTAPPROGRESSREGADDR, 70); // 进度70%
        }
        
        
        // 完成操作
        if(ecustatus.ErrorReg == 0)
        {
            set_modbus_reg_val(OTAPPROGRESSREGADDR, 100); // 进度100%
            set_modbus_reg_val(OTASTATUSREGADDR, OTASUCCESS);
            system("sync");// 确保数据写入磁盘
            sleep(5); // 保留成功状态，确保上位机能够读到
            LOG("[OTA] OTA process completed successfully\n");
        }else{
            LOG("[OTA] can id 0x%x device ota failed, error register val 0x%x!\r\n", get_ota_deviceID(), ecustatus.ErrorReg);
            set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED); 
        }
    }
    else
    {
        LOG("[OTA] get_ota_deviceID() = 0x%x, get_ota_deviceType() = %d\r\n",get_ota_deviceID(),get_ota_deviceType());
        ecustatus.ErrorReg = OTA_ERR_INVALID_DEVICE_CONTEXT;
        ecustatus.ErrorDeviceID = get_ota_deviceID();
    }
 
    return;
}



void FinshhECUOtaAndCleanup(void)
{
    set_ota_deviceType(0);
    set_ota_OTAStart(0);
    LOG("[OTA ECU] OTA finished, cleaning up...\n");
    cleanup_ota_staging_files();
	set_ota_UpDating(0);//1130(升级结束)
	ecustatus.CANStartOTA = 0;
    set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
    LOG("[OTA] OTA finished, cleaning up...\n");
	set_modbus_reg_val(OTASTATUSREGADDR, OTAIDLE);
    CANFDSendFcn_BCU_step();
}
