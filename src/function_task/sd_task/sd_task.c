#define _GNU_SOURCE
#include "sd_task.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/log/log.h"


pthread_t SDCardDataSave_TASKHandle = 0;
extern pthread_mutex_t ftp_file_io_mutex;
/*==========================================================**/
void *SDCardDataSaveTask(void *arg)
{
    const char *mount_point = USB_MOUNT_POINT; // 使用自动挂载的路径
    uint16_t SD_INIT_flag = 0;

    sd_storeInit();//SD卡存储消息的初始化

    // 1. 检查挂载点
    if (ensure_mount_point(mount_point) != 0)
    {
        LOG("[SD Card] please check the mount point.\n");
        return NULL;
    }

    LOG("[SD Card] successed to create mount point: %s\n", mount_point);

    while (1)
    {
        usleep(3000 * 1000);
        // 获取sd卡初始化标识
        get_modbus_reg_val(MDBUS_SD_FROMAT, &SD_INIT_flag);//接收上位机指令
        
        // 如果sd卡未初始化 则初始化sd卡
        if (SD_INIT_flag == 1)
        {
            int result = SD_Initialize();
            if (result == 0)
            {
                set_modbus_reg_val(MDBUS_SD_FROMAT, 2); // 成功
                LOG("[SD Card] SD_Initialize succeeded.\n");
                mkdir_log(USB_MOUNT_POINT);
            }
            else
            {
                set_modbus_reg_val(MDBUS_SD_FROMAT, 3); // 失败
                LOG("[SD Card] SD_Initialize error code: %d \n", result);
            }
        }

        if ((get_ota_OTAStart() == 0) && SD_INIT_flag != 1)// 没有ota中 并且 sd卡初始化过
        {
            pthread_mutex_lock(&ftp_file_io_mutex);  // ← 加锁，ftp读文件的时候，不写文件
            Drv_write_buffer_to_file(); // 将缓冲区内容写入文件
            pthread_mutex_unlock(&ftp_file_io_mutex);  // ← 解锁
        }
        else
        {
            LOG("[SD Card] ----------------OTAing---------flag = 0x%X-----\r\n", g_ota_flag);
        }
    }
}
void SDCardDataSaveTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&SDCardDataSave_TASKHandle, NULL, SDCardDataSaveTask, NULL);
        if (ret != 0)
        {
            LOG("[SD Card] Failed to create SDCardDataSaveTaskCreate thread : %s", strerror(ret));
            sleep(1);
        }
        else
        {
            LOG("[SD Card] SDCardDataSaveTaskCreate thread created successfully.\r\n");
        }
    } while (ret != 0);
}