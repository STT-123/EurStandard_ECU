#include "ocpp_send.h"
#include "update_firmware.h"
#include "get_diagnostics.h"

extern struct lws *global_wsi;
extern pthread_mutex_t wsi_lock;
extern int g_ocppupload_flag;
extern int g_ocppdownload_flag;
extern volatile int send_thread_should_run;
extern volatile int connection_is_active;
void update_bat_data(sqlite3 *db)
{
    tBatData data = {0};
    tBatData data_be = {0};
    uint16_t batDAq_version[15] = {0};

    data.usAirState =1;

    memcpy(batDAq_version,get_BMU_DAq_version(),sizeof(batDAq_version));

    memcpy(data.usSingleBatVal, get_BCU_usSingleBatVal(), sizeof(data.usSingleBatVal));
    memcpy(data.usSingleBatTemp, get_BCU_usSingleBatTemp(), sizeof(data.usSingleBatTemp));
    
    // for (int i = 0; i < 60; i++)
    // {
    //     printf("data.usSingleBatVal[%d] = %d\n", i, data.usSingleBatVal[i]);
    // }


    for(int i = 0; i < 15; i++){
        if(batDAq_version[i] != 0){
            data.uiBmuErrorNum[i] = get_BMU_DAqX_FaultCode1_at(i);
            // printf(" data.uiBmuErrorNum[%d]  = %x \r\n",i, data.uiBmuErrorNum[i]);
            data.uiBmuExErrorNum[i] = get_BMU_DAqX_FaultCode2_at(i);
        }else{
            data.uiBmuErrorNum[i] = 65535;
            data.uiBmuExErrorNum[i] = 65535;
        }
    }

    data.iDcPower = get_BCU_iDcPower();
    data.ullPosEleQuantity = get_BCU_ullPosEleQuantity();
    data.ullNegEleQuantity = get_BCU_ullNegEleQuantity();

    data.usAirState = get_BCU_usAirState();
    data.usAirPumpState = get_BCU_usAirPumpState();
    data.usAirCompressorSta = get_BCU_usAirCompressorSta();

    int faultCode = get_BCU_uiAirErrorfaultCode();
    data.uiAirErrorLv1 = 0;
    data.uiAirErrorLv2 = 0;
    data.uiAirErrorLv3 = 0;

    // Lv1 错误判断
    if (faultCode == 1){
        data.uiAirErrorLv1 |= (1U << 1);   
    }
    if (faultCode == 3){
        data.uiAirErrorLv1 |= (1U << 3);    
    }
    if (faultCode == 28){
        data.uiAirErrorLv1 |= (1U << 28);
    }

    // Lv2 错误判断
    if (faultCode == 4 || faultCode == 5 ||(faultCode >= 8 && faultCode <= 17) ||(faultCode >= 20 && faultCode <= 22) ||
        faultCode == 25 || faultCode == 29 ||faultCode == 30 || faultCode == 31){
        data.uiAirErrorLv2 |= (1U << faultCode);      
    }

    // Lv3 错误判断
    if (faultCode == 18 || faultCode == 19 ||faultCode == 23 || faultCode == 24 ||faultCode == 26 || faultCode == 27)
    {
        data.uiAirErrorLv3 |= (1U << faultCode);
    }

    data.usBmuH2MaxValue = get_usBmuH2MaxValue();
    data.usBmuCOMaxValue = get_usBmuCOMaxValue();
    data.usBmuPressureMaxValue = get_usBmuPressureMaxValue();
    data.usBmuLightMaxValue = get_usBmuLightMaxValue();

    data.usBmuH2MaxIndex = get_usBmuH2MaxIndex();
    data.usBmuCOMaxIndex = get_usBmuCOMaxIndex();
    data.usBmuPressureMaxIndex = get_usBmuPressureMaxIndex();
    data.usBmuLightMaxIndex = get_usBmuLightMaxIndex();

    data.usAirEnergyMode = get_usAirEnergyMode();
    
    data.usAirCoolSetTemp = get_usAirCoolSetTemp();
    data.usAirInletPressure = get_usAirInletPressure();
    data.usAirOutWaterTemp = get_usAirOutWaterTemp();
    data.usAirReturnWaterTemp =  get_usAirReturnWaterTemp();

    data.usBatMaxVoltCellIndex = get_usBatMaxVoltCellIndex();
    data.usBatMinVoltCellIndex = get_usBatMinVoltCellIndex();
    data.usBatMaxTempCellIndex = get_usBatMaxTempCellIndex();
    data.usBatMinTempCellIndex = get_usBatMinTempCellIndex();

    data.usBatCellVoltMaxValue = get_usBatCellVoltMaxValue();
    data.usBatCellVoltMinValue = get_usBatCellVoltMinValue();
    data.usBatCellTempMaxValue = get_usBatCellTempMaxValue();
    data.usBatCellTempMinValue = get_usBatCellTempMinValue();

    // struct tm utc_timeinfo;
    // utc_timeinfo.tm_year = get_BCU_TimeYearValue() + 100; // BCU年是如24，tm_year从1900起
    // utc_timeinfo.tm_mon = get_BCU_TimeMonthValue() - 1;   // BCU月是1~12，tm_mon是0~11
    // utc_timeinfo.tm_mday = get_BCU_TimeDayValue();
    // utc_timeinfo.tm_hour = get_BCU_TimeHourValue() - 8;
    // utc_timeinfo.tm_min = get_BCU_TimeMinuteValue();
    // utc_timeinfo.tm_sec = get_BCU_TimeSencondValue();
    // utc_timeinfo.tm_isdst = -1;
    // time_t t = mktime(&utc_timeinfo);
    // data.uiTimeStamp = (unsigned int)t;
    
    // printf("Unix时间戳: %u (十进制), 0x%08x (十六进制)\n", (unsigned int)t, (unsigned int)t);
    //data.uiTimeStamp 代表从 1970-01-01 00:00:00 UTC 到该时刻所经过的秒数
    data.uiTimeStamp = (unsigned int)time(NULL); //如果时间比ocpp网页的小，则网页不更新数据，所以不要错误的给网页一个过大的时间戳
    convert_tBatData_to_big_endian(&data_be, &data);
    insert_data(db, &data_be);
}
void update_bat_data1(sqlite3 *db) {
    tBatData data = {0};
    tBatData data_be = {0};

    data.usAirState =1;

    for(int i=0;i<240;i++)
    {
        data.usSingleBatVal[i] =30 + (rand() % 40);
    }

    data.uiTimeStamp = (unsigned int)time(NULL);
    convert_tBatData_to_big_endian(&data_be,&data);
    insert_data(db, &data_be); 
}
/*
*websocket_send_thread 线程
*该线程专门用来给ocpp服务端发送数据
*心跳、电池数据等
*/
void *websocket_send_thread(void *arg)
{
    LOG("[Ocpp] Send thread started\n");
    
    int dbcounter = 0;
    int heartcounter = 0;
    int boot_sent = 0;
    sqlite3 *db = NULL;
    
    // 主循环
    while (send_thread_should_run) 
    {
        // 1. 检查连接状态
        pthread_mutex_lock(&wsi_lock);
        int can_send = (global_wsi != NULL);
        pthread_mutex_unlock(&wsi_lock);
        
        if (!can_send) {
            // 没有活跃连接
            if (connection_is_active) {
                LOG("[Ocpp] Send thread: connection lost\n");
                connection_is_active = 0;
            }
            
            // 等待连接恢复
            int wait_time = 0;
            while (send_thread_should_run && !connection_is_active && wait_time < 60) {
                sleep(1);
                wait_time++;
                
                // 定期检查
                if (wait_time % 10 == 0) {
                    LOG("[Ocpp] Send thread waiting for connection... (%d/60s)\n", wait_time);
                }
                
                pthread_mutex_lock(&wsi_lock);
                can_send = (global_wsi != NULL);
                pthread_mutex_unlock(&wsi_lock);
                
                if (can_send) {
                    connection_is_active = 1;
                    LOG("[Ocpp] Send thread: connection restored\n");
                    boot_sent = 0;  // 重置启动通知标志
                    break;
                }
            }
            
            if (wait_time >= 60) {
                LOG("[Ocpp] Send thread: timeout waiting for connection\n");
                // 继续循环，等待主线程恢复
            }
            
            continue;
        }
        
        // 2. 连接有效，执行发送任务
        sleep(1);  // 基础间隔
        
        // 发送启动通知（仅一次）
        if (!boot_sent) {
            if (db == NULL) {
                // 初始化数据库
                if (init_db(&db) > 0) {
                    LOG("[Ocpp] Send thread: database initialized\n");
                }
            }
            
            if (db) {
                send_ocpp_message(build_boot_notification());
                boot_sent = 1;
                LOG("[Ocpp] Boot notification sent\n");
            }
        }

        if (heartcounter++ >= 10) {    
            send_ocpp_message(build_heartbeat());
            heartcounter = 0;
        }
        
        
        // 更新电池数据
        if (db) {
            update_bat_data(db);
        }

        // db消息（每10秒）
        // printf("dbcounter =%d...\n", dbcounter);
        if (dbcounter++ >= 60) {
            
            int ids[REPORT_COUNT];
            int count = 0;
            send_ocpp_message(compress_detail_data(db, ids, &count));
            //if (send_ocpp_message(json)) {
            // delete_data_by_ids(db, ids, count);//发送成功应该删除调db数据
            // }
            LOG("SEND BAT DATA \r\n");
            dbcounter = 0;
        }
        
        // 处理诊断状态标志
        if (g_ocppupload_flag == 1) {
            send_ocpp_message(DiagnosticsStatusNotification(Uploading));
        }
        
        // 处理固件状态标志
        if (g_ocppdownload_flag == 1) {
            send_ocpp_message(FirmwareStatusNotification(Downloading));
        }
        
    }
    
    // 清理
    LOG("[Ocpp] Send thread exiting\n");
    
    // 清理数据库
    if (db) {
        sqlite3_close(db);
        db = NULL;
    }
    
    LOG("[Ocpp] Send thread cleanup complete\n");
    
    return NULL;
}
