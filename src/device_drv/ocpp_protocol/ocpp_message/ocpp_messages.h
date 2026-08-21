#ifndef OCPP_MESSAGES_H
#define OCPP_MESSAGES_H

#include <json-c/json.h>
#include <zstd.h>
#include <sys/stat.h>
#include <curl/curl.h>
#include <libwebsockets.h>
#include "batdata.h"
#include "sqlite_storage.h"

#define BUILD_X86  1
#define REPORT_COUNT 60

#pragma pack(push, 1)
typedef struct {
    unsigned short usDataStyle;      /* 数据格式版本号（2字节），固定为1，用于指定后面数据的协议版本*/
    unsigned short usPackCount;      /* Pack数量（2字节），目前为15个Pack*/
    unsigned short usVoltCount;      /* 电池电压数量（2字节），目前为240个电池电压（15个Pack，每个16个）*/
    unsigned short usTempCount;      /* 电池温度数量（2字节），目前为120个采集点（15个Pack，每个8个）*/
    unsigned int   uiArrayLength;    /* 数组长度（4字节），最大255，无数据则为0*/
} tDetailHead;
#pragma pack(pop)

#define HEARTBEAT_ID                        "1915669379150258177"
#define BOOTNOTIFICATION_ID                 "1915669379150258176"
#define STATUSNOTIFICATION_ID               "1915669379150258178"
#define DIAGNOSTICSSTATUSNOTIFICATION_ID    "1915669379150258179"
#define FIRMWARESTATUSNOTIFICATION_ID       "1915669379150258180"

struct json_object *build_boot_notification();
struct json_object *build_heartbeat();
struct json_object *compress_detail_data(sqlite3 *db, int *out_ids, int *out_id_count);

void handle_call_message(struct lws *wsi, json_object *json);
void handle_call_result_message(struct lws *wsi, json_object *json);
void handle_call_error_message(struct lws *wsi, json_object *json);
void handle_trigger_report_energy_storage_status_v2(struct lws *wsi, json_object *json);
void handle_heartbeat(struct lws *wsi, json_object *json);
int process_ocpp_message(struct lws *wsi, const char *message);
void handle_ChangeConfiguration(struct lws *wsi, json_object *json);

int send_ocpp_message(json_object *msg);


#endif
