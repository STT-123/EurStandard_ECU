#include "ocpp_messages.h"
#include <json-c/json.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "message_queue.h"
#include "interface/log/log.h"
#include <json-c/json_object.h>
#include "get_diagnostics.h"
#include "update_firmware.h"


// 发送OCPP消息  /*消息队列存放json对象指针，发送后统一调用 json_object_put释放*/
int send_ocpp_message(json_object *msg) {

    if(enqueue_message(msg)){
        return 0;
    }else{
        LOG("enqueu_msg error\n");
        return -1;
    }    
}

// 处理OCPP消息
int process_ocpp_message(struct lws *wsi, const char *message) 
{
    json_object *json = json_tokener_parse(message);
    if (!json) {
        LOG("Failed to parse JSON message\n");
        return -1;
    }
    
    if (json_object_get_type(json) != json_type_array || json_object_array_length(json) < 3) {
        LOG("invalid OCPP message format\n");
        json_object_put(json);
        return -1;
    }
    
    json_object *msg_type_obj = json_object_array_get_idx(json, 0);
    int msg_type = json_object_get_int(msg_type_obj);
    
    switch (msg_type) {
        case 2:
            handle_call_message(wsi, json);//调用请求
            break;
        case 3:
            handle_call_result_message(wsi, json);//调用结果，不处理
            break;
        case 4:
            handle_call_error_message(wsi, json);//调用错误
            break;
        default:
            LOG("Unknown OCPP message type: %d\n", msg_type);
            break;
    }
    
    json_object_put(json);
    return 0;
}

// 处理CALL消息
void handle_call_message(struct lws *wsi, json_object *json) {
    json_object *action_obj = json_object_array_get_idx(json, 2);
    const char *action = json_object_get_string(action_obj);
    
    //printf("收到CALL消息，动作: %s\n", action);
    
    if (strcmp(action, "Heartbeat") == 0) {
        handle_heartbeat(wsi, json);
    }
    else if (strcmp(action, "UpdateFirmware") == 0) {
        handle_update_firmware(wsi, json);
    }
    else if (strcmp(action, "GetDiagnostics") == 0) {
        handle_get_diagnostics(wsi, json);
    }
    else if (strcmp(action, "FirmwareStatusNotification") == 0) {
        handle_firmware_status_notification(wsi, json);
    }
    else if (strcmp(action, "DiagnosticsStatusNotification") == 0) {
        handle_diagnostics_status_notification(wsi, json);
    }
    else if (strcmp(action, "DataTransfer") == 0){
        handle_trigger_report_energy_storage_status_v2(wsi, json);
    }
    else if (strcmp(action, "ChangeConfiguration") == 0){
        handle_ChangeConfiguration(wsi, json);
    }
    else {
        LOG("Unrealized OCPP action: %s\n", action);
    }
}


char *generate_unique_id() {
    static char id[32];
    static unsigned counter = 0;
    snprintf(id, sizeof(id), "%ld%05u", time(NULL), counter++);
    return id;
}

// 处理CALLRESULT消息
void handle_call_result_message(struct lws *wsi, json_object *json) {
    json_object *msg_id_obj = json_object_array_get_idx(json, 1);
    const char *msg_id = json_object_get_string(msg_id_obj);
}

// 处理CALLERROR消息
void handle_call_error_message(struct lws *wsi, json_object *json) {
    json_object *msg_id_obj = json_object_array_get_idx(json, 1);
    const char *msg_id = json_object_get_string(msg_id_obj);
    
    json_object *error_code_obj = json_object_array_get_idx(json, 2);
    const char *error_code = json_object_get_string(error_code_obj);
    
    json_object *error_description_obj = json_object_array_get_idx(json, 3);
    const char *error_description = json_object_get_string(error_description_obj);
    
    LOG("Processing CALLERROR messages, ID: %error code: %s, descriptor: %s\n", 
            msg_id, error_code, error_description);
}


// 处理心跳请求
void handle_heartbeat(struct lws *wsi, json_object *json) {

    json_object *response = json_object_new_array();//创建一个JSON数组对象
    // printf("Refcount: %d\n", json_object_get_refcount(response));
    /*-----------
        1.新建对象1
        json_object *temp = json_object_new_int(3); // refcount = 1
        json_object_array_add(response, temp);      // response 拥有 temp
    -------------*/

    json_object_array_add(response, json_object_new_int(3));//增加一个元素到json数组的末尾 ；//创建josn的int 32对象
    //2.新建对象2
    json_object *orig_msg_id  = json_object_array_get_idx(json, 1);//按照索引获取json数组的对象
    const char *msg_id_str = json_object_get_string(orig_msg_id);
    json_object_array_add(response, json_object_new_string(msg_id_str)); // 
    
    //3.新建对象3
    json_object *payload = json_object_new_object();//创建一个新的JSON对象，引用计数为1，该指针具有唯一的所有权
    time_t now = time(NULL);
    char timestamp[26];
    ctime_r(&now, timestamp);
    timestamp[24] = '\0';
    json_object_object_add(payload, "currentTime", json_object_new_string(timestamp));//添加一个键值对到JSON对象中
    json_object_array_add(response, payload);//增加一个元素到json数组的末尾

    send_ocpp_message(response); /*消息队列存放json对象指针，发送后统一调用 json_object_put释放*/; 
}

// 处理固件更新请求


void handle_trigger_report_energy_storage_status_v2(struct lws *wsi, json_object *json) {
    if (!json_object_is_type(json, json_type_array)) {
        LOG("Invalid TriggerReportEnergyStorageStatusV2 message: not a JSON array.\n");
        return;
    }

    json_object *msg_id_obj = json_object_array_get_idx(json, 1);
    if (!msg_id_obj || !json_object_is_type(msg_id_obj, json_type_string)) {
        LOG("Missing or invalid message ID in TriggerReportEnergyStorageStatusV2.\n");
        return;
    }

    const char *message_id = json_object_get_string(msg_id_obj);

    // ✅ 这里判断是否接受该请求，可替换为你自己的条件判断逻辑
    const char *status = "Accepted";  // 或 "Rejected"

    // 构造 CALLRESULT 消息
    json_object *response = json_object_new_array();
    json_object_array_add(response, json_object_new_int(3));  // CALLRESULT 类型
    json_object_array_add(response, json_object_new_string(message_id));

    json_object *payload = json_object_new_object();
    json_object_object_add(payload, "status", json_object_new_string(status));  // status: CiString
    json_object_array_add(response, payload);

    // ✅ 发送消息（会统一释放）
    send_ocpp_message(response);

    LOG("call TriggerReportEnergyStorageStatusV2，status: %s\n", status);
}


struct json_object *build_boot_notification() {
    // 第四个参数对象
    struct json_object *params = json_object_new_object();
    json_object_object_add(params, "chargeBoxSerialNumber", json_object_new_string("C8A215DPWUDYDTAWLL"));
    json_object_object_add(params, "chargePointModel", json_object_new_string("C8EU"));
    json_object_object_add(params, "chargePointSerialNumber", json_object_new_string("C8A215DPWUDYDTAWLL"));
    json_object_object_add(params, "chargePointVendor", json_object_new_string("xcharge"));
    json_object_object_add(params, "firmwareVersion", json_object_new_string("V1.0.0_C8EU_OCPP_STA_250508"));
    json_object_object_add(params, "iccid", json_object_new_string(""));

    // 外层数组
    struct json_object *msg = json_object_new_array();
    json_object_array_add(msg, json_object_new_int(2));  // [0] 消息类型 2 = CALL
    json_object_array_add(msg, json_object_new_string(BOOTNOTIFICATION_ID));   // [1] 消息 ID
    json_object_array_add(msg, json_object_new_string("BootNotification"));    // [2] 动作名
    json_object_array_add(msg, params);                                        // [3] 参数对象

    return msg;
}


void handle_ChangeConfiguration(struct lws *wsi, json_object *json){
    // 构建最外层的数组消息
    json_object *response = json_object_new_array();

    // 步骤1：返回 [0] 3
    json_object_array_add(response, json_object_new_int(3));  
    // 步骤2：
    json_object *msg_id_obj = json_object_array_get_idx(json, 1);
    if (!msg_id_obj) {
        LOG("Missing or invalid message ID in TriggerReportEnergyStorageStatusV2.\n");
        json_object_put(response);
        return;
    }

    if (!json_object_is_type(msg_id_obj, json_type_string)) {
        LOG("[OCPP] Error: Message ID is not a string (type=%d)\n", json_object_get_type(msg_id_obj));
        json_object_put(response);
        return;
    }

    const char *message_id = json_object_get_string(msg_id_obj);
    json_object_array_add(response, json_object_new_string(message_id));
    // 步骤3：返回 [2] status
    json_object *status_obj = json_object_new_object();
    json_object_object_add(status_obj, "status", json_object_new_string("Accepted"));
    json_object_array_add(response, status_obj);
    send_ocpp_message(response);
}
/**
 * 诊断状态通知 
*/
struct json_object *build_data_transfer(const char *base64_str, size_t size) {
    // 构建 status[0] 对象
    static int soc ;
    struct json_object *status_item = json_object_new_object();
    json_object_object_add(status_item, "num", json_object_new_int(1));
    json_object_object_add(status_item, "status", json_object_new_string("idle"));

    struct json_object *errorInfo = json_object_new_array();  // 空数组
    json_object_object_add(status_item, "errorInfo", errorInfo);

    json_object_object_add(status_item, "voltage", json_object_new_int(800));
    json_object_object_add(status_item, "current", json_object_new_int(20));
    json_object_object_add(status_item, "power", json_object_new_int(7));
    soc++;
    if(soc ==100)
    {
        soc = 0;
    }
    json_object_object_add(status_item, "soc", json_object_new_int(soc));
    json_object_object_add(status_item, "soh", json_object_new_int(95));
    json_object_object_add(status_item, "batteryTemperature", json_object_new_double(25.3));

    // ISO8601 时间戳
    time_t now = time(NULL);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
    json_object_object_add(status_item, "timestamp", json_object_new_string(timestamp));

    // 已是 base64 字符串，无需编码

    //json_object_object_add(status_item, "detail", json_object_new_string_len(base64_str, size));
    // detail 是一个 Base64 字符串，直接作为字符串加入
    json_object_object_add(status_item, "detail", json_object_new_string(base64_str));
    //json_object_object_add(status_item, "detail", json_object_new_string(base64_str));

    
    // 构建 status 数组
    struct json_object *status_array = json_object_new_array();
    json_object_array_add(status_array, status_item);

    // 构建 data 对象（嵌套 JSON 字符串）
    struct json_object *data_obj = json_object_new_object();
    json_object_object_add(data_obj, "status", status_array);
#if(BUILD_X86)
    const char *data_json_str = json_object_to_json_string(data_obj);
#else
    const char *data_json_str = json_object_to_json_string_ext(data_obj, JSON_C_TO_STRING_NOSLASHESCAPE);
#endif
    struct json_object *data_str_obj = json_object_new_string(data_json_str);
    json_object_put(data_obj); 


    // 构建 DataTransfer.req 消息体
    struct json_object *params = json_object_new_object();
    json_object_object_add(params, "vendorId", json_object_new_string("xcharge"));
    json_object_object_add(params, "messageId", json_object_new_string("ReportEnergyStorageStatusV2"));
    json_object_object_add(params, "data", data_str_obj);

    struct json_object *msg = json_object_new_array();
    json_object_array_add(msg, json_object_new_int(2));
    json_object_array_add(msg, json_object_new_string(generate_unique_id()));  // 可替换为实际唯一 ID
    json_object_array_add(msg, json_object_new_string("DataTransfer"));
    json_object_array_add(msg, params);

    return msg;
}
struct json_object *build_heartbeat() {
    struct json_object *msg = json_object_new_array();
    json_object_array_add(msg, json_object_new_int(2));
    json_object_array_add(msg, json_object_new_string(HEARTBEAT_ID));
    json_object_array_add(msg, json_object_new_string("Heartbeat"));
    json_object_array_add(msg, json_object_new_object());  // 无参数
    return msg;
}


static const char base64_encode_table[] = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
// Base64 编码函数
char *base64_encode(const unsigned char *data, size_t input_length, size_t *output_length)
{
    *output_length = 4 * ((input_length + 2) / 3); // 计算输出长度
    
    char *encoded_data = malloc(*output_length + 1); // 分配内存，+1 用于 null 终止符
    if (encoded_data == NULL) {
        return NULL;
    }
    
    for (size_t i = 0, j = 0; i < input_length;) {
        uint32_t octet_a = i < input_length ? data[i++] : 0;
        uint32_t octet_b = i < input_length ? data[i++] : 0;
        uint32_t octet_c = i < input_length ? data[i++] : 0;
        
        uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;
        
        encoded_data[j++] = base64_encode_table[(triple >> 3 * 6) & 0x3F];
        encoded_data[j++] = base64_encode_table[(triple >> 2 * 6) & 0x3F];
        encoded_data[j++] = base64_encode_table[(triple >> 1 * 6) & 0x3F];
        encoded_data[j++] = base64_encode_table[(triple >> 0 * 6) & 0x3F];
    }
    
    // 添加填充字符 '='
    for (size_t i = 0; i < (3 - input_length % 3) % 3; i++) {
        encoded_data[*output_length - 1 - i] = '=';
    }
    
    encoded_data[*output_length] = '\0'; // 添加 null 终止符
    return encoded_data;
}


// ✅ 构建并压缩 detail 字段数据，输出 JSON 对象（需要手动释放）
struct json_object *compress_detail_data(sqlite3 *db, int *out_ids, int *out_id_count) {
    tBatData buffer[REPORT_COUNT];
    int ids[REPORT_COUNT];
    int n = get_recent_data(db, buffer, REPORT_COUNT, ids);
    tBatData *network_buffer = NULL;
    ZSTD_CStream *cstream = NULL;
    void *compressed = NULL;
    struct json_object *json = NULL;

    if (n <= 0) return NULL;
    if (n > 255) n = 255;
    // 为网络传输准备数据
    network_buffer = malloc(sizeof(tBatData) * n);
    if (!network_buffer) {
        LOG("分配网络缓冲区失败\n");
        return NULL;
    }
    
    for (int i = 0; i < n; i++) {
        memcpy(&network_buffer[i], &buffer[i], sizeof(tBatData)); 
        unsigned int original_ts = buffer[i].uiTimeStamp;// 保存原始值用于调试
        network_buffer[i].uiTimeStamp = htonl(buffer[i].uiTimeStamp); // 转换为网络字节序
    }
    // printf("get_recent_data n = %d...data =%lu,\n", n,sizeof(tBatData));
    memcpy(out_ids, ids, sizeof(int) * n);
    *out_id_count = n;

    tDetailHead head = {
        .usDataStyle = 1,
        .usPackCount = 15,
        .usVoltCount = 240,
        .usTempCount = 120,
        .uiArrayLength = n
    };
    tDetailHead head_be;
    head_be.usDataStyle = htons(head.usDataStyle);
    head_be.usPackCount = htons(head.usPackCount);
    head_be.usVoltCount = htons(head.usVoltCount);
    head_be.usTempCount = htons(head.usTempCount);
    head_be.uiArrayLength = htonl(head.uiArrayLength);

    cstream = ZSTD_createCStream();
    if (!cstream) {
        LOG("ZSTD_createCStream failed\n");
        goto cleanup;
    }
    // 设置压缩参数
#if(BUILD_X86)
    ZSTD_initCStream(cstream, 3);  // 压缩等级 3
#else
    ZSTD_CCtx_setParameter(cstream, ZSTD_c_compressionLevel, 3);              // 压缩等级
    ZSTD_CCtx_setParameter(cstream, ZSTD_c_checksumFlag, 0);                 // 关闭 checksum（--no-check）
    ZSTD_CCtx_setParameter(cstream, ZSTD_c_contentSizeFlag, 0);              // 不在 header 中写 content size（默认不写，可显式设置）
    ZSTD_CCtx_setParameter(cstream, ZSTD_c_dictIDFlag, 0);                   // 不写字典 ID（--no-dictID）
#endif
    size_t max_dst_size = ZSTD_compressBound(sizeof(tDetailHead) + sizeof(tBatData) * n);
    compressed = malloc(max_dst_size);
    if (!compressed) {
        LOG("分配压缩缓冲区失败\n");
        goto cleanup;
    }
    ZSTD_outBuffer out = { compressed, max_dst_size, 0 };

    // 压缩头
    ZSTD_inBuffer inHead = { &head_be, sizeof(head_be), 0 };
    while (inHead.pos < inHead.size) {
        size_t ret = ZSTD_compressStream(cstream, &out, &inHead);
        if (ZSTD_isError(ret)) {
            // fprintf(stderr, "compressStream head failed: %s\n", ZSTD_getErrorName(ret));
            zlog_error(zlog_get_category("debug_file"), "compressStream head failed: %s", ZSTD_getErrorName(ret));
            goto cleanup;
        }
    }

    // 压缩数据
    ZSTD_inBuffer inData = { network_buffer, sizeof(tBatData) * n, 0 };
    while (inData.pos < inData.size) {
        size_t ret = ZSTD_compressStream(cstream, &out, &inData);
        if (ZSTD_isError(ret)) {
            // fprintf(stderr, "compressStream data failed: %s\n", ZSTD_getErrorName(ret));
             zlog_error(zlog_get_category("debug_file"), "compressStream data failed: %s", ZSTD_getErrorName(ret));
            goto cleanup;
        }
    }

    // 刷尾
    size_t remaining;
    do {
        remaining = ZSTD_endStream(cstream, &out);
        if (ZSTD_isError(remaining)) {
            // fprintf(stderr, "endStream failed: %s\n", ZSTD_getErrorName(remaining));
            zlog_error(zlog_get_category("debug_file"), "endStream failed: %s", ZSTD_getErrorName(remaining));
            goto cleanup;
        }
    } while (remaining > 0);

    ZSTD_freeCStream(cstream);
    cstream = NULL;

    // for(int i=0;i<out.pos;i++)
    // {
    //     printf("base64_str 压缩前是%x \n", ((unsigned char*)compressed)[i]);
    // }


    size_t base64_len;
    char *base64_str = base64_encode(compressed, out.pos, &base64_len);
    if (!base64_str) {
        LOG("Base64 encoding error\n");
        goto cleanup;
    }
    //printf("base64_str: %s 压缩后字节是%d \n",base64_str,out.pos);
    // 构建 JSON
    json = build_data_transfer(base64_str, base64_len);
    free(base64_str);

cleanup:
    if (cstream) {
        ZSTD_freeCStream(cstream);
    }
    free(network_buffer);
    free(compressed);
    return json;
}
