#include "update_firmware.h"
#include "device_drv/sd_store/sd_store.h"
#include "ocpp_messages.h"
#include "ocpp_app_update.h"
#include "device_drv/ota_upgrade/ota_fun.h"
int g_ocppdownload_flag = 0;

static size_t write_callback(void *contents, size_t size, size_t nmemb, void *userp) {
    FILE *fp = (FILE *)userp;
    size_t written = fwrite(contents, size, nmemb, fp);
    if (written != nmemb) {
        fprintf(stderr, "写入文件失败\n");
    }
    return written * size;
}


// 下载文件（支持HTTP/HTTPS和FTP）
// 参数: url - 文件下载地址
int download_file(const char *url,const char *filetype) {
    CURL *curl;
    FILE *fp;
    CURLcode res;
    char download_filename[MAX_PATH_LENGTH];

    if (!url || !filetype || !ota_filename_is_safe(filetype)) {
        fprintf(stderr, "Error: url or filetype is NULL\n");
        return -1;
    }
    
    curl = curl_easy_init();
    if (!curl) {
        fprintf(stderr, "curl init failed\n");
        return -1;
    }
    if (ensure_ota_storage_dirs() != 0) {
        fprintf(stderr, "local OTA storage is unavailable\n");
        curl_easy_cleanup(curl);
        return -1;
    }
    int path_len = snprintf(download_filename, MAX_PATH_LENGTH, "%s/%s", OTA_INCOMING_PATH, filetype);
    if (path_len < 0 || path_len >= MAX_PATH_LENGTH) {
        fprintf(stderr, "OTA download path is too long\n");
        curl_easy_cleanup(curl);
        return -1;
    }
    LOG("Download File...........: %s\n", download_filename);
    fp = fopen(download_filename, "wb");
    if (!fp) {
        fprintf(stderr, "can't open file %s\n", download_filename);
        curl_easy_cleanup(curl);
        return -1;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 300L);

    res = curl_easy_perform(curl);

    if (res != CURLE_OK) {
        fprintf(stderr, "Download File Failed: %s\n", curl_easy_strerror(res));
        LOG("Download File Failed\r\n");
        fclose(fp);
        unlink(download_filename);
        curl_easy_cleanup(curl);
        return -1;
    }

    int persist_error = 0;
    if (fflush(fp) != 0) {
        persist_error = 1;
    }
    if (fsync(fileno(fp)) != 0) {
        persist_error = 1;
    }
    if (fclose(fp) != 0) {
        persist_error = 1;
    }
    if (persist_error) {
        LOG("[OCPP] Failed to persist downloaded OTA file: %s\n", download_filename);
        unlink(download_filename);
        curl_easy_cleanup(curl);
        return -1;
    }
    curl_easy_cleanup(curl);
    LOG("Download File. Success: %s\n", download_filename);
    // get_check_upgarde_file_type(DOWNLOAD_FILENAME,filetype);
    upgarde_file_type(download_filename,filetype);
    return 0;
}

static int extract_ota_filename_from_url(const char *url, char *out, size_t out_size) {
    const char *start;
    const char *end;
    size_t len;

    if (!url || !out || out_size == 0) {
        return -1;
    }

    start = strrchr(url, '/');
    start = start ? start + 1 : url;
    end = strpbrk(start, "?#");
    len = end ? (size_t)(end - start) : strlen(start);
    if (len == 0 || len >= out_size) {
        return -1;
    }

    memcpy(out, start, len);
    out[len] = '\0';
    return ota_filename_is_safe(out) ? 0 : -1;
}

void* firmware_download_worker(void* arg) {
    char *url = (char*)arg;
    char filetype[OTAFILENAMEMAXLENGTH] = {0};

    if (extract_ota_filename_from_url(url, filetype, sizeof(filetype)) == 0) {
        LOG("The firmware file name is: %s\n", filetype);
    } else {
        LOG("Extracting file name failed\n");
        free(url);
        return NULL;
    }
    if (__sync_lock_test_and_set(&g_ocppdownload_flag, 1) != 0) {
        LOG("[OCPP] Another firmware download is already running\n");
        send_ocpp_message(FirmwareStatusNotification(DownloadFailed));
        free(url);
        return NULL;
    }
    int success = download_file(url,filetype); // 阻塞上传
    __sync_lock_release(&g_ocppdownload_flag);
    sleep(1);
    if(0 == success)
    {
        send_ocpp_message(FirmwareStatusNotification(Downloaded));
    }
    else{
        send_ocpp_message(FirmwareStatusNotification(DownloadFailed));
    }
    free(url);
    return NULL;
}
void handle_update_firmware(struct lws *wsi, json_object *json) {
    // json: [2, "messageId", "UpdateFirmware", { "location": "...", "retrieveDate": "..." }]
    if (!json_object_is_type(json, json_type_array)) {
        LOG("[OCPP] Invalid message: not a JSON array.\n");
        return;
    }

    json_object *msg_type = json_object_array_get_idx(json, 0);
    json_object *msg_id = json_object_array_get_idx(json, 1);
    //json_object *command = json_object_array_get_idx(json, 2);
    json_object *payload = json_object_array_get_idx(json, 3);

    if (!msg_type || !msg_id || !payload || !json_object_is_type(msg_id, json_type_string) || !json_object_is_type(payload, json_type_object)) {
        LOG("[OCPP] Invalid UpdateFirmware message structure.\n");
        return;
    }

    const char *message_id = json_object_get_string(msg_id);

    // 1. 回复 CALLRESULT 消息
    json_object *ack = json_object_new_array();
    json_object_array_add(ack, json_object_new_int(3));  // CALLRESULT
    json_object_array_add(ack, json_object_new_string(message_id)); // Same ID as request
    json_object_array_add(ack, json_object_new_object()); // Empty payload

    send_ocpp_message(ack);

    // 2. 提取参数
    json_object *location_obj = NULL;
    json_object *retrieve_date_obj = NULL;

    json_object_object_get_ex(payload, "location", &location_obj);
    json_object_object_get_ex(payload, "retrieveDate", &retrieve_date_obj);

    if (!location_obj || !retrieve_date_obj || !json_object_is_type(location_obj, json_type_string) || !json_object_is_type(retrieve_date_obj, json_type_string)) {
        LOG("Missing or invalid location/retrieveDate in payload.\n");
        return;
    }

    const char *location = json_object_get_string(location_obj);
    const char *retrieve_date = json_object_get_string(retrieve_date_obj);

    LOG("UpdateFirmware: location=%s, retrieveDate=%s\n", location, retrieve_date);

    // Step 2: 启动异步上传（不要阻塞！）
    char *url_copy = strdup(location); // 避免悬空指针
    if (!url_copy) {
        send_ocpp_message(FirmwareStatusNotification(DownloadFailed));
        return;
    }
    pthread_t tid;
    if (pthread_create(&tid, NULL, firmware_download_worker, url_copy) == 0) {
        pthread_detach(tid);
    } else {
        perror("Failed to start upload thread");
        send_ocpp_message(FirmwareStatusNotification(DownloadFailed));
        free(url_copy);
    }
}



// 处理固件状态通知
void handle_firmware_status_notification(struct lws *wsi, json_object *json) {
    json_object *payload = json_object_array_get_idx(json, 3);
    json_object *status_obj = json_object_object_get(payload, "status");
    const char *status = json_object_get_string(status_obj);
    
    LOG("Received firmware status notification: %s\n", status);
    
    json_object *response = json_object_new_array();
    json_object_array_add(response, json_object_new_int(3));
    
    json_object *msg_id_obj = json_object_array_get_idx(json, 1);
    json_object_array_add(response, msg_id_obj);
    
    json_object *response_payload = json_object_new_object();
    json_object_array_add(response, response_payload);
    
    send_ocpp_message(response); /*消息队列存放json对象指针，发送后统一调用 json_object_put释放*/;
}

// 发送固件状态通知
struct json_object *FirmwareStatusNotification(OCPP_Download_STATUS Status) {

    char FirmwareStatus[100];

    switch (Status)
    {
        case Downloading:
            strcpy(FirmwareStatus,"Downloading");
            break;
        case Downloaded:
            strcpy(FirmwareStatus,"Downloaded");
            break;
        case DownloadFailed:
            strcpy(FirmwareStatus,"DownloadFailed");
            break;
        case Installing:
            strcpy(FirmwareStatus,"Installing");
            break;
        case Installed:
            strcpy(FirmwareStatus,"Installed");
            break;
        case InstallFailed:
            strcpy(FirmwareStatus,"InstallFailed");
            break;
        default:
            break;
    }

    struct json_object *payload = json_object_new_object();
    json_object_object_add(payload, "status", json_object_new_string(FirmwareStatus)); // 通常 0 表示整机状态

    // 构建最外层的数组消息
    struct json_object *msg = json_object_new_array();
    json_object_array_add(msg, json_object_new_int(2));  // 消息类型 CALL = 2
    json_object_array_add(msg, json_object_new_string(FIRMWARESTATUSNOTIFICATION_ID)); // 消息 ID，应唯一
    json_object_array_add(msg, json_object_new_string("FirmwareStatusNotification"));
    json_object_array_add(msg, payload);  // 参数对象

    return msg;
}
