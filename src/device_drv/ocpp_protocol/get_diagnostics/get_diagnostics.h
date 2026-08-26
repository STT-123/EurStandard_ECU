#ifndef _GET_DIAGNOSTICS_H
#define _GET_DIAGNOSTICS_H

#include "device_drv/sd_store/sd_store.h"
#include "ocpp_messages.h"
#include "ocpp_app_update.h"

#define UPLOAD_FILE_PATH "/userdata/xcharge/log/app_project.log.bz2"  // 固定上传的文件路径
#define LOG_FILE_PATH     "/userdata/xcharge/log/app_project.log"

typedef enum
{
	Uploading,
    Uploaded,
    UploadFailed
} OCPP_UPLOAD_STATUS;


struct json_object *DiagnosticsStatusNotification(OCPP_UPLOAD_STATUS Status);
void handle_diagnostics_status_notification(struct lws *wsi, json_object *json) ;
static int upload_progress_callback(void *clientp,curl_off_t dltotal, curl_off_t dlnow,curl_off_t ultotal,curl_off_t ulnow);
int ocpp_upload_file(const char *url);
void* diagnostics_upload_worker(void* arg) ;
void handle_get_diagnostics(struct lws *wsi, json_object *json);
#endif
