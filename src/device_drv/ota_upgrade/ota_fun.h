#ifndef _OTA_FUN_H
#define _OTA_FUN_H 
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>

#define XMODEMISTTRANFERINGFILE 0x0001  // XMODEM正在传输文件
#define XMODEMNORMALTERMINATION 0x0002  // XMODEM正常终止（传输成功）
#define XMODEMOUTTIMETERMINATION 0x0004 // XMODEM超时终止
#define XMODEMABORTTERMINATION 0x0006   // XMODEM异常中止
//以上4个未使用
#define FILEISDECRYPTIIONING 0x0008		// 文件正在解密中
#define FILEDECRYPTIONNORMALTERMINATION 0x0010  // 文件解密正常完成

#define OTASTARTRUNNING 0x0080 		// OTA开始运行
#define OTAIDLE 0x0000				// OTA空闲状态
#define OTASUCCESS 0x0100			// OTA升级成功
#define OTAFAILED 0x0200			// OTA升级失败

#define OTAFILENAMEMAXLENGTH 128
#define MAX_FILE_COUNT 20



typedef enum
{
	ECU = 1,
	BCU,
	BMU,
	AC,
	ACP,
	DCDC
} OTADEVICETYPE;

typedef enum {
    FILE_TYPE_DEB,      // 查找 .deb 文件
    FILE_TYPE_BIN,      // 查找 .bin 文件
    FILE_TYPE_TAR,      // 查找 .tar 或 .tar.gz 等
    FILE_TYPE_IMG,      // 查找 .img 文件
    FILE_TYPE_BAT_ECU,  // 查找 bat_ecu 可执行文件
    FILE_TYPE_CONF_ONLY // 仅查找配置文件（特殊用途）
} file_type_t;

typedef struct
{
	_Atomic unsigned char OTAFileType; //文件类型
	char OTAFilename[OTAFILENAMEMAXLENGTH];//其他ota文件名
	char OTAUdsSblFilename[MAX_FILE_COUNT][OTAFILENAMEMAXLENGTH];//专门给AC使用的
	char OTAUdsFilename[MAX_FILE_COUNT][OTAFILENAMEMAXLENGTH];//专门给AC使用的
	_Atomic unsigned int deviceID;//CAN 设备ID 
	_Atomic unsigned char deviceType;	//设备类型
	_Atomic unsigned char OTAStart;	//OTA 烧文件开始标志
	_Atomic unsigned char multDeviceOTA; //
	_Atomic unsigned char multDeviceOTANum;	//
	_Atomic unsigned char acOTAFlag; //
	_Atomic unsigned char UpDating;	//OTA上传更新文件开始标志。OTA顺序：上传更新文件+烧写文件
} OTAObject;

extern OTAObject g_otactrl;

// ============ 原子变量的 get/set 函数实现 ============

// deviceID 操作
 unsigned int get_ota_deviceID(void);
 void set_ota_deviceID(unsigned int value);

// deviceType 操作  
 unsigned char get_ota_deviceType(void);
 void set_ota_deviceType(unsigned char value);

// OTAStart 操作
 unsigned char get_ota_OTAStart(void);
 void set_ota_OTAStart(unsigned char value);

// multDeviceOTA 操作
 unsigned char get_ota_multDeviceOTA(void);
 void set_ota_multDeviceOTA(unsigned char value);

// multDeviceOTANum 操作
 unsigned char get_ota_multDeviceOTANum(void);
 void set_ota_multDeviceOTANum(unsigned char value);

// acOTAFlag 操作
 unsigned int get_ota_acOTAFlag(void);
 void set_ota_acOTAFlag(unsigned int value);

// UpDating 操作
 unsigned char get_ota_UpDating(void);
 void set_ota_UpDating(unsigned char value);

// OTAFileType 操作
 unsigned char get_ota_OTAFileType(void);
 void set_ota_OTAFileType(unsigned char value);

// ============ 非原子变量的 get/set 函数 ============
// OTAFilename 操作
 const char* get_ota_OTAFilename(void);
 void set_ota_OTAFilename(const char* filename);

// OTAUdsSblFilename 操作
 const char* get_ota_OTAUdsSblFilename(int index);
 void set_ota_OTAUdsSblFilename(int index, const char* filename);

// OTAUdsFilename 操作
 const char* get_ota_OTAUdsFilename(int index);
 void set_ota_OTAUdsFilename(int index, const char* filename);
int unzipfile(char * cp_filepath,unsigned int * error_status, file_type_t file_type);
static int extract_index(const char* section) ;
static int find_ota_files_simple(const char *extract_dir,file_type_t file_type, char *conf_path, size_t conf_len,char *deb_path, size_t deb_len);
static int compute_file_md5(const char *filepath, char *out_md5) ;
static int handler(void* user, const char* section, const char* name,const char* value);
int copy_file(const char *src, const char *dst);
#endif
