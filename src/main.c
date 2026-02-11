#include <time.h>
#include "interface/bms/bms_simulink/CANFDRcvFcn_BCU.h"
#include "interface/bms/bms_simulink/CANRcvFcn_BMU.h"
#include "interface/log/log.h"
#include "interface/epoll/myepoll.h"
#include "interface/setting/ip_setting.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/bmu_deal/bmu_deal.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"
#include "function_task/bcu_task/bcu_task.h"
#include "function_task/bmu_task/bmu_task.h"
#include "function_task/ota_task/otaupgrad_task.h"
#include "function_task/xmodem_task/xmodem_task.h"
#include "function_task/abnormal_check_task/abnormal_check_task.h"
#include "device_drv/abncheck/abncheck.h"
#include "ocpp_send.h"
extern ecu_fault_t ecu_fault ;
static void printf_version(void)
{
    char compile_date[12] = {0}, compile_time[20] = {0};
    sprintf(compile_date, "%s", __DATE__);
    sprintf(compile_time, "%s", __TIME__);
    LOG("========================================================= \n");
    LOG("[VERSION] BAT ECU_EU START RUN!!!. \n");
    LOG("[VERSION] Software compilation time %s--%s. \n", compile_date, compile_time);
    LOG("========================================================= \n");
}


void crash_handler(int sig) {
    void *array[20];
    size_t size;
    
    // 区分信号类型
    if (sig == SIGINT || sig == SIGTERM) {
        // 正常退出信号
        printf("\nProgram exit... (sign: %d)\n", sig);
        
        if (sig == SIGINT) {
            printf("reason: Ctrl+C \n");
        } else {
            printf("reason: termination signal\n");
        }
        
        // 清理资源
        // close_all_connections();
        
        exit(0);  // 正常退出码
    }
    else {
        // 真正的崩溃信号
        LOG("!!! Program crash !!!\r\n");
        switch(sig) {
            case SIGSEGV: LOG("Segmentation fault (null pointer/memory overflow)\n"); break;
            case SIGABRT: LOG("Program abort (assert/abort call)\n"); break;
            case SIGBUS:  LOG("Bus error (memory alignment issue)\n"); break;
            case SIGFPE:  LOG("Arithmetic exception (division by zero, etc.)\n"); break;
            case SIGILL:  LOG("Illegal instruction\n"); break;
            default:      LOG("Unknown error\n"); break;
        }
        
        LOG("Crash stack trace:\n");// 获取堆栈跟踪
        size = backtrace(array, 20);
        if(size >= 3){
            for (size_t i = 0; i < size; i++) {
                uintptr_t addr = (uintptr_t)array[i];
                LOG("Error Addr[%d] = 0x%x\r",i,addr);
            }
        }
        backtrace_symbols_fd(array, size, STDERR_FILENO);
        
        // fprintf(stderr, "\n调试建议:\n");
        // fprintf(stderr, "1. 使用地址信息定位问题\n");
        // fprintf(stderr, "2. 检查 0x404198 附近的代码\n");
        
        exit(1);  // 异常退出码
    }
}

void setup_crash_handler() {
    signal(SIGSEGV, crash_handler);  // 段错误
    signal(SIGABRT, crash_handler);  // 中止
    signal(SIGBUS, crash_handler);   // 总线错误
    signal(SIGFPE, crash_handler);   // 浮点异常
    signal(SIGILL, crash_handler);   // 非法指令
    signal(SIGTERM, crash_handler);  // 终止信号
    signal(SIGINT, crash_handler);   // Ctrl+C中断信号 ← 添加这一行！
    
    printf("Crash handler installed\n");
}
/*
加一个读取判断是否非法0。0。0，255等等
查一下ftp会自己复制，或者自己复制一份
代码升级任务--加一个过滤ID 
增加记录BCU_info2
*/
int main(int argc, char **argv)
{
    setup_crash_handler();
    /*=================接口初始化部分================*/
    log_init();// 日志初始化
    printf_version();//初始打印
    settings_Init();// 判断本机IP 如果不存在 默认使用110---加一个读取判断是否非法0。0。0，255等等
    my_epoll_Init(); // 初始化epoll环境
   
    /*=================任务初始化部分================*/
    bcu_DealTaskCreate();
    bmu_DealTaskCreate();
    modbusTcpServerTaskCreate();//moduTCP服务
    ota_Upgrade_TaskCreate();//代码升级任务--加一个过滤ID 
    xmodemCommTaskCreatee();//监听OTA 存储升级文件Xmodem协议
    SDCardDataSaveTaskCreate(); // SD卡写任务
    abnormalDetectionTaskCreate(); // 异常监测任务
    // ocppCommunicationTaskCreate(); //ocpp通信任务
    FtpServiceThreadCreate();//查一下ftp会自己复制，或者自己复制一份
    int index1 = 0;

    while(1)
    {
        sleep(1);
        // main_test();
        // printf("DAqX_FaultCode1 = %d\r\n", DAqX_FaultCode1[0]);//一级故障
        // printf("data.get_usBatMaxTempCellIndex = %ld\n", get_usBatMaxTempCellIndex());
        // printf("data. get_usBatCellTempMax( = %d\n",  get_usBatCellTempMax());
        // printf("data. get_usBatMinTempCellIndex( = %d\n",  get_usBatMinTempCellIndex());
        // printf("data.get_usBatCellTempMin = %ld\n", get_usBatCellTempMin());

        // printf("data. get_usBatMaxVoltCellIndex( = %d\n",  get_usBatMaxVoltCellIndex());
        // printf("data.get_usBatCellVoltMax = %ld\n", get_usBatCellVoltMax());
        // printf("data.get_usBatMinVoltCellIndex = %ld\n", get_usBatMinVoltCellIndex());
        // printf("data.get_usBatCellVoltMin = %ld\n", get_usBatCellVoltMin());

        // printf("get_ota_UpDating(): %d\r\n",get_ota_UpDating());
        // printf("get_BCUFD() = %d\r\n",get_BCU_CAN_FD());
        // printf("main printf sleep(1) \r\n");
        // set_emcu_fault(PHY_LINK_FAULT, SET_ERROR);
        // index1 = 0x4B40 - 0x3000;
        // printf("Mobud[0x4B40] = 0x %x\r\n",modbusBuff[index1]);//ota上载寄存器判断
        // index1 = 0x4B5D - 0x3000;
        // printf("Mobud[0x4B5D] = %x\r\n",modbusBuff[index1]);//ota上载寄存器判断
        // printf("BCU_SystemWorkMode = %x\r\n",BCU_SystemWorkMode);//ota上载寄存器判断
        // printf("BCU_FaultInfoLv1 = %x\r\n",BCU_FaultInfoLv1);//ota上载寄存器判断
        // printf("BCU_FaultInfoLv2 = %x\r\n",BCU_FaultInfoLv2);//ota上载寄存器判断    
    }
}

void main_test(void){

    CAN_MESSAGE CanMes;
	memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
	CanMes.Extended = 1;
	CanMes.Length = 1;
	CanMes.ID = 111;
	CanMes.Data[0] = 0xCF;
    int counter = 0;
    // 测试堆内存泄漏
    // size_t size = 2 * 1024 * sizeof(int);  // 8KB
    // int* ptr = malloc(size);
    
    // if (ptr != NULL) {
    //     // 关键：写入数据，强制分配物理内存
    //     memset(ptr, 0xFF, size);  // 填充数据
    //     // 记录分配信息（可选）
    //     ptr[0] = counter++;
    // }

    // // 测试句柄无限增长 
    // FILE *rfile = NULL;
    // rfile = fopen("/mnt/sda/xx.txt", "rb");  // "rb" = 只读，二进制
    // if (rfile == NULL)
    // {
    //     printf("xxx\r\n");
    // }else{
    //     printf("ok\r\n");
    // }

    while(1){
        // Drv_bmu_can_send(&CanMes);
        sleep(1); 
    }
}