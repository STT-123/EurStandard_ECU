#include "bcu_deal.h"
#include "interface/log/log.h"
#include "interface/epoll/myepoll.h"
#include "device_drv/abncheck/abncheck.h"
#include "device_drv/bmu_deal/bmu_deal.h"
#include "device_drv/sd_store/sd_store.h"

static int BCU_CAN_FD = -1;
queue_t Queue_BCURevData;
queue_t Queue_BCURevData_FD;
/*======================================静态函数==================================================*/
my_event_data_t bcuCanEventData = {
.fd = -1,  // 无效的文件描述符
.pin = -1, // 无效的引脚
.fun_handle = NULL,
.call_back = NULL
};

extern my_event_data_t bmuCanEventData ;
static pthread_mutex_t can_recover_mutex = PTHREAD_MUTEX_INITIALIZER; //恢复锁，当需要can复位的时候，避免两个任务都复位

// 最新值覆盖策略：队列满时清空历史，仅保留当前最新帧
static void bcu_queue_post_latest_can(const struct can_frame *frame)
{
    if (queue_post(&Queue_BCURevData, (unsigned char *)frame, sizeof(*frame)) == 0) {
        return;
    }
    queue_clear(&Queue_BCURevData);
    if (queue_post(&Queue_BCURevData, (unsigned char *)frame, sizeof(*frame)) != 0) {
        LOG("[BCU CAN] Queue overwrite failed\n");
    } 
}

static void bcu_queue_post_latest_canfd(const struct canfd_frame *frame)
{
    if (queue_post(&Queue_BCURevData_FD, (unsigned char *)frame, sizeof(*frame)) == 0) {
        return;
    }
    queue_clear(&Queue_BCURevData_FD);
    if (queue_post(&Queue_BCURevData_FD, (unsigned char *)frame, sizeof(*frame)) != 0) {
        LOG("[BCU CANFD] Queue overwrite failed\n");
    } 
}
static void bcu_can_epoll_msg_transmit(void *arg)
{
    struct canfd_frame canfd_rev ;
    struct can_frame can_rev ;

    memset(&canfd_rev, 0, sizeof(struct canfd_frame));
    memset(&can_rev, 0, sizeof(struct can_frame));

    if(BCU_CAN_FD <0){
        return;
    }
    int frame_type = HAL_canfd_read(BCU_CAN_FD, &canfd_rev, 1);

    if (frame_type == 1 || frame_type == 2) {time(&g_last_bcu_rx_time);}
    if (frame_type == 1)//1 表示CAN 数据-8
    {
        Convert_canfd_frame_to_can_fram(&canfd_rev, &can_rev);//把canfd转换成can
        // 在OTA 的过程中，可以根据CAN ID进行过滤放在消息队列中，避免在OTA浪费计算
        bcu_queue_post_latest_can(&can_rev);
    }
    else if (frame_type == 2)//2    表示CAN FD数据-64
    {
        // 往CAN FD队列放数据，队列满时保留最新值
        bcu_queue_post_latest_canfd(&canfd_rev);
    }
    else if(frame_type < 0)
    {
        if (errno == EBADF) 
        {
            static uint32_t bcu_ebadf_recover_cnt = 0;
            int recover_ret = -1;
            LOG("[BCU] CAN fd is bad, triggering recovery...\n");
            recover_ret = Drv_can_auto_recover(BCU_CAN_DEVICE_NAME, BCU_CAN_BITRATE, &BCU_CAN_FD, bcu_can_epoll_msg_transmit);
            if (recover_ret != 0) {
                bcu_ebadf_recover_cnt++;
                if ((bcu_ebadf_recover_cnt % 20) == 1) {
                    LOG("[BCU] CAN recover failed(ret=%d), cnt=%u\n", recover_ret, bcu_ebadf_recover_cnt);
                }
            }
        }
    }
}


/*======================================外部函数==================================================*/
bool bcu_Init(void)
{
    int bind_ret = 0;

    queue_init(&Queue_BCURevData);    // 用于接收消息后存入
    queue_init(&Queue_BCURevData_FD); // 用于接收消息后存入

    bind_ret = Drv_can_bind_interface(BCU_CAN_DEVICE_NAME, BCU_CAN_BITRATE ,&BCU_CAN_FD,bcu_can_epoll_msg_transmit);
    if (bind_ret != 0)
    {
        LOG("[BCU]%s initial bind failed\n", BCU_CAN_DEVICE_NAME);
        return false;
    }
    
    return true;
}


int Drv_bcu_can_send(CAN_MESSAGE *pFrame)
{
    struct can_frame can_frame;
    int retryCount = 0;
    const int maxRetries = 5;

    Convert_CAN_MESSAGE_to_can_frame(pFrame, &can_frame);
    while (retryCount < maxRetries)
    {
        if(BCU_CAN_FD <0){
            return -1;
        }
        if (HAL_can_write(BCU_CAN_FD, &can_frame))
        {
            return 0;
        }

        retryCount++;
    }
    if (retryCount >= maxRetries)//软件层面发送失败，系统的can底层有问题了
    {
        // LOG("[BCU]%s retryCount error\r\n", BCU_CAN_DEVICE_NAME);
    }

    return -1;
}

int Drv_bcu_canfd_send(CAN_FD_MESSAGE_BUS *pFrame)
{
    struct canfd_frame canfd_frame;
    int retryCount = 0;
    const int maxRetries = 5;

    Drv_write_to_active_buffer(pFrame, 0); // 0709添加
    
    ConvertBusToCANFD(pFrame, &canfd_frame);

    while (retryCount < maxRetries)
    {
        if(BCU_CAN_FD <0){
            return -1;
        }
        if (HAL_canfd_write(BCU_CAN_FD, &canfd_frame))
        {
            return 0;
        }
        retryCount++;
    }
    if (retryCount >= maxRetries)
    {
        // LOG("[BCUFD]%s retryCount error\r\n", BCU_CAN_DEVICE_NAME);
    }
    return -1;
}



/**
 * @brief 重新绑定CAN接口（包含epoll管理）
 * @param can_name CAN设备名
 * @param bitrate CAN比特率
 * @param can_fd_ptr 指向CAN文件描述符的指针
 * @param callback CAN消息回调函数
 * @return 成功返回0，失败返回-1
 */
int Drv_can_bind_interface(const char *can_name, int bitrate, int *can_fd_ptr,
                           void (*callback)(void *arg))
{
    LOG("[CAN]Rebinding %s interface...\n", can_name);

    // 1. 清理旧的epoll注册和文件描述符
    if (*can_fd_ptr >= 0) 
    {
        struct epoll_event ev;
        my_epoll_deltast(*can_fd_ptr, &ev); // 从epoll中删除
        HAL_can_closeEx(can_fd_ptr);        // 关闭旧的CAN socket
        *can_fd_ptr = -1;
        LOG("[CAN]%s old resources cleaned\n", can_name);
    }

    // 2. 重启物理CAN接口,重新初始化CAN配置

    if (can_ifconfig_init(can_name, bitrate) == false) {
        LOG("[CAN]%s can_ifconfig_init failed\n", can_name);
        return -1;
    }

    // 3. 重新绑定CAN设备
    int retry_count = 0;
    const int max_retry = 5;
    
    while (can_band_init(can_name, can_fd_ptr) == false) {
        retry_count++;
        if (retry_count >= max_retry) {
            LOG("[CAN]%s can_band_init failed after %d retries\n", 
                can_name, max_retry);
            return -1;
        }
        LOG("[CAN]%s can_band_init failed, retrying %d/%d...\n", 
            can_name, retry_count, max_retry);
        sleep(1);
    }

    LOG("[CAN]%s rebound successfully, new fd: %d\n", can_name, *can_fd_ptr);


    // 4. 重新注册到epoll
    struct epoll_event ev;
    ev.events = EPOLLIN;


    if(strcmp(can_name, BCU_CAN_DEVICE_NAME) == 0){
        bcuCanEventData.fd = *can_fd_ptr;  // 设置新的文件描述符
        bcuCanEventData.fun_handle = callback;  // 使用传入的回调函数
	    ev.data.ptr = (void *)&bcuCanEventData;
    }else if(strcmp(can_name, BMU_CAN_DEVICE_NAME) == 0){
        bmuCanEventData.fd = *can_fd_ptr;  // 设置新的文件描述符
        bmuCanEventData.fun_handle = callback;  // 使用传入的回调函数
        ev.data.ptr = (void *)&bmuCanEventData;
    }else{
        return -1;
    }

    if (-1 == my_epoll_addtast(*can_fd_ptr, &ev)) {
        LOG("[CAN]%s re-add to epoll failed\n", can_name);// 即使epoll注册失败，也不完全算失败，因为CAN已经重新绑定了     
        return -2;
    }

    LOG("[CAN]%s fully recovered and re-registered to epoll\n", can_name);
    return 0;
}



/**
 * @brief 检测并自动恢复CAN接口状态
 * @param can_name CAN设备名（如 "can2"）
 * @param bitrate CAN比特率
 * @param can_fd_ptr 指向CAN文件描述符的指针
 * @param callback CAN消息回调函数
 * @return 成功返回0，失败返回-1
 */
int Drv_can_auto_recover(const char *can_name, int bitrate, int *can_fd_ptr, 
                        void (*callback)(void *arg))
{
    if (can_name == NULL || can_fd_ptr == NULL || callback == NULL) {
        return -1;
    }

    // 加锁 - 确保同一时间只有一个任务执行恢复操作
    if (pthread_mutex_lock(&can_recover_mutex) != 0) {
        LOG("[CAN]%s Failed to acquire lock\n", can_name);
        return -1;
    }

    bool need_rebind = false;
    int ret = -1;

    // 0. fd异常优先重绑（如EBADF）
    if (*can_fd_ptr < 0)
    {
        LOG("[CAN]%s fd invalid(%d), force rebind\n", can_name, *can_fd_ptr);
        need_rebind = true;
    }

    // 1. 检查物理链路状态
    if (!need_rebind)
    {
        if (check_can_state_detailed(can_name) <= 0)
        {
            LOG("[CAN]%s physical link is DOWN, need rebind\n", can_name);
            need_rebind = true;
        }
        else
        {
            // printf("[CAN]%s physical link is UP, not need rebind\n", can_name);
            ret = 0;
            goto unlock; // 使用goto确保锁被释放
        }
    }

    // 如果需要重新绑定
    if (need_rebind) 
    {
        ret = Drv_can_bind_interface(can_name, bitrate, can_fd_ptr, callback);
    }
unlock:
    // 确保锁被释放
    pthread_mutex_unlock(&can_recover_mutex);
    return ret;
}

int get_BCU_CAN_FD(void){
    return BCU_CAN_FD;
}
