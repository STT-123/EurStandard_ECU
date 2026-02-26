#include "bmu_deal.h"
#include "interface/epoll/myepoll.h"
#include "device_drv/bcu_deal/bcu_deal.h"
static int BMU_CAN_FD = -1;
queue_t Queue_BMURevData; // 分机消息队列，用于epoll接收数据存入，防止处理不过来所以用队列，内部使用
my_event_data_t bmuCanEventData = {
    .fd = -1,  // 无效的文件描述符
    .pin = -1, // 无效的引脚
    .fun_handle = NULL,
    .call_back = NULL
};

// 最新值覆盖策略：队列满时清空历史，仅保留当前最新帧
static void bmu_queue_post_latest(const struct can_frame *frame)
{
    if (queue_post(&Queue_BMURevData, (unsigned char *)frame, sizeof(*frame)) == 0) {
        return;
    }
    queue_clear(&Queue_BMURevData);
    if (queue_post(&Queue_BMURevData, (unsigned char *)frame, sizeof(*frame)) != 0) {
        LOG("[BMU CAN] Queue overwrite failed\n");
    } 
}


static void bmu_can_epoll_msg_transmit(void *arg)
{
    struct can_frame can_rev;
    memset(&can_rev, 0, sizeof(struct can_frame));

    if(BMU_CAN_FD < 0){
        return;
    }

    if (HAL_can_read(BMU_CAN_FD, &can_rev, 1) > 0) 
    {
        if(get_ota_OTAStart() == 1){
            uint32_t can_id = can_rev.can_id;// 检查是否是扩展帧
            uint32_t effective_id;
            
            if (can_id & CAN_EFF_FLAG) { 
                effective_id = can_id & CAN_EFF_MASK;// 扩展帧：获取29位ID
            } else {
                effective_id = can_id & CAN_SFF_MASK;// 标准帧：获取11位ID
            }

            uint32_t id_prefix_mask = 0x1FFF0000;  // 高17位的掩码
            uint32_t expected_prefix = 0x18210000;  // 0x18210左移12位


            if ((effective_id & id_prefix_mask) == expected_prefix) {
                // 在OTA 的过程中，可以根据CAN ID进行过滤放在消息队列中，避免在OTA浪费计算
                bmu_queue_post_latest(&can_rev);
            }
        }
        else
        {
            bmu_queue_post_latest(&can_rev);
        }
    }
    else
    {
        if (errno == EBADF) 
        {
            static uint32_t bmu_ebadf_recover_cnt = 0;
            int recover_ret = -1;
            LOG("[BMU] CAN fd is bad, triggering recovery...\n");
            recover_ret = Drv_can_auto_recover(BMU_CAN_DEVICE_NAME, BMU_CAN_BITRATE, &BMU_CAN_FD, bmu_can_epoll_msg_transmit);
            if (recover_ret != 0) {
                bmu_ebadf_recover_cnt++;
                if ((bmu_ebadf_recover_cnt % 20) == 1) {
                    LOG("[BMU] CAN recover failed(ret=%d), cnt=%u\n", recover_ret, bmu_ebadf_recover_cnt);
                }
            }
        }
    }
}

// 初始化
bool bmu_Init(void)
{
    int bind_ret = 0;
    queue_init(&Queue_BMURevData); // 用于接收消息后存入

    bind_ret = Drv_can_bind_interface(BMU_CAN_DEVICE_NAME, BMU_CAN_BITRATE ,&BMU_CAN_FD,bmu_can_epoll_msg_transmit);
    if (bind_ret != 0)
    {
        LOG("[BMU]%s initial bind failed\n", BMU_CAN_DEVICE_NAME);
        return false;
    }
    return true;
}

// 发送can报文
int Drv_bmu_can_send(CAN_MESSAGE *pFrame)
{
    struct can_frame can_frame;
    int retryCount = 0;
    const int maxRetries = 3;
    Convert_CAN_MESSAGE_to_can_frame(pFrame, &can_frame);
    while (retryCount < maxRetries)
    {
        if(BMU_CAN_FD <0){
            return -1;
        }
        if (HAL_can_write(BMU_CAN_FD, &can_frame))
        {
            return 0;
        }
        retryCount++;
        usleep(100);
    }
    if (retryCount >= maxRetries)
    {
        // LOG("[BMU] Drv_can_auto_recover... Drv_bmu_can_send\r\n");
        // Drv_can_auto_recover(BMU_CAN_DEVICE_NAME, BMU_CAN_BITRATE,&BMU_CAN_FD, bmu_can_epoll_msg_transmit);
    }
    return -1;
}

// 发送canfd报文
int Drv_bmu_canfd_send(struct canfd_frame *cansend_data)
{
    int retryCount = 0;
    const int maxRetries = 3;

    while (retryCount < maxRetries)
    {
        if(BMU_CAN_FD <0){
            return -1;
        }
        if (HAL_canfd_write(BMU_CAN_FD, cansend_data))
        {
            return 0;
        }

        retryCount++;
        usleep(100);
    }
    if (retryCount >= maxRetries)
    {
        // LOG("[BMU] Drv_can_auto_recover... Drv_bmu_canfd_send\r\n");
        // Drv_can_auto_recover(BMU_CAN_DEVICE_NAME, BMU_CAN_BITRATE,&BMU_CAN_FD, bmu_can_epoll_msg_transmit);
    }
    return -1;
}
