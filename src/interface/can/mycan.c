
#define _GNU_SOURCE
#include "mycan.h"
#include "interface/log/log.h"
#include "interface/epoll/myepoll.h"
#include "interface/can/libsocketcan.h"

enum CANFrameType
{
	CAN_FRAME_NONE = 0,
	CAN_FRAME_STANDARD = 1,
	CAN_FRAME_FD = 2
};
/*================*/
// 确保互斥锁已经初始化

/*==================*/
bool can_ifconfig_init(const char *device, int bitrate)
{
	// ========== 使用 libsocketcan API ==========
    struct can_ctrlmode cm = {0};
    
    // 0. 先停止接口
    can_do_stop(device);
    
    // 1. 尝试设置CAN FD模式（在接口DOWN状态下）
    if (can_get_ctrlmode(device, &cm) != 0) {
        LOG("can_get_ctrlmode failed");
        return false;
    }
    
    // 2. 设置比特率和采样点
    if (can_set_canfd_bitrates_samplepoint(device, bitrate, 0, bitrate, 0) != 0) {
        LOG("Failed to set CAN FD bitrates");
        return false;
    }
    
    // 3. 启动接口
    if (can_do_start(device) != 0) {
        LOG("Failed to start CAN interface");
        return false;
    }
	return true;
}

bool can_band_init(const char *device, int *fd)
{
	struct ifreq m_ifr;
	struct sockaddr_can m_addr;

	// 创建 CAN 的套接字
	*fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (*fd < 0)
	{
		return false; // 创建套接字失败
	}
	int enable_fd = 1;
	if (setsockopt(*fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0)
	{
		close(*fd);	  // 关闭套接字
		return false; // 设置`CAN FD`模式失败
	}

	snprintf(m_ifr.ifr_name, IFNAMSIZ, "%s", device);
	if (ioctl(*fd, SIOCGIFINDEX, &m_ifr) < 0)
	{
		close(*fd);	  // 关闭套接字
		return false; // 获取接口索引失败
	}

	m_addr.can_family = AF_CAN;
	m_addr.can_ifindex = m_ifr.ifr_ifindex;

	if (bind(*fd, (struct sockaddr *)&m_addr, sizeof(m_addr)) < 0)
	{
		close(*fd);	  // 关闭套接字
		return false; // 绑定失败
	}
	int flags = fcntl(*fd, F_GETFL, 0);
	fcntl(*fd, F_SETFL, flags | O_NONBLOCK);

	return true; // 成功
}


bool HAL_canfd_write(int fd, struct canfd_frame *pFrame)
{
	int len;
	if(pFrame == NULL){
		return false;
	}
	len = write(fd, pFrame, CANFD_MTU);

	if (len == sizeof(struct canfd_frame)){
		return true;
	}else
	{
		switch(errno) {
			case ENETDOWN:
				// LOG("[CAN FD] Write failed: Network interface down\n");
				break;
			case EINVAL:
				LOG("[CAN FD] Write failed: Invalid argument\n");
				break;
			case ENOBUFS:
				{
					//LOG("[CAN FD] Write failed: No buffer space available\n");//缓存满了
					break;
				}
			case EIO:
				LOG("[CAN FD] Write failed: I/O error\n");
				break;
			case ENODEV:
				LOG("[CAN FD] Write failed: No such device\n");
				break;
			default:
				LOG("[CAN FD] Write failed: errno=%d (%s)\n", errno, strerror(errno));
				break;
		}
	}
		return false;
}
bool HAL_can_write(int fd, struct can_frame *pFrame)
{
	int len;
	if(pFrame == NULL){
		return false;
	}
	len = write(fd, pFrame, CAN_MTU);

	if (len == sizeof(struct can_frame)){
		return true;
	}
	else
	{
		switch(errno) {
            case ENETDOWN:
                // LOG("[CAN] Write failed: Network interface down\n");
                break;
            case EINVAL:
                LOG("[CAN] Write failed: Invalid argument\n");
                break;
            case ENOBUFS:
				{
					//LOG("[CAN] Write failed: No buffer space available\n");//缓存满了
					break;
				}
            case EIO:
                LOG("[CAN] Write failed: I/O error\n");
                break;
            case ENODEV:
                LOG("[CAN] Write failed: No such device\n");
                break;
            default:
                LOG("[CAN] Write failed: errno=%d (%s)\n", errno, strerror(errno));
                break;
        }
	}
		return false;
}

int HAL_canfd_read(int fd, struct canfd_frame *pFrame, unsigned int msTimeout)
{
	if(pFrame == NULL){
		return -1;
	}
	int len = read(fd, pFrame, sizeof(struct canfd_frame));
	
	if (len == sizeof(struct can_frame))
	{

		pFrame->flags = 0;
		return 1;
	}
	else if (len == sizeof(struct canfd_frame))
	{
		return 2;
	}
	else
	{
		if (len == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
		{
			// 非阻塞下没有数据，epoll误触发或数据已读完
			return 0; // 无数据
		}
		else
		{
			perror("HAL_canfd_read failed");
			LOG("HAL_canfd_read fd = %d\r\n",fd);
			return -1; // 真实错误，交给上层触发恢复
		}
	}
}

int HAL_can_read(int fd, struct can_frame *pFrame, unsigned int msTimeout)
{
	if(pFrame == NULL){
		return -1;
	}
	int len = read(fd, pFrame, sizeof(struct can_frame));
	if (len == sizeof(struct can_frame))
	{
		return 1; // 成功读取一个标准 CAN 报文
	}
	else
	{
		if (len == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
		{
			// 非阻塞下没有数据，epoll误触发或数据已读完
			return 0; // 无数据
		}
		else
		{
			perror("HAL_can_read failed");
			LOG("HAL_can_read fd = %d\r\n",fd);
			return -1; // 真实错误，交给上层触发恢复
		}
	}
}

/*
 * 函数名称：can_close
 * 功能描述：CAN关闭
 * 输入参数：fd:SOCKET句柄
 * 输出参数：无
 */
void HAL_can_closeEx(int *fd)
{
	if(fd == NULL){
		return;
	}
	if (*fd >= 0)
	{
		close(*fd);
		*fd = -1;
	}
}


