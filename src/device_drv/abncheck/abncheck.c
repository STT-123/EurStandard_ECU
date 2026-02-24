#include <time.h>
#include <netdb.h>  
#include <features.h>
#include "interface/log/log.h"
#include "interface/bms/bms_analysis.h"
#include "device_drv/abncheck/abncheck.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/bmu_deal/bmu_deal.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"
#include "interface/setting/ip_setting.h"
#include "interface/bms/bms_simulink/CANFDRcvFcn_BCU.h"
#include "interface/bms/bms_simulink/CANRcvFcn_BMU.h"
#include "modbus_defines.h"
#define _POSIX_C_SOURCE 199309L
#define RECOVER_REPORT_TIME 5000
#define FAULT_REPORT_TIME 3000
#define RECOVER_KM_DEFAULT_STATE 0 // 恢复接触器默认状态
#define RECOVER_KM_ACTION_STATE 1  // 恢复接触器动作状态

int PHY_RECOVER = 0;
int PHY_ERROR = 0;
uint8_t ECUState = 0;
int RECOVER = 0;
int ERROR = 0;

ecu_fault_t ecu_fault ={0};
ecu_fault_t ecu_fault_last = {0};
static int g_bcu_can_ready = 0;
static int g_bmu_can_ready = 0;
/*----------------------*/
struct timespec lasttimes ;
struct timespec lastCheckTick = {0};
time_t g_last_bcu_rx_time = 0;
/*--------------*/
static const fault_mapping_t fault_map_4H[] = {
    {23, EMERGENCY_STOP},//BCU急停
};

static const fault_mapping_t fault_map_3H[] = {
    {8, PCS_STOP},//外部急停(充电打桩)
    {9, ISO_SWITCH_FAULT},//隔开开关故障
    {10,DOOR_OPEN},//门禁故障
};

static const fault_mapping_t fault_map_2H[] = {
    {6, INSIDE_NTC_FAULT},//内部温度故障
    {7, OUTSIDE_COM_FAULT},//外部温度故障
};


int CheckSinglePHYStatus(const char *ifname)
{
	FILE *fp;
	char line[512];
	char devname[32];
	net_stats_t stats = {0};
	static net_stats_t last_stats[2] = {0}; // 简单缓存上次统计
	static int initialized = 0;

	fp = fopen("/proc/net/dev", "r");
	if (!fp)
	{
		perror("fopen /proc/net/dev failed");
		return 0; // 没有
	}

	// 跳过前两行标题
	fgets(line, sizeof(line), fp);
	fgets(line, sizeof(line), fp);

	while (fgets(line, sizeof(line), fp))
	{
        if (strlen(line) >= sizeof(line) - 1) {
            LOG("[Abnormal] Line too long in /proc/net/dev\n");
            continue;
        }
		// 解析接口统计信息
		if (sscanf(line, "%31[^:]: %llu %llu %*u %*u %*u %*u %*u %*u %llu %llu",
				   devname, &stats.rx_bytes, &stats.rx_packets,
				   &stats.tx_bytes, &stats.tx_packets) >= 4)
		{

			// 去除接口名末尾的空格
			char *p = devname;
			while (*p == ' ')
				p++;

			if (strcmp(p, ifname) == 0)
			{
				fclose(fp);

				// 检查是否有数据活动
				int has_traffic = 0;

				if (initialized)
				{
					// 比较当前统计和上次统计
					net_stats_t *last = &last_stats[strcmp(ifname, "eth0") == 0 ? 0 : 1];
					if (stats.rx_bytes > last->rx_bytes ||
						stats.tx_bytes > last->tx_bytes ||
						stats.rx_packets > last->rx_packets ||
						stats.tx_packets > last->tx_packets)
					{
						has_traffic = 1;
					}
				}

				// 更新缓存
				net_stats_t *last = &last_stats[strcmp(ifname, "eth0") == 0 ? 0 : 1];
				*last = stats;
				initialized = 1;

				return has_traffic;
			}
		}
	}

	fclose(fp);
	return 0; // 接口未找到
}

void PHYlinktate()
{
	static struct timespec phy_last_check_tick = {0};
	clock_gettime(CLOCK_MONOTONIC, &phy_last_check_tick); // 记录lastCheckTick初始时间
	static int PHY_RECOVER_FLAG = 0;
	static int PHY_ERROR_FLAG = 0;

	int eth1_status = 0;

	// 检查eth1
	eth1_status = CheckSinglePHYStatus(NET_ETH_1);//检测网线口十分有数据流量活动，代码只支持eth0和eth1检测，其他不支持。

	// LOG("TTTTTTTTTTTTTTTTT eth0_status = %d,  eth1_status=%d ", eth0_status, eth1_status);
	// eth1_status = 1;

	if (eth1_status) // 连接
	{
		if (PHY_RECOVER_FLAG == 1)
		{
			if (GetTimeDifference_ms(phy_last_check_tick) >= RECOVER_REPORT_TIME)
			{
				set_emcu_fault(PHY_LINK_FAULT, SET_RECOVER);
				PHY_ERROR_FLAG = 0;
			}
		}
		else
		{
			clock_gettime(CLOCK_MONOTONIC, &phy_last_check_tick);
			PHY_RECOVER_FLAG = 1;
			PHY_ERROR_FLAG = 0;
		}
	}
	else // 未连接就报故障
	{
		if (PHY_ERROR_FLAG == 1)
		{
			if (GetTimeDifference_ms(phy_last_check_tick) >= FAULT_REPORT_TIME)
			{
				set_emcu_fault(PHY_LINK_FAULT, SET_ERROR);
				PHY_RECOVER_FLAG = 0;
			}
		}
		else
		{
			clock_gettime(CLOCK_MONOTONIC, &phy_last_check_tick);
			PHY_ERROR_FLAG = 1;
			PHY_RECOVER_FLAG = 0;
		}
	}
}


void ECUfault_process()
{
	int temp = 0;
	set_modbus_reg_val(MDBUS_ADDR_BECU_FAULT0, ecu_fault.emcu_fault0);
	set_modbus_reg_val(MDBUS_ADDR_BECU_FAULT1, ecu_fault.emcu_fault1);
	set_modbus_reg_val(MDBUS_ADDR_BECU_FAULT2, ecu_fault.emcu_fault2);
	set_modbus_reg_val(MDBUS_ADDR_BECU_FAULT3, ecu_fault.emcu_fault3);

	// printf("MDBUS_ADDR_BECU_FAULT0 = 0x%x\r\n",ecu_fault.emcu_fault0);
	// printf("MDBUS_ADDR_BECU_FAULT0 = 0x%x\r\n",ecu_fault.emcu_fault1);
	// printf("MDBUS_ADDR_BECU_FAULT0 = 0x%x\r\n",ecu_fault.emcu_fault2);
	// printf("MDBUS_ADDR_BECU_FAULT0 = 0x%x\r\n",ecu_fault.emcu_fault3);

	// get_modbus_reg_val(MDBUS_ADDR_BECU_FAULT0, &temp);
	// printf("MDBUS_ADDR_BECU_FAULT0 = 0x%x\r\n",temp);
	// get_modbus_reg_val(MDBUS_ADDR_BECU_FAULT1, &temp);
	// printf("MDBUS_ADDR_BECU_FAULT1 = 0x%x\r\n",temp);
	// get_modbus_reg_val(MDBUS_ADDR_BECU_FAULT2, &temp);
	// printf("MDBUS_ADDR_BECU_FAULT2 = 0x%x\r\n",temp);
	// get_modbus_reg_val(MDBUS_ADDR_BECU_FAULT3, &temp);
	// printf("MDBUS_ADDR_BECU_FAULT3 = 0x%x\r\n",temp);
}


/********************************************************************************
 *
 * 输入参数：
 *                      unsigned int parameter   参数 //詳見fault_intaface.h
 *                      unsigned char status        分机的状态 1 0
 * 			无
 * 输出参数：无
 ********************************************************************************/
void set_emcu_fault(unsigned char parameter, unsigned char status)
{

	unsigned char byte_num = (parameter & 0xf0) >> 4; // 高4位字节号
	unsigned short bit_num = (parameter & 0x0F);	  //  低4位bit位

	switch (byte_num)
	{
	case 0:
		if (status)
		{
			ecu_fault.emcu_fault0 &= ~(1 << bit_num);
		}
		else
		{
			ecu_fault.emcu_fault0 |= (1 << bit_num);
		}
		break;
	case 1:
		if (status)
		{
			ecu_fault.emcu_fault1 &= ~(1 << bit_num);
		}
		else
		{
			ecu_fault.emcu_fault1 |= (1 << bit_num);
		}
		break;
	case 2:
		if (status)
		{
			ecu_fault.emcu_fault2 &= ~(1 << bit_num);
		}
		else
		{
			ecu_fault.emcu_fault2 |= (1 << bit_num);
			
		}
		break;
	case 3:
		if (status)
		{
			ecu_fault.emcu_fault3 &= ~(1 << bit_num);
		}
		else
		{
			ecu_fault.emcu_fault3 |= (1 << bit_num);
		}
		break;
	default:
		break;
	}
	if (ecu_fault.emcu_fault0 + ecu_fault.emcu_fault1 + ecu_fault.emcu_fault2)
	{
		ecu_fault.emcu_fault_state = 1;
	}
	else
	{
		ecu_fault.emcu_fault_state = 0;
	}
}

/**
 * 检测BCU的通信是否超时函数
*/
void check_bcu_rx_timeout(void)
{
	static bool can0_fault_reported = false;
	time_t current_time;
	time(&current_time);
	double diff = difftime(current_time, g_last_bcu_rx_time);

	if (diff >= 5.0)
	{
		if (!can0_fault_reported)
		{
			set_emcu_fault(BMS_COM_FAULT, SET_ERROR);
			can0_fault_reported = true;
			LOG("CAN0 TimeoutCheck warning\n");
		}
	}
	else
	{
		if (can0_fault_reported)
		{
			set_emcu_fault(BMS_COM_FAULT, SET_RECOVER);
			can0_fault_reported = false;
			LOG("CAN0 TimeoutCheckv normal\n");
		}
	}
}

/**
 * 检测CAN 是否异常函数
*/
int can_monitor_fun(void) {
    // ============ can2 处理 ============
    int bcu_can_state = check_can_state_detailed(BCU_CAN_DEVICE_NAME);
    
    // 分离逻辑：检测到任何异常都处理
    if (bcu_can_state <= 0)  // 0或负数都表示异常
    { 
        static time_t last_restart_time_can2 = 0;
        time_t now = time(NULL);
        
        // 避免频繁重启（至少间隔5秒）
        if (now - last_restart_time_can2 > 5) 
        {          
            LOG("[CHECK] Restarting can2...\n");
            restart_can_interface_enhanced(BCU_CAN_DEVICE_NAME);
            last_restart_time_can2 = now;
            
            // 重置标志，等待恢复
            g_bcu_can_ready = 0;
        } else {
            LOG("[CHECK] can2 abnormal but restart cooldown (%lds)\n",
                5 - (now - last_restart_time_can2));
        }
    } 
    else 
    {
        // 正常状态
        if (g_bcu_can_ready == 0) {
            LOG("[CHECK] can2 recovered\n");
        }
        g_bcu_can_ready = 1;
    }

    // ============ can3 处理（统一逻辑） ============
    int bmu_can_state = check_can_state_detailed(BMU_CAN_DEVICE_NAME);
    
    // 使用完全相同的逻辑处理can3
    if (bmu_can_state <= 0)  // 0或负数都表示异常
    { 
        static time_t last_restart_time_can3 = 0;
        time_t now = time(NULL);
        
        // 避免频繁重启（至少间隔5秒）
        if (now - last_restart_time_can3 > 5) 
        {           
            LOG("[CHECK] Restarting can3...\n");
            restart_can_interface_enhanced(BMU_CAN_DEVICE_NAME);
            last_restart_time_can3 = now;
            
            // 重置标志，等待恢复
            g_bmu_can_ready = 0;
        } else {
            LOG("[CHECK] can3 abnormal but restart cooldown (%lds)\n",
                5 - (now - last_restart_time_can3));
        }
    } 
    else 
    {
        // 正常状态
        if (g_bmu_can_ready == 0) {
            LOG("[CHECK] can3 recovered\n");
        }
        g_bmu_can_ready = 1;
    }
}
void restart_can_interface_enhanced(const char* can_if) {
    struct can_ctrlmode cm = {0};
    
    // 0. 先停止接口
    can_do_stop(can_if);
    
    // 1. 尝试设置CAN FD模式（在接口DOWN状态下）
    if (can_get_ctrlmode(can_if, &cm) != 0) {
        LOG("can_get_ctrlmode failed");
        return;
    }

    // 2. 设置比特率和采样点
    if (can_set_canfd_bitrates_samplepoint(can_if, 500000, 0, 500000, 0) != 0) {
        LOG("Failed to set CAN FD bitrates");
        return;
    }
    
    // 3. 启动接口
    if (can_do_start(can_if) != 0) {
        LOG("Failed to start CAN interface");
        return;
    }
}

int check_can_state_detailed(const char* can_if) {
 
    int can_state = 0;  
	int can_up = 0; 
    can_get_state(can_if,&can_state);
	can_up = get_can_interface_state(can_if);
    if((can_state == CAN_STATE_BUS_OFF )|| (can_state == CAN_STATE_STOPPED) || (can_up == 0)){
        LOG("[CHECK] %s is in ERROR , can_state= %d, can_up= %d\n", can_if,can_state,can_up);
        return -2;  // 特殊返回码表示BUS-OFF
    }  
    return 1;
}

// 主业务判断函数
int is_bcu_can_ready(void) {
    return g_bcu_can_ready;
}
int is_bmu_can_ready(void) {
    return g_bmu_can_ready;
}


void get_BCU_FaultInfo(uint32_T faultValue_4H, uint32_T faultValue_3H,uint32_T faultValue_2H)
{
    unsigned int i = 0;
    //故障映射，新增故障只需要更改fault_map_4H
	// printf("faultValue_4H = %x\n", faultValue_4H);
    // printf("faultValue_3H = %x\n", faultValue_3H);
    // printf("faultValue_2H = %x\n", faultValue_2H);
    for ( i = 0; i < sizeof(fault_map_4H)/sizeof(fault_map_4H[0]); i++) 
    {
        if (faultValue_4H & (1UL << fault_map_4H[i].bit_position))
        {
            set_emcu_fault(fault_map_4H[i].fault_type, SET_ERROR);
        } 
        else 
        {
            set_emcu_fault(fault_map_4H[i].fault_type, SET_RECOVER);
        }
    }
    //故障映射，新增故障只需要更改fault_map_3H
    for ( i = 0; i < sizeof(fault_map_3H)/sizeof(fault_map_3H[0]); i++) 
    {
        if (faultValue_3H & (1UL << fault_map_3H[i].bit_position))
        {
            set_emcu_fault(fault_map_3H[i].fault_type, SET_ERROR);
        } 
        else 
        {
            set_emcu_fault(fault_map_3H[i].fault_type, SET_RECOVER);
        }
    }
    
    //故障映射，新增故障只需要更改fault_map_2H
    for ( i = 0; i < sizeof(fault_map_2H)/sizeof(fault_map_2H[0]); i++) 
    {
        if (faultValue_2H & (1UL << fault_map_2H[i].bit_position))
        {
            set_emcu_fault(fault_map_2H[i].fault_type, SET_ERROR);
        } 
        else 
        {
            set_emcu_fault(fault_map_2H[i].fault_type, SET_RECOVER);
        }
    }
}
/**
 * @brief 检测是否能ping通指定主机
 * @param hostname 主机名或IP地址
 * @param timeout_sec 超时时间（秒）
 * @return 1: 可ping通, 0: 不可ping通, -1: 执行错误
 */
int can_ping_host(const char *hostname, int timeout_sec) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) return -1;

    int result = -1;
    do {
        // 所有操作在此块内完成
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(80);

        struct hostent *server = gethostbyname(hostname);
        if (!server) break;

        memcpy(&addr.sin_addr.s_addr, server->h_addr_list, server->h_length);

        struct timeval tv = { .tv_sec = timeout_sec };
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        result = (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == 0) ? 1 : 0;
    } while (0);

    close(sockfd);
    return result;
}
int check_and_fix_ip(const char *if_name)
{
	unsigned char expected_ip[16] = IP_ADDRESS;//

    char command[256];
    FILE *fp;
    char buffer[1024];
    char current_ip[16] = {0};
    int need_fix = 1;
    
    //LOG("[IP自动修复] 检查接口 %s 的IP状态\n", if_name);


    int len = snprintf(expected_ip, sizeof(expected_ip), "%d.%d.%d.%d",
                    (g_ipsetting.ip >> 24) & 0xFF,
                    (g_ipsetting.ip >> 16) & 0xFF,
                    (g_ipsetting.ip >> 8) & 0xFF,
                    g_ipsetting.ip & 0xFF);
    if (len < 0 || (size_t)len >= sizeof(expected_ip)) {
        LOG("Failed to format IP address\n");
        return -1;
    }
    // 检测当前IP
    snprintf(command, sizeof(command), 
             "ip -4 addr show %s 2>/dev/null | grep -oE '([0-9]{1,3}\\.){3}[0-9]{1,3}/' | head -1 | sed 's|/||'", 
             if_name);
    
    fp = popen(command, "r");
    if (fp != NULL) {
        if (fgets(buffer, sizeof(buffer), fp) != NULL) {
            buffer[strcspn(buffer, "\n")] = 0;
            char *trimmed = buffer;
            while (*trimmed == ' ') trimmed++;
            
            if (strlen(trimmed) > 0) {
                strncpy(current_ip, trimmed, sizeof(current_ip) - 1);
                
                if (strcmp(current_ip, expected_ip) == 0) {
                    need_fix = 0;
                }
            }
        }
        pclose(fp);
    }
    
    // 如果需要修复
    if (need_fix) {
		// 打印当前不正确的IP
		if (strlen(current_ip) > 0) {
			LOG("[IP] The current IP address is incorrect: %s, Expected IP: %s\n", current_ip, expected_ip);
		} else {
			LOG("[IP] Current IP not detected, expected IP: %s\n", expected_ip);
		}
        LOG("[IP] IP incorrect, start modifying...\n");
        
        int ret = set_ip_address(if_name, expected_ip);// 调用set_ip_address函数修改IP

        if (ret == 0) {
            LOG("[IP] IP modification successful\n");
            sleep(2);// 修改后验证
            memset(current_ip, 0, sizeof(current_ip));
            snprintf(command, sizeof(command), 
                     "ip -4 addr show %s 2>/dev/null | grep -oE '([0-9]{1,3}\\.){3}[0-9]{1,3}/' | head -1 | sed 's|/||'", 
                     if_name);
            
            fp = popen(command, "r");
            if (fp != NULL && fgets(buffer, sizeof(buffer), fp) != NULL) {
                buffer[strcspn(buffer, "\n")] = 0;
                char *trimmed = buffer;
                while (*trimmed == ' ') trimmed++;
                
                if (strlen(trimmed) > 0) {
                    strncpy(current_ip, trimmed, sizeof(current_ip) - 1);
                    if (strcmp(current_ip, expected_ip) == 0) {
                        LOG("[IP] Verified successfully after modification\n");
                        return 0;
                    } else {
                        LOG("[IP] Verification failed after modification. Current IP address: %s\n", current_ip);
                        return -1;
                    }
                }
            }
            pclose(fp);
            
            LOG("[IP] Modified successfully but unable to verify\n");
            return 0;
        } else {
            LOG("[IP] IP modification failed\n");
            return -1;
        }
    }
    
    return 0; // IP正确，无需修改
}