#include "ip_setting.h"
#include "interface/log/log.h"
#include "interface/bms/bms_simulink/CANFDRcvFcn_BCU.h"
#include "interface/bms/bms_simulink/CANRcvFcn_BMU.h"
#include "modbus_defines.h"
Setting_t g_ipsetting = {0};

void static save_setting_to_file(const char *filepath, Setting_t *ipsetting)
{
    FILE *fp = fopen(filepath, "wb");
    if (fp)
    {
        fwrite(ipsetting, sizeof(Setting_t), 1, fp);
        fclose(fp);
    }
    else
    {
        LOG("[Setting] Failed to write ipsetting\r\n");
    }
}

int load_setting_from_file(const char *filepath, Setting_t *ipsetting)
{
    FILE *fp = fopen(filepath, "rb");
    if (fp)
    {
        fread(ipsetting, sizeof(Setting_t), 1, fp);
        fclose(fp);
        return 0;
    }
    return -1;
}

// 判断是否为合法的可配置 IPv4 地址（排除 0.0.0.0, 255.255.255.255, 回环, 广播等）
static int is_valid_ip(uint32_t ip)
{
    // 所以这里 ip 也是主机字节序
    if (ip == 0) return 0;                      // 0.0.0.0
    if (ip == 0xFFFFFFFFU) return 0;            // 255.255.255.255

    uint8_t a = (ip >> 24) & 0xFF;
    uint8_t b = (ip >> 16) & 0xFF;

    if (a == 127) return 0;// 回环地址 127.0.0.0/8
    if (a == 169 && b == 254) return 0;// 链路本地地址 169.254.0.0/16（通常不应作为配置 IP）
    if (a >= 224 && a <= 239) return 0; // 多播地址 224.0.0.0/4
    if (a >= 240) return 0;// 保留地址（240.0.0.0/4，包括 255.255.255.255 已处理）
    return 1; // 合法
}


void settings_Init()
{
    memset(&g_ipsetting, 0, sizeof(g_ipsetting));

    struct stat st;
    int config_exist = (stat(CONFIG_FILE_PATH, &st) == 0);

    if (config_exist && (load_setting_from_file(CONFIG_FILE_PATH, &g_ipsetting) == 0) && (g_ipsetting.flag == 1) && (is_valid_ip(g_ipsetting.ip)))
    {
        struct in_addr addr;
        addr.s_addr = htonl(g_ipsetting.ip);
        LOG("[Setting] Read IP from config: %s\n", inet_ntoa(addr));
    }
    else
    {
        // 配置文件不存在或数据无效，写入默认值
        g_ipsetting.flag = 1;
        g_ipsetting.ip = (192 << 24) | (168 << 16) | (1 << 8) | 110;

        save_setting_to_file(CONFIG_FILE_PATH, &g_ipsetting);

        struct in_addr addr;
        addr.s_addr = htonl(g_ipsetting.ip);
        LOG("[Setting] Default IP used and saved: %s\n", inet_ntoa(addr));
    }
}

void save_ip_to_conffile(uint16_t address, uint16_t data)
{
    static uint32_t ip_addr = 0;
    if (address == MDBUS_IPSET_HIGH) // 高 16 位
    {
        ip_addr &= 0x0000ffff;
        ip_addr |= ((uint32_t)data) << 16;
        return;
    }
    else if (address == MDBUS_IPSET_LOW) // 低 16 位
    {
        ip_addr &= 0xffff0000;
        ip_addr |= data & 0xFFFF;

        g_ipsetting.ip = ip_addr;
        g_ipsetting.flag = 1;

        // 显示 IP
        struct in_addr ip_struct;
        ip_struct.s_addr = htonl(ip_addr); // 转为网络字节序
        LOG("[IP] Set IP to: %s\n", inet_ntoa(ip_struct));
        save_setting_to_file(CONFIG_FILE_PATH, &g_ipsetting);
        usleep(500 * 1000);
        system("reboot");
    }
}

void set_system_time_from_bcu(void)
{
    struct tm tm;
    struct timeval tv;

    memset(&tm, 0, sizeof(struct tm));

    tm.tm_year = 2000 + get_BCU_TimeYearValue() - 1900; // tm_year 是从1900年开始计数的
    tm.tm_mon = get_BCU_TimeMonthValue() - 1;           // tm_mon 范围是 0-11
    tm.tm_mday = get_BCU_TimeDayValue();
    tm.tm_hour = get_BCU_TimeHourValue();
    tm.tm_min = get_BCU_TimeMinuteValue();
    tm.tm_sec = get_BCU_TimeSencondValue();

    time_t t = mktime(&tm); // 转为 time_t 类型（时间戳）
    if (t == -1)
    {
        LOG("[IP] Invalid time provided\n");
        return;
    }

    tv.tv_sec = t;
    tv.tv_usec = 0;

    if (settimeofday(&tv, NULL) != 0)
    {
        perror("settimeofday failed");
    }
    else
    {
        // 设置成功
    }
}


int set_ip_address(const char *if_name, const char *ip_addr)
{
    int fd;
    struct ifreq ifr;
    struct sockaddr_in sin;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        LOG("[set_ip_address] socket创建失败: %s\n", strerror(errno));
        return -1;
    }
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1);

    memset(&sin, 0, sizeof(struct sockaddr_in));
    sin.sin_family = AF_INET;
    
    if (inet_pton(AF_INET, ip_addr, &sin.sin_addr) <= 0) {
        LOG("[set_ip_address] IP地址转换失败: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    
    memcpy(&ifr.ifr_addr, &sin, sizeof(struct sockaddr));

    // 设置IP地址
    if (ioctl(fd, SIOCSIFADDR, &ifr) < 0)
    {
        LOG("[set_ip_address] SIOCSIFADDR失败: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // 获取当前网卡标志
    if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0)
    {
        LOG("[set_ip_address] SIOCGIFFLAGS失败: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // 设置网口状态为 up
    ifr.ifr_flags |= IFF_UP | IFF_RUNNING;

    if (ioctl(fd, SIOCSIFFLAGS, &ifr) < 0)
    {
        LOG("[set_ip_address] SIOCSIFFLAGS失败: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}
