#include "modbustcp_pro.h"
#include "function_task/modbustcp_task/modbustcp_task.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/sd_store/sd_store.h"
#include "interface/log/log.h"
#include "device_drv/ota_upgrade/ota_fun.h"
#include "modbus_defines.h"
#include <time.h>
#include <stdint.h>
extern unsigned short g_ota_flag;
pthread_mutex_t modbus_reg_mutex = PTHREAD_MUTEX_INITIALIZER;//所有写modbusBuff寄存器的时候都会调用加锁
atomic_int rtc_sync_pending = 0;
//测试使用
static uint64_t get_time_us(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        return 0;
    }

    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

// modbus接收数据处理，只处理06的写入操作
 void modbus_write_reg_deal(modbus_t *ctx, const uint8_t *query, int req_length)
{
    int header_length = 0;
    unsigned short data = 0;
	unsigned char sencount = 0;
    unsigned short address = 0;

  	static unsigned char last_data_power = 0xFF;
	static unsigned char last_data_ecomode = 0xFF;
	if(ctx == NULL){
		return;
	}
    header_length = modbus_get_header_length(ctx); // 获取数据长度
	if (req_length < header_length + 5) return;
    if (query[header_length] == 0x06) // 功能码
    {
        // 获取目标地址和数据
        address = (query[header_length + 1] << 8) | query[header_length + 2];
        data = (query[header_length + 3] << 8) | query[header_length + 4];

        // 判断地址范围
        if ((address >= REGISTERS_START_ADDRESS) && (address < (REGISTERS_START_ADDRESS + REGISTERS_NB)))
        {
			// printf("get_ota_UpDating = %d\r\n",get_ota_UpDating);
            // 开关机操作
            if ((address == MDBUS_BATTERY_CTL) && (get_ota_UpDating() == 0)) // 过滤，自己需要判断是否在升级来进行自主上下电
            {
                if (data == 0)
                {
					// printf("111get_ota_UpDating = %d\r\n",get_ota_UpDating);
					set_TCU_PowerUpCmd(BMS_POWER_ON);

                }
                else if (data == 1)
                {
					// printf("222get_ota_UpDating = %d\r\n",get_ota_UpDating);
                    set_TCU_PowerUpCmd(BMS_POWER_OFF);
                }

				if(data != last_data_power){
					LOG("[ModbusTcp] last_data_power = %d\r\n",data);
					last_data_power = data;
				}
				
            }
            // RTC时间设置
            else if (address >= MDBUS_RTC_YEAR && address <= MDBUS_RTC_SECOND)
            {
                rtc_Modbus_Deal(address, data);

            }
            // 设置ip
            else if (address == MDBUS_IPSET_HIGH || address == MDBUS_IPSET_LOW)
            {
                save_ip_to_conffile(address, data);
				LOG("[ModbusTcp] Set IP %d\r\n",data);
            }
            // 重启
            else if ((address == MDBUS_ECU_REBOOT) && (data == 1))
            {
				LOG("[ModbusTcp] Set Reboot %d\r\n",data);
                set_ems_bms_reboot();
            }
            else if ((address == MDBUS_ENESAV_CTL))//节能模式使能控制
            {
                if (data == 0)
                {
                    set_modbus_reg_val(MDBUS_ENESAV_STA, 0);
                    set_TCU_ECOMode(0);
                }
                else if (data == 1)
                {
                    set_modbus_reg_val(MDBUS_ENESAV_STA, 1);
                    set_TCU_ECOMode(1);
                }

				if(data != last_data_ecomode){
					LOG("[ModbusTcp] last_data_ecomode = %d\r\n",data);
					last_data_ecomode = data;
				}
            }
            else if ((address == MDBUS_OFFGRID_STA) || (address == MDBUS_VOLCAL_MODE) || (address == MDBUS_VOLCAL_VALUE))//离网、屏蔽、电压校准
            {
                VoltageCalibration_ModBus_Deal(address, data);
            }
            else if ((address == MDBUS_SET_SOH) || (address == MDBUS_SET_SOC) ||(address == MDBUS_REALY_CTL) || \
					(address == MDBUS_ENERGYACCUM_POSVALUE_H) ||(address == MDBUS_ENERGYACCUM_POSVALUE_L) || \
					(address == MDBUS_ENERGYACCUM_NEGVALUE_H) ||(address == MDBUS_ENERGYACCUM_NEGVALUE_L))//SOHCmd,SOCMinCmd,SOCMaxCmd,RelayCtl
            {
				LOG("[ModbusTcp] address: 0x%x,data: 0x%x\r\n",address,data);
				for(sencount = 0;sencount < 3;sencount++){
					BatteryCalibration_ModBus_Deal(address, data);
					usleep(2*1000);
				}
            }
            else if (address == MDBUS_SD_FROMAT)//SD卡格式化
            {
                LOG("SDCard Format.........\r\n");
                set_modbus_reg_val(address, data);
            }
        }
    }
}

/********************************************************************************
 * 函数名称： get_modbus_reg_val
 * 功能描述：   获取modbus寄存器的值
 * 输入参数：
 * 输出参数： 0表示获取成功，非0表示获取失败
 ********************************************************************************/
int get_modbus_reg_val(uint16_t addr, uint16_t *get_val)
{
	if (modbusBuff == NULL || get_val == NULL)
	{
		return -1;
	}
	if ((addr >= REGISTERS_START_ADDRESS) && (addr < (REGISTERS_START_ADDRESS + REGISTERS_NB)))
	{
		pthread_mutex_lock(&modbus_reg_mutex);
		*get_val = modbusBuff[addr - REGISTERS_START_ADDRESS];
		pthread_mutex_unlock(&modbus_reg_mutex);
		return 0;
	}
	else
	{
		*get_val = 0;
		return -2;
	}
	return 0;
}
/********************************************************************************
 * 函数名称： set_modbus_reg_val
 * 功能描述：   向modbus寄存器的值
 * 输入参数：
 * 输出参数： 0 表示写入成功，非0表示写入失败
 ********************************************************************************/
int set_modbus_reg_val(uint16_t addr, uint16_t set_val)
{
	if (modbusBuff == NULL)
	{
		return -1;
	}
	if ((addr >= REGISTERS_START_ADDRESS) && (addr < (REGISTERS_START_ADDRESS + REGISTERS_NB)))
	{
		pthread_mutex_lock(&modbus_reg_mutex);
		modbusBuff[addr - REGISTERS_START_ADDRESS] = set_val;
		pthread_mutex_unlock(&modbus_reg_mutex);
		return 0;
	}
	else
	{
		return -2;
	}
	return 0;
}

static int update_system_time(const Rtc_Ip_TimedateType *timeData)
{
	if (timeData == NULL)
	{
		fprintf(stderr, "Invalid time data pointer\n");
		return -1;
	}

	// 构造 struct tm
	struct tm external_time = {0};
	external_time.tm_year = timeData->year - 1900; // tm_year 是从1900年起计算的
	external_time.tm_mon = timeData->month - 1;	   // tm_mon 月份从0开始
	external_time.tm_mday = timeData->day;
	external_time.tm_hour = timeData->hour;
	external_time.tm_min = timeData->minutes;
	external_time.tm_sec = timeData->seconds;
	external_time.tm_isdst = -1; // 自动判断夏令时

	// 转换为 time_t
	time_t calibrated_time = mktime(&external_time);

	if (calibrated_time == -1){
		perror("mktime failed");
		return -1;
	}

    time_t local_time = time(NULL);
    if (local_time == (time_t)-1) {
        perror("time failed");
        return -1;
    }

	long diff = labs((long)(calibrated_time - local_time));

    if (diff > 2)
    {
        struct timespec ts;
        ts.tv_sec = calibrated_time;
        ts.tv_nsec = 0;

        if (clock_settime(CLOCK_REALTIME, &ts) == -1){
            perror("clock_settime failed");
            return -1;
        }
        atomic_store(&rtc_sync_pending, 1);
        LOG("System time updated, diff=%ld s, RTC sync pending\r\n", diff);
    }
    else
    {
        LOG("System time not updated, diff=%ld s\r\n", diff);
    }
	// 设置系统时间
	struct timespec ts;
	ts.tv_sec = calibrated_time;
	ts.tv_nsec = 0;

	if (clock_settime(CLOCK_REALTIME, &ts) == -1)
	{
		perror("clock_settime failed (need root?)");
		return -1;
	}
	return 0;
}

/********************************************************************************
 * 函数名称： rtc_Modbus_Deal
 * 功能描述： ModBus设置RTC指令
 * 输入参数：
 * 输出参数： 0 表示写入成功，1表示写入完成，-1表示失败。
 *sqw
 ********************************************************************************/
static int rtc_Modbus_Deal(uint16_t address, uint16_t data)
{
	static Rtc_Ip_TimedateType TmData = {0};

	if (address == MDBUS_RTC_YEAR) // 年
	{
		TmData.year = data;
		return 0; // 成功
	}
	else if (address == MDBUS_RTC_MONTH) // 月
	{
		TmData.month = data;
		return 0; // 成功
	}
	else if (address == MDBUS_RTC_DAY) // 日
	{
		TmData.day = data;
		return 0; // 成功
	}
	else if (address == MDBUS_RTC_HOUR) // 时
	{
		TmData.hour = data;
		return 0; // 成功
	}
	else if (address == MDBUS_RTC_MINUTE) // 分
	{
		TmData.minutes = data;
		return 0; // 成功
	}
	else if (address == MDBUS_RTC_SECOND) // 秒
	{
		static uint8_t rtccount = 0;
		TmData.seconds = (uint8_t)data;
		// uint64_t t_start = get_time_us();
		LOG("RTC Set Success!  \r\n");
		set_TCU_TimeYear((TmData.year % 100));
		set_TCU_TimeMonth(TmData.month);
		set_TCU_TimeDay(TmData.day);
		set_TCU_TimeHour(TmData.hour);
		set_TCU_TimeMinute(TmData.minutes);
		set_TCU_TimeSecond(TmData.seconds);
		set_TCU_TimeCalFlg(1);

		int ret = update_system_time(&TmData);
		LOG("[ModbusTcp] rtc_Modbus_Deal\r\n");
		for (int i = 0; i < 3; i++)
		{
			CANFDSendFcn_BCU_step();
			usleep(1 * 1000);
		}
		set_TCU_TimeCalFlg(0); // RTC设置完毕标志位为0

		// uint64_t t_end = get_time_us();
		// LOG("update_system_time ret=%d, cost=%.3f ms\r\n",ret,(double)(t_end - t_start) / 1000.0);
		return 1; // 完成
	}
	else
	{
		LOG("RTC Set Error!  \r\n");
		return -1; // 失败
	}
}

/********************************************************************************
 * 函数名称： BatteryCalibration_ModBus_Deal
 * 功能描述： ModBus设置电池标定指令
 * 输入参数：
 * 输出参数： 0 表示写入成功，1表示写入完成，-1表示失败。
 *sqw
 ********************************************************************************/
static int BatteryCalibration_ModBus_Deal(uint16_t address, uint16_t data)
{
	uint8_t SOHCmd, SOCMaxCmd, SOCMinCmd ,relayCtl = 0;
	CAN_FD_MESSAGE tx_msg = {0};
	static int pos_high_received = 0;
	static int pos_low_received = 0;
	static int neg_high_received = 0;
	static int neg_low_received = 0;
	static uint8_t pos_bytes[4] = {0};
	static uint8_t neg_bytes[4] = {0};
	int needsend = 0;
	int send_type = 0;

	tx_msg.Extended = 1;
	tx_msg.Length = 64U;
	tx_msg.ID = 0x1824E410;
	tx_msg.Remote = 0;
	tx_msg.BRS = 1;
	tx_msg.ProtocolMode = 1;
	tx_msg.DLC = 15U;

	if (address == MDBUS_SET_SOH)
	{
		SOHCmd = (data >> 8);

		tx_msg.Data[9] = SOHCmd;
		needsend = 1;
	}
	else if (address == MDBUS_SET_SOC)
	{
		SOCMaxCmd = (data >> 8);
		SOCMinCmd = (data & 0xff);

		tx_msg.Data[6] = SOCMaxCmd;
		tx_msg.Data[7] = SOCMinCmd;
		needsend = 1;
	}
	else if (address == MDBUS_REALY_CTL)
	{
		relayCtl = data;
		// 清除 Data[0] 的 bit2~5（共4位），保留其他位
	    tx_msg.Data[0] &= ~0x3C;  // 0x3C = 0b00111100，取反后为 ...11000011，即只更改Pos和Neg
	    uint8_t shifted = (relayCtl & 0x0F) << 2;  // 只取低4位，然后左移2，将 relayCtl 的 bit0~3 左移 2 位，对齐到目标位置（bit2~5）
		tx_msg.Data[0] |= shifted;// 写入到 Data[0]
		needsend = 1;
	}
	else if (address == MDBUS_ENERGYACCUM_POSVALUE_H)
	{
		pos_bytes[0] = (uint8_t)((data & 0xFF00)>> 8);
		pos_bytes[1] = (uint8_t)(data & 0xFF);
		pos_high_received = 1;
		if (pos_low_received) {
			needsend = 1;
			send_type = 1;
			pos_high_received = 0;
			pos_low_received = 0;
		}
	}
	else if (address == MDBUS_ENERGYACCUM_POSVALUE_L)
	{
		pos_bytes[2] = (uint8_t)((data & 0xFF00)>> 8);
		pos_bytes[3] = (uint8_t)(data & 0xFF);
		pos_low_received = 1;
		if (pos_high_received) {
			needsend = 1;
			send_type = 1;
			pos_high_received = 0;
			pos_low_received = 0;
		}
	}	
	else if (address == MDBUS_ENERGYACCUM_NEGVALUE_H)
	{
		neg_bytes[0] = (uint8_t)((data & 0xFF00)>> 8);
		neg_bytes[1] = (uint8_t)(data & 0xFF);
		neg_high_received = 1;
		if (neg_low_received) {
			needsend = 1;
			send_type = 2;
			neg_high_received = 0;
			neg_low_received = 0;
		}
	}
	else if (address == MDBUS_ENERGYACCUM_NEGVALUE_L)
	{
		neg_bytes[2] = (uint8_t)((data & 0xFF00)>> 8);
		neg_bytes[3] = (uint8_t)(data & 0xFF);
		neg_low_received = 1;
		if (neg_high_received) {
			needsend = 1;
			send_type = 2;
			neg_high_received = 0;
			neg_low_received = 0;
		}
	}
	if(needsend == 1){
		if (send_type == 1) {
			memcpy(&tx_msg.Data[14], pos_bytes, sizeof(pos_bytes));
			memset(pos_bytes, 0, sizeof(pos_bytes));
		} else if (send_type == 2) {
			memcpy(&tx_msg.Data[18], neg_bytes, sizeof(neg_bytes));
			memset(neg_bytes, 0, sizeof(neg_bytes));
		}

		char data_str[256] = {0}; // 64 字节 → 最多 "XX " * 64 + '\0' ≈ 192 字节
		int offset = 0;
		for (int i = 0; i < 64; i++) {
			offset += snprintf(data_str + offset, sizeof(data_str) - offset,
							"%02X%s", tx_msg.Data[i], (i < 64 - 1) ? " " : "");
		}
	
		LOG("[RECORD] TesterRly_Data, ID = 0x%x ,Data = %s\r", tx_msg.ID, data_str);
	
		Drv_bcu_canfd_send(&tx_msg);
	}

	return 0;
}

static int VoltageCalibration_ModBus_Deal(uint16_t address, uint16_t data)
{
	static uint8_t HighVoltType, Offgridstate = 0;
	static unsigned char last_data_offgrid = 0xFF;
	static uint16_t HighVoltValue = 0;
	if (address == MDBUS_OFFGRID_STA)//离网屏蔽
	{
		Offgridstate = data;
		set_TCU_FcnStopSet(Offgridstate);//bit0：屏蔽故障，支持开关离网,bit1：屏蔽绝缘故障，但是计算绝缘值,bit2：屏蔽绝缘功能，不计算绝缘值

		if(data != last_data_offgrid){
			LOG("[ModbusTcp] last_data_offgrid = %d\r\n",data);
			last_data_offgrid = data;
		}
	}
	else if (address == MDBUS_VOLCAL_MODE) //电压校准模式
	{
		HighVoltType = data;
		set_TCU_HighVoltType(HighVoltType);//电压校准模式
		LOG("[ModbusTcp] HighVoltType %d\r\n",data);
	}
	else if (address == MDBUS_VOLCAL_VALUE)//电压校准数值
	{
		HighVoltValue = data;
		set_TCU_HighVoltValue(HighVoltValue);//电压校准数值
		LOG("[ModbusTcp] HighVoltValue %d\r\n",data);
	}	
	return 0;
}

static void set_ems_bms_reboot()
{
	for(int i = 0; i < 3; i++){
		set_OTA_XCPConnect(170);
	}
	CANFDSendFcn_BCU_step();
	usleep(250 * 1000);
	LOG("\r\n\r\n  ******* ECU cmd Reset  *******  r\n\r\n");
	sleep(2);
	system("reboot"); // 复位并准备跳转
}
/*继电器控制，测试使用*/
