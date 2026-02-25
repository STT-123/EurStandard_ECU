#include "modbustcp_pro.h"
#include "function_task/modbustcp_task/modbustcp_task.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/sd_store/sd_store.h"
#include "interface/log/log.h"
#include "device_drv/ota_upgrade/ota_fun.h"
#include "modbus_defines.h"
extern unsigned short g_ota_flag;
pthread_mutex_t modbus_reg_mutex = PTHREAD_MUTEX_INITIALIZER;//所有写modbusBuff寄存器的时候都会调用加锁
// modbus接收数据处理，只处理06的写入操作
 void modbus_write_reg_deal(modbus_t *ctx, const uint8_t *query, int req_length)
{
    int header_length = 0;
    unsigned short data = 0;
	unsigned char sencount = 0;
    unsigned short address = 0;

  	static unsigned char last_data_power = 0xFF;
	static unsigned char last_data_ecomode = 0xFF;

    header_length = modbus_get_header_length(ctx); // 获取数据长度
    if ((req_length < 12) || (header_length >= MODBUS_TCP_MAX_ADU_LENGTH)){return; }// 长度不够直接退出
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
            else if ((address == MDBUS_SET_SOH) || (address == MDBUS_SET_SOC) ||(address == MDBUS_REALY_CTL))//SOHCmd,SOCMinCmd,SOCMaxCmd,RelayCtl
            {
				LOG("[ModbusTcp] address: 0x%x,data: 0x%x\r\n",address,data);
				for(sencount = 0;sencount < 5;sencount++){
					BatteryCalibration_ModBus_Deal(address, data);
					usleep(5*1000);
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

	if (calibrated_time == -1)
	{
		perror("mktime failed");
		return -1;
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

	// 将系统时间写入 RTC
	system("hwclock --systohc");

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

		LOG("RTC Set Success!  \r\n");
		set_TCU_TimeYear((TmData.year % 100));
		set_TCU_TimeMonth(TmData.month);
		set_TCU_TimeDay(TmData.day);
		set_TCU_TimeHour(TmData.hour);
		set_TCU_TimeMinute(TmData.minutes);
		set_TCU_TimeSecond(TmData.seconds);
		set_TCU_TimeCalFlg(1);

		update_system_time(&TmData);
		LOG("[ModbusTcp] rtc_Modbus_Deal\r\n");
		for (int i = 0; i < 3; i++)
		{
			CANFDSendFcn_BCU_step();
			usleep(1 * 1000);
		}
		set_TCU_TimeCalFlg(0); // RTC设置完毕标志位为0
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
	static uint8_t SOHCmd, SOCMaxCmd, SOCMinCmd,relayCtl = 0;
    static CAN_FD_MESSAGE bms_calibration_msg = {0}; // <-- 关键：static + 初始化一次,参考DBC文件，要是DBC 文件改了，这个也要动

    // 第一次调用时初始化结构体头（只做一次）
    if (bms_calibration_msg.ID == 0) {
        bms_calibration_msg.Extended = 1;
        bms_calibration_msg.Length = 64U;
        bms_calibration_msg.ID = 0x1824E410;
        bms_calibration_msg.Remote = 0;
        bms_calibration_msg.BRS = 1;
        bms_calibration_msg.ProtocolMode = 1;
        bms_calibration_msg.DLC = 15U;
        // Data 数组默认为0，后续逐步填充
    }

	if (address == MDBUS_SET_SOH)
	{
		SOHCmd = (data >> 8);

		bms_calibration_msg.Data[9] = SOHCmd;
	}
	else if (address == MDBUS_SET_SOC)
	{
		SOCMaxCmd = (data >> 8);
		SOCMinCmd = (data & 0xff);

		bms_calibration_msg.Data[6] = SOCMaxCmd;
		bms_calibration_msg.Data[7] = SOCMinCmd;
	}
	else if (address == MDBUS_REALY_CTL)
	{
		relayCtl = data;
		// 清除 Data[0] 的 bit2~5（共4位），保留其他位
    	bms_calibration_msg.Data[0] &= ~0x3C;  // 0x3C = 0b00111100，取反后为 ...11000011，即只更改Pos和Neg
    	uint8_t shifted = (relayCtl & 0x0F) << 2;  // 只取低4位，然后左移2，将 relayCtl 的 bit0~3 左移 2 位，对齐到目标位置（bit2~5）
		bms_calibration_msg.Data[0] |= shifted;// 写入到 Data[0]
	}
	Drv_bcu_canfd_send(&bms_calibration_msg);

	char data_str[256] = {0}; // 64 字节 → 最多 "XX " * 64 + '\0' ≈ 192 字节
    int offset = 0;
    for (int i = 0; i < 64; i++) {
        offset += snprintf(data_str + offset, sizeof(data_str) - offset,
                          "%02X%s", bms_calibration_msg.Data[i], (i < 64 - 1) ? " " : "");
    }

    LOG("[RECORD] TesterRly_Data, ID = 0x%x ,Data = %s\r", bms_calibration_msg.ID, data_str);
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
