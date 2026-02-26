#ifndef __C_MODBUS_UPDATE_H__
#define __C_MODBUS_UPDATE_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include "interface/modbus/modbus.h"
#include "interface/modbus/modbus-tcp.h"

#define LOGO 0x9528  //公司标识
#define ECU_VERSION 0x11E //  表示v1.01 格式xx。yy xx 高八位yy低八位
#define REGISTERS_NB 0x4000 // 寄存器数量


typedef struct
{
	uint16_t year;	  /**< @brief Year       */
	uint16_t month;	  /**< @brief Month      */
	uint16_t day;	  /**< @brief Day        */
	uint16_t hour;	  /**< @brief Hour       */
	uint16_t minutes; /**< @brief Minutes    */
	uint8_t seconds;  /**< @brief Seconds    */
} Rtc_Ip_TimedateType;

static void set_ems_bms_reboot();
static int VoltageCalibration_ModBus_Deal(uint16_t address, uint16_t data);
static int update_system_time(const Rtc_Ip_TimedateType *timeData);
static int rtc_Modbus_Deal(uint16_t address, uint16_t data);
static int BatteryCalibration_ModBus_Deal(uint16_t address, uint16_t data);

int set_modbus_reg_val(uint16_t addr, uint16_t set_val);
int get_modbus_reg_val(uint16_t addr, uint16_t *get_val);
void modbus_write_reg_deal(modbus_t *ctx, const uint8_t *query, int req_length);

#endif
