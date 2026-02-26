#ifndef __DRV_EXTERNAL_EMERGENCY_STOP_H__
#define __DRV_EXTERNAL_EMERGENCY_STOP_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>
#include "interface/bms/bms_simulink/rtwtypes.h"
#include "interface/setting/ip_setting.h"

#define Emergencystop_output 32
#define SET_ERROR 0
#define SET_RECOVER 1

//参考APP_DCB储能相关协议
#define PCS_STOP 0x03					// 外部急停，充电桩急停输入		emcu0 的第3bit，BCU提供
#define DOOR_OPEN 0x04					// 门禁故障检测。+			  emcu0 的第4bit，BCU提供
#define EMERGENCY_STOP 0x05 			// 急停故障，				 emcu0 的第5bit，BCU提供
#define BMS_COM_FAULT 0x06				// 和bms通讯故障 			 ecmu0 的第六bit，BCU提供

#define SD_FAULT 0x31					// sd卡故障   				emcu3 的第1字节，自己检测
#define INSIDE_NTC_FAULT  0x32        //内部温度故障  			     emcu3 的第2字节，BCU提供
#define OUTSIDE_COM_FAULT 0x33      	//外部温度传感器故障  		  emcu3 的第3字节，BCU提供
#define PHY_LINK_FAULT 0x34 			// PHY连接故障   			emcu3 的第8字节，自己检测
#define ISO_SWITCH_FAULT  0x35      	//隔开开关故障  		     emcu3 的第5字节，BCU提供

#define  NET_ETH_1	"eth1"
#define  NET_ETH_2 	"eth2"	
extern time_t g_last_bcu_rx_time;
/*-------------------*/
typedef struct {
    uint32_t bit_position;
    int fault_type;
} fault_mapping_t;

typedef struct
{
	unsigned long long rx_bytes;
	unsigned long long tx_bytes;
	unsigned long long rx_packets;
	unsigned long long tx_packets;
} net_stats_t;

typedef struct
{
	unsigned char emcu_fault_state; // 总故障标志
	unsigned short emcu_fault0;
	unsigned short emcu_fault1;
	unsigned short emcu_fault2;
	unsigned short emcu_fault3; // 告警不影响功能，比如sd卡
} ecu_fault_t;
// uint16_t *modbusBuff = NULL;
extern ecu_fault_t ecu_fault;
extern ecu_fault_t ecu_fault_last;
/*---------------------*/



void PHYlinktate();
void ECUfault_process();
void check_bcu_rx_timeout(void);
void set_emcu_fault(unsigned char parameter, unsigned char status);
void can_monitor_fun(void) ;
int check_can_state_detailed(const char* can_if);
void restart_can_interface_enhanced(const char* can_if);
void get_BCU_FaultInfo(uint32_T faultValue_4H,uint32_T faultValue_3H,uint32_T faultValue_2H);
int is_bcu_can_ready(void);
int is_bmu_can_ready(void);
int check_and_fix_ip(const char *if_name);
#endif