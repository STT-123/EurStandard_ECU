#ifndef _G_AddressDefinition_H_
#define _G_AddressDefinition_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>


#define OTASTATUSREGADDR        0x4010
#define OTAPPROGRESSREGADDR     0x4011
#define OTACTRLREGADDR          0x4012

#define AC_SBL_OTAFILENUMBER    0x4013
#define AC_APP_OTAFILENUMBER    0x4014

#define MDBUS_ADDR_DC_VOL       0x3460   // 直流电压
#define MDBUS_ADDR_DC_CUR       0x3462   // 直流电流
#define MDBUS_ADDR_DC_POW       0x3464   // 直流功率
#define MDBUS_ADDR_P_ENERGY     0x3466 // 正向能量
#define MDBUS_ADDR_N_ENERGY     0x3468 // 负向能量

#define MDBUS_ENESAV_STA 	    0x3418  //系统节能模式状态

#define MDBUS_ADDR_BECU_FAULT0 0x3440  //ECMU故障0
#define MDBUS_ADDR_BECU_FAULT1 0x3441  //ECMU故障1
#define MDBUS_ADDR_BECU_FAULT2 0x3442  //ECMU故障2
#define MDBUS_ADDR_BECU_FAULT3 0x3443  //ECMU故障3
#define MDBUS_ADDR_BECU_FAULT4 0x3444  //ECMU故障4

#define MDBUS_BATTERY_CTL  	   0x6700  //电池组工作状态控制
#define MDBUS_RTC_YEAR  	   0x6705  //rtc校准年
#define MDBUS_RTC_MONTH  	   0x6706  //rtc校准月
#define MDBUS_RTC_DAY  	  	   0x6707  //rtc校准日
#define MDBUS_RTC_HOUR  	   0x6708  //rtc校准时
#define MDBUS_RTC_MINUTE  	   0x6709  //rtc校准分
#define MDBUS_RTC_SECOND  	   0x670A  //rtc校准秒

#define MDBUS_IPSET_HIGH  	   0x6711  //设置ip地址高位
#define MDBUS_IPSET_LOW  	   0x6712  //设置ip地址低位

#define MDBUS_SET_SOH  	   	   0x6714  //SOHCmd
#define MDBUS_SET_SOC  	   	   0x6715  //SOCCmd
#define MDBUS_ENESAV_CTL 	   0x6718	//节能模式使能控制
#define MDBUS_OFFGRID_STA 	   0x6719	//离网模式状态
#define MDBUS_ECU_REBOOT  	   0x6720	//ECU重启
#define MDBUS_SD_FROMAT  	   0x6721	//SD卡格式化

#define MDBUS_VOLCAL_MODE  	   0x6734	//电压校准模式
#define MDBUS_VOLCAL_VALUE     0x6735	//电压校准数值
#define MDBUS_REALY_CTL        0x6736	//继电器控制

#define MDBUS_ENERGYACCUM_POSVALUE_H         0x6723	//校准正电能值_H
#define MDBUS_ENERGYACCUM_POSVALUE_L         0x6724	//校准正电能值_L
#define MDBUS_ENERGYACCUM_NEGVALUE_H         0x6725	//校准负电能值_H
#define MDBUS_ENERGYACCUM_NEGVALUE_L         0x6726	//校准负电能值_L

#define MDBUS_ADDR_PRODUCTION   0x3416  // ems 厂家
#define MDBUS_ADDR_ECU_VERSION  0x3417 // ems 版本号

#define MDBUS_BCU_CAPACITY            0x5010  //BCU启动版本
#define MDBUS_BCU_COOLING             0x5011  //指定制冷方式

#define MDBUS_SD_CAPACITY       0x5021  //SD卡使用百分比，单位: %
#define MDBUS_ROOT_CAPACITY     0x5022  //系统盘使用百分比，单位: %

#define MDBUS_SN_START          0x5050  

#endif
