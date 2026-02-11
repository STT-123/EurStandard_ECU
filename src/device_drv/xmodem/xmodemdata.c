#define _GNU_SOURCE
// #include "C_OTADataMonitor.h"
#include "xmodemdata.h"
#include "xmodemstate.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/log/log.h"
#include "device_drv/ota_upgrade/ota_uds_update.h"
#include "device_drv/ota_upgrade/ota_xcp_update.h"
#include "device_drv/ota_upgrade/ota_other_update.h"
#include "interface/bms/bms_analysis.h"
#include "device_drv/ota_upgrade/ota_fun.h"
#include "device_drv/sd_store/sd_store.h"

unsigned char tcp_server_recvbuf[2048] = {0};
unsigned char tcp_server_Txbuf[256] = {0};
volatile unsigned long curmsgtimer =0 ;
const unsigned int crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


void *lwip_data_TASK(void *param)
{
	LOG("[Xmodem] lwip_data_TASK %d\r\n", otasock1);
	LOG("[Xmodem] otasock1 %d\r\n", otasock1);
	char otafilenamestr[128] = {'\0'};//文件名
	int filesize = 0;//总字节数
	int xmodempacknum = 0;//总包数
	int readdatanum = 128;//写入字节数
	int packno = 0;//当前接收包数，逐渐增加
	unsigned int errpacknum = 0;
	static unsigned char findfirstpack = 0;
	unsigned char prvpackno = 0;
	unsigned char curpackno = 0;
	unsigned int packidoverflownum = 0;
	char otafilenamestr1[130] = {'\0'};
	unsigned char otadeviceType = 0;
	static int filenormalflag =0;
	int errorCount = 0;
	while (1)
	{
		if(otasock1 > 0)
		{
			memset(tcp_server_recvbuf, 0, 2048);
			int length = read(otasock1, tcp_server_recvbuf, 2048);
			curmsgtimer = OsIf_GetMilliseconds();
			// LOG("[Xmodem] length :%d\r\n",length);
			//目前接收到的BCU\BMU\ECU都是133长度的数据
			if(length == 133)
			{
				if((tcp_server_recvbuf[0] == SOH)  && (crcGet(tcp_server_recvbuf, 131) == (tcp_server_recvbuf[131] << 8 | tcp_server_recvbuf[132])))
				{
					errpacknum = 0;
					tcp_server_Txbuf[0] = ACK;
					write(otasock1, tcp_server_Txbuf, 1);
					if(tcp_server_recvbuf[1] == 0x00) //文件起始帧filesize
					{
						LOG("Received first pack !\r\n");
						setXmodemServerReceiveSOH(1);
						if(GetOTAFILEInfo(&(tcp_server_recvbuf[3]),otafilenamestr, &filesize, &xmodempacknum) == 0)
						{
							LOG("[Xmodem] File name %s filesize %d packnum %d\r\n", otafilenamestr, filesize, xmodempacknum);
							findfirstpack = 1;
							curpackno = 0;
							prvpackno = 0;
							packidoverflownum = 0;  //lx
							snprintf(otafilenamestr1, sizeof(otafilenamestr1), "%s/%s", USB_MOUNT_POINT, otafilenamestr);
	
						}
					}
					else  //文件数据帧
					{
						if(findfirstpack)
						{
							curpackno = tcp_server_recvbuf[1];//系列号

							if(curpackno == 0x01 && prvpackno == 0xff)
							{
								packidoverflownum++;//循环次数
							}
							packno = tcp_server_recvbuf[1] + packidoverflownum * 255;//总包数
							// LOG("[Xmodem] recv packnum = %d,packidoverflownum = %d,tcp_server_recvbuf[1] = %d \r\n",packno,packidoverflownum,tcp_server_recvbuf[1]);
							if(packno != xmodempacknum)
							{
								readdatanum = 128;//每次读取128字节
								if(packno == 1)//第一包
								{
									LOG("[Xmodem] otafilenamestr : %s\r\n",otafilenamestr);
									if(strstr(otafilenamestr, "ECU") != NULL)									
									{
										otadeviceType = ECU;
										set_ota_UpDating(1);//1130
										LOG("[Xmodem] otadeviceType  %d\r\n", otadeviceType);
									}
									else if(strstr(otafilenamestr, "BCU") != NULL)
									{
										otadeviceType = BCU;
										set_ota_UpDating(1);//1130g_otactrl.UpDating
										LOG("[Xmodem] otadeviceType  %d\r\n", otadeviceType);
									}
									else if(strstr(otafilenamestr, "BMU") != NULL)
									{
										otadeviceType = BMU;
										set_ota_UpDating(1);//1130
										LOG("[Xmodem] otadeviceType  %d\r\n", otadeviceType);
									}
									else if( sblfilenumber == 1)//AC
									{
										LOG("[Xmodem] sblfilenumber = %d\r\n",sblfilenumber);
										set_ota_UpDating(1);//1130

									}
									else if(strstr(otafilenamestr, "ACP") != NULL)
									{
										LOG("[Xmodem] ACP_OTA_FILE_DATA..... \r\n");
										set_ota_UpDating(1);//1220
									}
									else if(strstr(otafilenamestr, "DCDC") != NULL)
									{
										LOG("[Xmodem] DCDC UpDating ..... \r\n");
										set_ota_UpDating(1);//1220
									}

									else
									{
										set_ota_UpDating(0);//1130
										otadeviceType = 0;
										delete_files_with_prefix("USB_MOUNT_POINT", "XC");
										LOG("[Xmodem] Invalid upgrade file\r\n");
										setXmodemServerReceiveFileEnd(1);
										set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);;

									}
								}

								int err = SaveOtaFile(otafilenamestr1, &(tcp_server_recvbuf[3]), xmodempacknum, packno, readdatanum);
								if(err != 0)
								{
									filenormalflag =1;
									set_ota_UpDating(0);//1130
									otadeviceType = 0;
									if (fclose(&OTAfil) != 0)
									{
										LOG("[Xmodem] Error file close failed err code!\r\n");
									}
									else
									{
										LOG("[Xmodem] file closed successfully!\r\n");
									}
									delete_files_with_prefix("0:", "XC");
									LOG("[Xmodem] Failed to write upgrade file\r\n");
									setXmodemServerReceiveFileEnd(1);
									set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
									set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
								}
							}
							else//最后1包
							{
								filesize%128?(readdatanum = filesize%128):(readdatanum = 128);//剩余字节数
								LOG("Receive the last pack , need read %d data from xmodem data area!\r\n", readdatanum);
								int err = SaveOtaFile(otafilenamestr1, &(tcp_server_recvbuf[3]), xmodempacknum, packno, readdatanum);
								if(err != 0)
								{
									filenormalflag =1;
									set_ota_UpDating(0);//1130
									otadeviceType = 0;
									if (fclose(&OTAfil) != 0)
									{
										LOG("[Xmodem] Error file close failed err code!\r\n");
									}
									else
									{
										LOG("[Xmodem] file closed successfully!\r\n");
									}
									delete_files_with_prefix("0:", "XC");
									LOG("[Xmodem] Failed to write upgrade file\r\n");
									// XmodemServerReceiveFileEnd = 1;
									setXmodemServerReceiveFileEnd(1);
									set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
									set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
								}
								setXmodemServerReceiveFileEnd(1);//考虑后移动
								LOG("[Xmodem] get_ota_UpDating(): %d\r\n",get_ota_UpDating());
								LOG("[Xmodem] otafilenamestr1111111 : %s\r\n",otafilenamestr1);
								if((strstr(otafilenamestr, "bin") != NULL) || (strstr(otafilenamestr1, "bz2") != NULL) || (strstr(otafilenamestr1, "deb") != NULL) || (strstr(otafilenamestr1, "tar") != NULL))
								{
									set_modbus_reg_val(OTASTATUSREGADDR, FILEDECRYPTIONNORMALTERMINATION);
									set_ota_OTAFileType(0);
									if(strstr(otafilenamestr, "ECU") != NULL)
									{
										set_ota_deviceType(otadeviceType);
										LOG("[Xmodem] ECU_OTA_otadeviceType: %u\r\n");
										LOG("[Xmodem] otafilenamestr: %u\r\n",otafilenamestr);
										set_ota_OTAFilename(otafilenamestr);
										set_ota_deviceID(0);
										set_ota_OTAStart(1);

									}
									else if(strstr(otafilenamestr, "BCU") != NULL)
									{
										set_ota_deviceType(otadeviceType);
										set_ota_deviceID(BCUOTACANID);
										set_ota_OTAFilename(otafilenamestr);
										set_ota_OTAStart(1);
									}
									else if(strstr(otafilenamestr, "BMU") != NULL)
									{
										set_ota_deviceType(otadeviceType);
										set_ota_OTAFilename(otafilenamestr);
										set_ota_deviceID(0x1821FF10);
										set_ota_OTAStart(1);
									}
									else if(strstr(otafilenamestr, "ACP") != NULL)
									{
										set_ota_deviceType(ACP);
										set_ota_OTAFilename(otafilenamestr);
										set_ota_deviceID(ACPOTACANID);
										set_ota_OTAStart(1);

									}
									else if(strstr(otafilenamestr, "DCDC") != NULL)
									{
										set_ota_deviceType(DCDC);
										set_ota_OTAFilename(otafilenamestr);
										set_ota_deviceID(DCDCOTACANID);
										set_ota_OTAStart(1);
									}
									//------------------------------OTAACP----------------------------------------//
									else if(strstr(otafilenamestr, "AC") != NULL) 
									{
									    char *token;
									    char *delimiter = "_";
										set_modbus_reg_val(OTASTATUSREGADDR, FILEDECRYPTIONNORMALTERMINATION);

										set_ota_OTAFileType(0);
										if(strstr(otafilenamestr, "AC_SBL") != NULL)//XC_AC_SBL_<地址>_<长度>_<CRC>.bin    // Bootloader
										{
											clock_gettime(CLOCK_MONOTONIC, &AC_OTA_lastCheckTick);
											memset(g_otactrl.OTAUdsSblFilename[SBl_index],0 ,sizeof(g_otactrl.OTAUdsSblFilename[SBl_index]));
											//需要告诉我一共分成多少bin文件，然后达到数量后赋值otactrl.deviceType = ACP;，进入XcpOTATestTask
											LOG("[Xmodem] AC_SBL_OTA_FILE_NAME: %s\r\n", otafilenamestr);
											LOG("[Xmodem] AC_SBL_OTA_FILE_NAME: %s\r\n", otafilenamestr1);
											memcpy(g_otactrl.OTAUdsSblFilename[SBl_index], otafilenamestr1, strlen(otafilenamestr1));

										    token = strtok(otafilenamestr, delimiter); // AC
										    token = strtok(NULL, delimiter);          // SBL
											token = strtok(NULL, delimiter);
											flashData.writeAddr =(uint32_t)strtoul(token, NULL, 16);
											token = strtok(NULL, delimiter);
											flashData.writeLen =(uint32_t)strtoul(token, NULL, 16);
											token = strtok(NULL, delimiter);
											flashData.CRC =(uint16_t)strtoul(token, NULL, 16);
											LOG("[Xmodem] g_otactrl.OTAUdsSblFilename[SBl_index]: %s!\r\n", g_otactrl.OTAUdsSblFilename[SBl_index]);
											LOG("[Xmodem] flashData.writeAddr: 0x%08X\r\n", flashData.writeAddr);
											LOG("[Xmodem] flashData.writeLen: 0x%08X\r\n", flashData.writeLen);
											LOG("[Xmodem] flashData.CRC: 0x%04X\r\n", flashData.CRC);
											LOG("[Xmodem] SBl_index: %d\r\n", SBl_index);
											SBl_index++;

										}
										else if(strstr(otafilenamestr, "AC_APP") != NULL)//XC_AC_APP_<地址>_<长度>_<CRC>.bin   // 应用程序
										{
												clock_gettime(CLOCK_MONOTONIC, &AC_OTA_lastCheckTick);
												memset(g_otactrl.OTAUdsFilename[APP_index],0 ,sizeof(g_otactrl.OTAUdsFilename[APP_index]));
												LOG("[Xmodem] AC_APP_OTA_FILE_NAME: %s\r\n", otafilenamestr);
												LOG("[Xmodem] AC_SBL_OTA_FILE_NAME: %s\r\n", otafilenamestr1);
												memcpy(g_otactrl.OTAUdsFilename[APP_index], otafilenamestr1, strlen(otafilenamestr1));

											    token = strtok(otafilenamestr, delimiter); // AC
											    token = strtok(NULL, delimiter);          // SBL
												token = strtok(NULL, delimiter);
												appData[APP_index].writeAddr =(uint32_t)strtoul(token, NULL, 16);
												token = strtok(NULL, delimiter);
												appData[APP_index].writeLen =(uint32_t)strtoul(token, NULL, 16);
												token = strtok(NULL, delimiter);
												appData[APP_index].CRC =(uint16_t)strtoul(token, NULL, 16);
												LOG("[Xmodem] g_otactrl.OTAUdsFilename[APP_index]: %s\r\n", g_otactrl.OTAUdsFilename[APP_index]);
												LOG("[Xmodem] appData.writeAddr[APP_index]: 0x%08X\r\n", appData[APP_index].writeAddr);
												LOG("[Xmodem] appData.writeLen[APP_index]: 0x%08X\r\n", appData[APP_index].writeLen);
												LOG("[Xmodem] appData.CRC[APP_index]: 0x%04X!\r\n", appData[APP_index].CRC);
												LOG("[Xmodem] APP_index: %d\r\n", APP_index);
												APP_index++;

										}

									}
										get_modbus_reg_val(AC_SBL_OTAFILENUMBER, &sblfilenumber);
										get_modbus_reg_val(AC_APP_OTAFILENUMBER, &appfilenumber);
										usleep(50*1000);
										LOG("[Xmodem] SBl_index ...  %d \r\n",SBl_index);
										LOG("[Xmodem] APP_index ...  %d \r\n",APP_index);
										LOG("[Xmodem] sblfilenumber...%d\r\n",sblfilenumber);
										LOG("[Xmodem] appfilenumber...%d\r\n",appfilenumber);


									if((SBl_index != 0) && (SBl_index == sblfilenumber) && (APP_index != 0) && (APP_index == appfilenumber))
									{
										LOG("[Xmodem] SBl_index ...  %d \r\n",SBl_index);
										LOG("[Xmodem] APP_index ...  %d \r\n",APP_index);
										LOG("[Xmodem] sblfilenumber...%d\r\n",sblfilenumber);
										LOG("[Xmodem] appfilenumber...%d\r\n",appfilenumber);
										set_ota_deviceID(ACOTACANID);
										set_ota_deviceType(AC);
										set_ota_OTAStart(1);

									}
									//------------------------------OTAACP----------------------------------------//

									else
									{

									}
									LOG("[Xmodem] OTAStart:%d,deviceID:%d,OTAFilename:%s,OTAFileType:%d,deviceType:%d\n", g_otactrl.OTAStart, g_otactrl.deviceID, g_otactrl.OTAFilename, g_otactrl.OTAFileType, g_otactrl.deviceType);

								}
								else if(strstr(otafilenamestr, "srec") != NULL || strstr(otafilenamestr, "s19") != NULL)
								{
									set_modbus_reg_val(OTASTATUSREGADDR, FILEISDECRYPTIIONING);
									g_otactrl.OTAFileType = 1;
									if(strstr(otafilenamestr, "ECU") != NULL)
									{
										g_otactrl.deviceType = ECU;
										memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
										memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));
									}
									else if(strstr(otafilenamestr, "BCU") != NULL)
									{
										g_otactrl.deviceType = BCU;
										g_otactrl.deviceID = BCUOTACANID;
										memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
										memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));
									}
									else if(strstr(otafilenamestr, "BMU") != NULL)
									{
										g_otactrl.deviceType = BMU;
										g_otactrl.deviceID = 0x1821FF10;
										memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
										memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));
									}
									else
									{

									}
								}
	

							}
							prvpackno = curpackno;

						}
					}

				}
				else
				{
					errpacknum++;
					tcp_server_Txbuf[0] = NAK;
					write(otasock1, tcp_server_Txbuf, 1);
				}
			}
			else if(length == 1029)
			{
				if((tcp_server_recvbuf[0] == STX) && (crcGet(tcp_server_recvbuf, 1027) == (tcp_server_recvbuf[1025] << 8 | tcp_server_recvbuf[1026])))
				{

					errpacknum = 0;
					tcp_server_Txbuf[0] = ACK;
					write(otasock1, tcp_server_Txbuf, 1);

					if(findfirstpack)
					{
						curpackno = tcp_server_recvbuf[1];

						if(curpackno == 0x01 && prvpackno == 0xff)
						{
							packidoverflownum++;
						}
						packno = tcp_server_recvbuf[1] + packidoverflownum * 255;
						if(packno != xmodempacknum - 1)
						{
							readdatanum = 1024;
							if(packno == 1)
							{
								if(strstr(otafilenamestr, "ECU") != NULL)									
								{
									otadeviceType = ECU;
									set_ota_UpDating(1);//1130
								}
								else if(tcp_server_recvbuf[51]==0x42 && tcp_server_recvbuf[52]==0x43 && tcp_server_recvbuf[53]==0x55)
								{
									otadeviceType = BCU;
								}
								else if(tcp_server_recvbuf[51]==0x42 && tcp_server_recvbuf[52]==0x4d && tcp_server_recvbuf[53]==0x55)
								{
									otadeviceType = BMU;
								}
								else
								{
									delete_files_with_prefix(USB_MOUNT_POINT, "XC");
									otadeviceType = 0;
									LOG("[Xmodem] Invalid upgrade file\r\n");
									setXmodemServerReceiveFileEnd(1);
								}
							}
							int err = SaveOtaFile(otafilenamestr, &(tcp_server_recvbuf[3]), xmodempacknum, packno, readdatanum);
							if(err != 0)
							{
								filenormalflag =1;
								set_ota_UpDating(0);//1130
								otadeviceType = 0;
								if (fclose(&OTAfil) != 0)
								{
									LOG("[Xmodem] Error file close failed err code!\r\n");
								}
								else
								{
									LOG("[Xmodem] file closed successfully!\r\n");
								}
								delete_files_with_prefix("0:", "XC");
								LOG("[Xmodem] Failed to write upgrade file\r\n");
								setXmodemServerReceiveFileEnd(1);
								set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
								set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
							}
						}
						else
						{

							filesize%1024?(readdatanum = filesize%1024):(readdatanum = 1024);
							LOG("[Xmodem] Receive the last pack , need read %d data from xmodem data area!\r\n", readdatanum);
							int err = SaveOtaFile(otafilenamestr, &(tcp_server_recvbuf[3]), xmodempacknum, packno, readdatanum);
							if(err != 0)
							{
								filenormalflag =1;
								set_ota_UpDating(0);//1130
								otadeviceType = 0;
								if (fclose(&OTAfil) != 0)
								{
									LOG("[Xmodem] Error file close failed err code!\r\n");
								}
								else
								{
									LOG("[Xmodem] file closed successfully!\r\n");
								}

								delete_files_with_prefix("0:", "XC");
								LOG("[Xmodem] Failed to write upgrade file\r\n");
								setXmodemServerReceiveFileEnd(1);
								set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
								set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
							}
							setXmodemServerReceiveFileEnd(1);

							if(strstr(otafilenamestr1, "bin") != NULL)
							{
								set_ota_OTAFileType(0);
								if(strstr(otafilenamestr1, "ECU") != NULL)
								{
									g_otactrl.deviceType = otadeviceType;
									memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
									memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));

								}
								else if(strstr(otafilenamestr1, "BCU") != NULL)
								{
									g_otactrl.deviceType = otadeviceType;
									memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
									memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));
								}
								else if(strstr(otafilenamestr1, "BMU") != NULL)
								{
									g_otactrl.deviceType = otadeviceType;
									memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
									memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));
								}
								else
								{

								}
							}
							else if(strstr(otafilenamestr1, "srec") != NULL)
							{
								g_otactrl.OTAFileType = 1;
								if(strstr(otafilenamestr1, "ECU") != NULL)
								{
									g_otactrl.deviceType = ECU;
									memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
									memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));

								}
								else if(strstr(otafilenamestr1, "BCU") != NULL)
								{
									g_otactrl.deviceType = BCU;
									memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
									memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));
								}
								else if(strstr(otafilenamestr1, "BMU") != NULL)
								{
									g_otactrl.deviceType = BMU;
									memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
									memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));
								}
								else
								{

								}
							}
							else if(strstr(otafilenamestr1, "zip") != NULL)
							{
								set_ota_OTAFileType(0);
								if(strstr(otafilenamestr1, "ECU") != NULL)
								{
									LOG("[Xmodem] ECU tar.bz2 file\r\n");
									g_otactrl.deviceType = otadeviceType;
									memset(g_otactrl.OTAFilename ,0 ,sizeof(g_otactrl.OTAFilename));
									memcpy(g_otactrl.OTAFilename, otafilenamestr, strlen(otafilenamestr));

								}
								else
								{

								}
							}
						}
					}
				}
				else
				{
					errpacknum++;
					tcp_server_Txbuf[0] = NAK;
					write(otasock1, tcp_server_Txbuf, 1);
				}

			}
			else if(length == 1)
			{
				LOG("[Xmodem] Rcv 1 byte data -> 0x%x\r\n", tcp_server_recvbuf[0]);
				if(tcp_server_recvbuf[0] == EOT)
				{
					setXmodemServerReceiveEOT(1);
				}
			}
			else if(length == -1 || length == 0)
			{
			    errorCount++;

			    if (errorCount >= 10)
			    {
					setXmodemServerReceiveFileEnd(1);
			        set_ota_UpDating(0);//1130
			    }
			}
			else
			{
			    errorCount = 0;
			}
		}

		if(errpacknum > 5)
		{
			LOG("[Xmodem] error pack over 5!\r\n");
			setXmodemServerEnd(1);
		}

		if(getXmodemServerReceiveFileEnd())
		{
			LOG("[Xmodem] receive file end!\r\n");
			uint32_t starttime = OsIf_GetMilliseconds();
			while(1)
			{
				uint32_t time = OsIf_GetMilliseconds()-starttime;

				{
					LOG("[Xmodem] wait XmodemServerReceiveEOT over !\r\n");
					setXmodemServerEnd(1);
					setXmodemServerReceiveEOT(0);
					setXmodemServerReceiveFileEnd(0);
					break;
				}
				if(time >= 10000)
				{
					tcp_server_Txbuf[0] = CAN;
					write(otasock1, tcp_server_Txbuf, 1);
					LOG("[Xmodem] wait XmodemServerReceiveEOT over 10s exit!\r\n");
					setXmodemServerEnd(1);
					setXmodemServerReceiveEOT(0);
					setXmodemServerReceiveFileEnd(0);
					break;

				}
				usleep(100*1000);
			}
		}
		usleep(5*1000);
	}
}


signed char GetOTAFILEInfo(unsigned char *databuf, char *name, int *filesize, int *xmodempacknum)

{
	char namebuf[128] = {'\0'};
	char filesizebuf[128] = {'\0'};
	char filepacknumbuf[128] = {'\0'};
	int idx = 0;
	int idx1 = 0;
	unsigned short i = 0;
	if(databuf == NULL)
	{
		LOG("[Xmodem] databuf null\r\n");
		return -1;
	}

	for( i = 0; i < 128; i++)
	{

		if(databuf[i] == 0x00)
		{
			idx++;
		}else{
		}
		if(idx == 1)//根据实际代码可以看出，起始和结束都有0x00标志开始，所以判断idx 1、3、5
		{
			idx1 = i;
			for(int j = 0; j < idx1 + 2; j++)//idx1 + 2长度
			{
				namebuf[j] = databuf[idx + j - 1];//位置
			}
			memcpy(name, namebuf, 128);
		}
		else if(idx == 3)
		{
			for(int j = 0; j < i - idx1 - 2; j++)//i - idx1 - 2长度
			{
				filesizebuf[j] = databuf[idx1 + j + 2];//位置
			}
			idx += 1;
			idx1 = i;
		}

		else if(idx == 5)
		{
			for(int j = 0; j < i - idx1 - 1; j++)//i - idx1 - 1 长度
			{
				filepacknumbuf[j] = databuf[idx1 + j + 1];//位置
			}
			idx1 = i;
			break;
		}

	}

	if (sscanf(filesizebuf, "%d", filesize) == 1)
	{
		//printf(" filesize %s->%d success \r\n", filesize);
	}
	else
	{
		LOG("[Xmodem] filesize %s->%d error \r\n");
		return -2;
	}
	if (sscanf(filepacknumbuf, "%d", xmodempacknum) == 1)
	{
		//printf(" xmodempacknum %s->%d success \r\n", xmodempacknum);
	}
	else
	{
		LOG("[Xmodem] xmodempacknum %s->%d error \r\n");
		return -3;
	}

	return 0;

}


int switch_to_root_directory(void) {
    char current_path[PATH_MAX];

    if (getcwd(current_path, sizeof(current_path)) == NULL) {
        perror("Failed to get current working directory");
        return errno;
    }

    if (strcmp(current_path, "/") != 0) {
        if (chdir("/") == 0) {
            LOG("[Xmodem] Changed to root directory.\n");
        } else {
            perror("Failed to change to root directory");
            return errno;
        }
    } else {
        // printf("Already in root directory.\n");
    }

    return 0; 
}


signed char SaveOtaFile(char *name, unsigned char *buf, int totalpacknum, int curpackno, int datanum)
{
    int writebytenum = 0;

    if(name == NULL || buf == NULL)
    {
        return -1;
    }
	//switch_to_root_directory();
    // 如果文件还没打开，打开它（第一次包）
    if(OTAfil == NULL)
    {
        OTAfil = fopen(name, "wb+");
        if(OTAfil == NULL)
        {
            perror("Error creating OTA file");
            return -4;
        }
        else
        {
            LOG("[Xmodem] file create successfully!\n");
        }
    }

    writebytenum = fwrite(buf, 1, datanum, OTAfil);
    if(writebytenum != datanum)
    {
        LOG("[Xmodem] xmodem write file %s error! Expected %d bytes, wrote %d bytes\n", name, datanum, writebytenum);
        return -2;
    }

    if(fflush(OTAfil) != 0)
    {
        perror("File synchronization failure");
        return -3;
    }

    if(curpackno == totalpacknum)
    {
        LOG("[Xmodem] The last pack!\n");
        if(fclose(OTAfil) != 0)
        {
            perror("close ota file error");
			LOG("OTAfil:close ota file error");
            OTAfil = NULL;
            return -5;
        }
        OTAfil = NULL; // 记得清空指针
    }

    return 0;
}



unsigned short crcGet(unsigned char *ptr, int len)
{
    unsigned short crc = 0x0000;

    while(len--)
    {
        crc = (crc << 8) ^ crc_table[(crc >> 8 ^ *ptr++) & 0xff];
    }

    return(crc);
}



void delete_files_with_prefix(const char *path, const char *substring) {
    DIR *dir;
    struct dirent *entry;

    dir = opendir(path);
    if (dir == NULL) {
        perror("opendir failed");
        return;
    }

    while ((entry = readdir(dir)) != NULL) {
        char filepath[512];

        // Skip directories
        snprintf(filepath, sizeof(filepath), "%s/%s", path, entry->d_name);

        struct stat st;
        if (stat(filepath, &st) == -1) {
            perror("stat failed");
            continue;
        }
        if (S_ISDIR(st.st_mode)) {
            continue;
        }

        // Check prefix match
        if (strstr(entry->d_name, substring) != NULL){
            if (unlink(filepath) == 0) {
                LOG("[Xmodem] Deleted: %s\n", filepath);
            } else {
                perror("Failed to delete file");
                LOG("[Xmodem] Filename: %s\n", filepath);
            }
        }
    }

    closedir(dir);
}
