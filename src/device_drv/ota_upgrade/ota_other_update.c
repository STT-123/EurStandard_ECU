#define _GNU_SOURCE
#include "ota_other_update.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/log/log.h"
#include "device_drv/xmodem/xmodemdata.h"
#include "device_drv/sd_store/sd_store.h"
#define ACPDC_BLOCK_SIZE 120
IndependentStatus independentStatus = {0};

void printBuffer(uint8_t *buffer, size_t size) {
    for (size_t i = 0; i < size; i++) {
    	LOG("0x%02X ", buffer[i]);  // 打印每个字节的十六进制值
        if ((i + 1) % 16 == 0) {  // 每行打印16个字节
        	LOG("\n");
        }
    }
    LOG("\n");
}

signed char  AcpDcUpgradesend(const char *filename,FILE *rfile)
{
    uint8_t fileBuffer[ACPDC_BLOCK_SIZE];          // 用于存放从文件中读取的原始数据
    uint32_t totalCount = 0;                 // 文件总长度
    uint32_t sendSuccessCount = 0;           // 已发送的字节数
    uint16_t blockCount = 0;                 // 块数量
    unsigned int bytesRead;
    struct timespec acpdcdcota_start = {0};
    uint8_t zeroBuffer[ACPDC_BLOCK_SIZE] = {0};
    CAN_MESSAGE CanMes;
    CAN_MESSAGE CanMes_1;
    CAN_MESSAGE canmsg;
	int TStatus;
	int UStatus;
	int err = 0;
    LOG("filename:%s\r\n", filename);

    // 打开文件
    rfile = fopen(filename, "rb");
    if (rfile == NULL) {
        independentStatus.ErrorReg = 1;
        return 1;
    }
    // 获取文件大小
    fseek(rfile, 0, SEEK_END);
    totalCount = ftell(rfile);
    fseek(rfile, 0, SEEK_SET);

    LOG("Bin OTA file size: %d\r\n", totalCount);
    blockCount = (totalCount + ACPDC_BLOCK_SIZE - 1) / ACPDC_BLOCK_SIZE;
    LOG("blockCount : %d\r\n", blockCount);
    unsigned int percent_count = blockCount/80;
	clock_gettime(CLOCK_MONOTONIC, &acpdcdcota_start);//更新系统启动时间
	queue_clear(&Queue_BCURevData);
    for (uint16_t count = 0; count < blockCount; count++) 
	{
    	unsigned int sum = 0;
    	uint8_t byte = 1;
    	uint16_t blockSizeSet = 0;//当前数据帧的长度
    	uint32_t blockSize = 0;
		unsigned int per = count/percent_count;
		set_modbus_reg_val(OTAPPROGRESSREGADDR, (10+per));//0124,升级进度
        memset(fileBuffer, 0, ACPDC_BLOCK_SIZE);

        // 从文件中读取数据
     	bytesRead = fread(fileBuffer, 1, ACPDC_BLOCK_SIZE, rfile);
        if (bytesRead <= 0) {
            independentStatus.ErrorReg = 2;
            fclose(rfile);
            return 1;
        }

        LOG("bytesRead : %d\r\n", bytesRead);
        printBuffer(fileBuffer, ACPDC_BLOCK_SIZE);
        blockSize = bytesRead -4;
        LOG("blockSize : %d\r\n", blockSize);
        blockSizeSet = bytesRead +2;
        LOG("blockSizeSet : %d\r\n", blockSizeSet);


        uint16_t zhengshu = blockSize / 8;
        LOG("zhengshu : %d\r\n", zhengshu);


        uint16_t yushu = blockSize % 8;
        LOG("yushu : %d\r\n", yushu);
        //发送第一帧
        memset(&CanMes_1, 0 , sizeof(CAN_MESSAGE));
        CanMes_1.Extended = 1;
        CanMes_1.Length = 8U;
        CanMes_1.ID = 0x03160800;
        CanMes_1.Data[0] = 0xB0;
        CanMes_1.Data[1] = bytesRead + 4 + 1;
        uint16_t seqnum = count * 0x1E;
        CanMes_1.Data[2] = (seqnum & 0xFF);        // 低字节
        CanMes_1.Data[3] = ((seqnum >> 8) & 0xFF);  // 高字节
        memcpy(&CanMes_1.Data[4], fileBuffer, 4);

        TStatus = Drv_bcu_can_send(&CanMes_1);
    	if(TStatus ==0)
    	{
    		//发送成功
    		// printf("First frame data send successfully");
    	}
    	else
    	{
    		//发送失败
    		LOG("First frame data send failed");
    		independentStatus.ErrorReg = 3;
            fclose(rfile);
            return 1;
    	}

    	independentStatus.sendCount += 4;
    	memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
        CanMes.Extended = 1;
        CanMes.Length = 8U;
        // 循环发送每8字节的数据帧
        for (int i = 0; i < zhengshu; i++) {
        	usleep(20*1000);
            memcpy(&CanMes.Data[0], &fileBuffer[4 + i * 8], 8);
            CanMes.ID = 0x03160801 + i;
			// printf("CanMes.ID = %x\r\n",CanMes.ID);
			// printf("CanMes.Data = %x %x %x %x %x %x %x %x\r\n",CanMes.Data[0],CanMes.Data[1],CanMes.Data[2],CanMes.Data[3],CanMes.Data[4],CanMes.Data[5],CanMes.Data[6],CanMes.Data[7]);
            TStatus = Drv_bcu_can_send(&CanMes);
			// printf("TStatus = %d\r\n",TStatus);
            if (TStatus != 0) {
            	LOG("zhengshu data send failed");
            	independentStatus.ErrorReg = 4;
                fclose(rfile);
                return 1;
            }
            independentStatus.sendCount += 8;
            byte++;
        }

        usleep(20*1000);
        // 发送余下不足8字节的数据帧
        if (yushu > 0) {
            memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
            CanMes.Extended = 1;
            CanMes.Length = yushu;
            CanMes.ID = 0x03160000 + (yushu << 8) + zhengshu +1;
            memcpy(&CanMes.Data[0], &fileBuffer[4 + zhengshu * 8], (yushu));
			printf(" CanMes.Data[0]:%02x\r\n",CanMes.Data[0]);
			printf(" CanMes.Data[1]:%02x\r\n",CanMes.Data[1]);
			printf(" CanMes.Data[2]:%02x\r\n",CanMes.Data[2]);
			printf(" CanMes.Data[3]:%02x\r\n",CanMes.Data[3]);
			printf(" CanMes.Data[4]:%02x\r\n",CanMes.Data[4]);
			printf(" CanMes.Data[5]:%02x\r\n",CanMes.Data[5]);
			printf(" CanMes.Data[6]:%02x\r\n",CanMes.Data[6]);
			printf(" CanMes.Data[7]:%02x\r\n",CanMes.Data[7]);
            TStatus = Drv_bcu_can_send(&CanMes);
            if (TStatus != 0) {
            	LOG("yushu data send failed");
            	independentStatus.ErrorReg = 5;
                fclose(rfile);
                return 1;
            }
            independentStatus.sendCount += yushu+1;
        }

        usleep(20*1000);
        for (int j = 0; j < ACPDC_BLOCK_SIZE; j++) {
            sum += fileBuffer[j];  // 将每个字节的值累加到 sum
        }
        sum = sum + 0xB0 + bytesRead + 4 + 1 + (seqnum & 0xFF) + ((seqnum >> 8) & 0xFF);
        // printf("sum :%d\r\n",sum);
		//发送校验和
        memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
        CanMes.Extended = 1;
        CanMes.Length = 1U;
        if(yushu == 0)
        {
        	CanMes.ID = 0x03161100 + zhengshu + 1;
        }
        else
        {
            CanMes.ID = 0x03161100 + zhengshu + 2;
        }

        CanMes.Data[0] = sum & 0xff;
        printf("CanMes.Data[0]  :%02x\r\n",CanMes.Data[0]);
        TStatus = Drv_bcu_can_send(&CanMes);
		if(TStatus == 0)
		{
			//成功
			memset(&canmsg, 0, sizeof(CAN_MESSAGE));
			printf("count: %d  \r\n",count);
			if( (count <20))
			{
				independentStatus.XCPCMDOuttimeTime  = 100;
			}
			else
			{
				independentStatus.XCPCMDOuttimeTime  = 5000;
			}
			UStatus = queue_pend(&Queue_BCURevData, &canmsg,&err);
			if(UStatus == 0)
			{
	        	printf(" canmsg.Data[0]:%02x\r\n",canmsg.Data[0]);
	        	printf(" canmsg.Data[1]:%02x\r\n",canmsg.Data[1]);
	        	printf(" canmsg.Data[2]:%02x\r\n",canmsg.Data[2]);
	        	printf(" canmsg.Data[3]:%02x\r\n",canmsg.Data[3]);
	        	printf(" canmsg.Data[4]:%02x\r\n",canmsg.Data[4]);
	        	printf(" canmsg.Data[5]:%02x\r\n",canmsg.Data[5]);
				//成功

				if( (count <20))
				{
					//不执行报错
				}
				else
				{

					if(canmsg.Data[4] == 0x00)
					{

						// printf("count: %d    The data packet was delivered successfully\r\n",count);
						//接受成功
						independentStatus.IndReturnSuccess = true;
					}
					else if(canmsg.Data[4] == 0x01)
					{
						printf("count: %d    The Packet sequence number was delivered failed\r\n",count);
						//包序号错误
						independentStatus.IndReturnSuccess = false;
						independentStatus.ErrorReg = 6;
						fclose(rfile);
						return 1;
					}
					else if(canmsg.Data[4] == 0x08)
					{
						printf("count: %d    The check  data packet was delivered failed\r\n",count);
						independentStatus.ErrorReg = 7;
						fclose(rfile);
						//包校验错误
						return 1;
					}
					else
					{
						printf("unknown error\r\n");
						independentStatus.ErrorReg = 8;
						fclose(rfile);
						//包校验错误
						return 1;
					}
				}


			}
			else
			{
			//失败
				printf("5 seconds timeout,response code data receive failed\r\n");
				if( (GetTimeDifference_ms(acpdcdcota_start) < 18000))
				{
					//不执行报错
				}
				else
				{
					independentStatus.ErrorReg = 9;
					printf("receive failed:%d\r\n",independentStatus.ErrorReg);
					fclose(rfile);
					return 1;
				}

			}
		}
		else
		{
			//失败
			printf("check code data send failed\r\n");
			independentStatus.ErrorReg = 10;
			fclose(rfile);
			return 1;
		}
		usleep(20*1000);

    }
    fclose(rfile);
    return 0;
}


void ACPDCDC_OTA(void )
{
	if (!get_ota_OTAStart()) return;
	
	if( (get_ota_deviceID() != 0) && (get_ota_OTAUdsFilename(0) != 0)  && (get_ota_OTAFileType()  != ECU ) && (get_ota_deviceType() == ACP || get_ota_deviceType() == DCDC))
	{
		FILE *rfile =NULL;
		set_modbus_reg_val(OTASTATUSREGADDR, OTASTARTRUNNING);//0124.升级状态
		memset(&independentStatus, 0, sizeof(IndependentStatus));
        char otafilenamestr1[OTAFILENAMEMAXLENGTH + 64] = {'\0'};
		snprintf(otafilenamestr1, sizeof(otafilenamestr1), "%s/%s", USB_MOUNT_POINT, get_ota_OTAFilename());
		printf("otafilenamestr1 %s\r\n", otafilenamestr1);
		printf("OTAStart:%d,deviceID:%d,OTAFilename:%s,OTAFileType:%d,deviceType:%d\n", get_ota_OTAStart(), get_ota_deviceID(), get_ota_OTAFilename(), get_ota_OTAFileType(), get_ota_deviceType());
        rfile = fopen(otafilenamestr1, "rb"); // Open the file for reading in binary mode
        if(rfile == NULL)
        {
			LOG("[OTA] %s open error, error code %d %s\r\n",otafilenamestr1, errno, strerror(errno));
			independentStatus.ErrorReg |= 1 << 0;
            independentStatus.ErrorDeviceID = get_ota_deviceID();
        }
		set_modbus_reg_val(OTAPPROGRESSREGADDR, 10);//0124,升级进度
		if(independentStatus.ErrorReg == 0)
		{
			independentStatus.XCPCMDOuttimeTimes = 5000;
			independentStatus.XCPCMDOuttimeTime = INDOVERTIME;
			independentStatus.XCPCMDRpeatTimes = 0;

			while(1)
			{
				signed char res = 1;
				if(get_ota_deviceType() == ACP)
				{
					independentStatus.CANStartOTA = 1;

					printf("otafilenamestr1: %s\r\n", otafilenamestr1);
					printf("get_ota_OTAFilename(): %s\r\n", get_ota_OTAFilename());
					queue_clear(&Queue_BCURevData);
					res = AcpDcUpgradesend(otafilenamestr1,rfile);//首帧和连续帧
					usleep(5*1000);

					memset(otafilenamestr1, '\0', OTAFILENAMEMAXLENGTH + 2);

					char *str = "w.bin";

					if (res == 0)
					{
						printf("Block transferred successfully.\r\n");
						set_modbus_reg_val(OTAPPROGRESSREGADDR, 90);//0124,升级进度
					}
					else
					{
						printf("Block transfer failed. Error code: %d\r\n",res);
					}
					break;

				}
				if(get_ota_deviceType() == DCDC)
				{

					independentStatus.CANStartOTA = 1;

					queue_clear(&Queue_BCURevData);
					res = AcpDcUpgradesend(otafilenamestr1,rfile);//首帧和连续帧
					usleep(5*1000);

					memset(otafilenamestr1, '\0', OTAFILENAMEMAXLENGTH + 2);
					// otafilenamestr1[0] = '0';
					// otafilenamestr1[1] = ':';
					char *str = "w.bin";
					// memcpy(&otafilenamestr1[2], str, strlen(str));

					if (res == 0)
					{
						printf("Block transferred successfully.\r\n");
						set_modbus_reg_val(OTAPPROGRESSREGADDR, 90);//0124,升级进度
					}
					else
					{
						printf("Block transfer failed. Error code: %d\r\n",res);

					}
					break;
				}
			}
		}

		if(independentStatus.ErrorReg == 0)
		{
			LOG("[OTA] can id 0x%x device ota success!\r\n", get_ota_deviceID());
			independentStatus.DeviceProgramOkFlag = 1;
			set_modbus_reg_val(OTAPPROGRESSREGADDR, 100);//0124,升级进度
			set_modbus_reg_val(OTASTATUSREGADDR, OTASUCCESS);
			set_ota_UpDating(0);//1130(升级成功)
			independentStatus.CANStartOTA = 0;

		}
		else
		{
			LOG("[OTA] can id 0x%x device ota failed, error register val 0x%x!\r\n", get_ota_deviceID(), independentStatus.ErrorReg);
			set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);

		}

		if(rfile != NULL)
		{
			fclose(rfile);
			rfile = NULL;
		}

		set_ota_OTAStart(0);


	}
}




void FinishACPOtaAndCleanup(void)
{
    set_ota_deviceType(0);             // 停止升级
    set_ota_OTAStart(0);
    delete_files_with_prefix(USB_MOUNT_POINT, "ACP"); // 删除升级文件
	delete_files_with_prefix(USB_MOUNT_POINT, "md5"); // 删除升级文件
    independentStatus.CANStartOTA = 0;
    set_ota_UpDating(0);               // 升级结束标志
	set_ota_acOTAFlag(0);
	set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
    set_modbus_reg_val(OTASTATUSREGADDR, OTAIDLE); // 设置状态寄存器为 IDLE
	CANFDSendFcn_BCU_step();//发送TCU 信息
}

void FinishDCDCOtaAndCleanup(void)
{
    set_ota_deviceType(0);            // 停止升级
    set_ota_OTAStart(0);
    delete_files_with_prefix(USB_MOUNT_POINT, "DCDC"); // 删除升级文件
	delete_files_with_prefix(USB_MOUNT_POINT, "md5"); // 删除升级文件
    independentStatus.CANStartOTA = 0;
    set_ota_UpDating(0);               // 升级结束标志
	set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
    set_modbus_reg_val(OTASTATUSREGADDR, OTAIDLE); // 设置状态寄存器为 IDLE
	CANFDSendFcn_BCU_step();
}