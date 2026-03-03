#define _GNU_SOURCE
#include "ota_xcp_update.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/bmu_deal/bmu_deal.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/log/log.h"
#include "device_drv/xmodem/xmodemdata.h"
#include "device_drv/sd_store/sd_store.h"
#include "interface/can/mycan.h"

XCPStatus xcpstatus = {0};
unsigned int OTA_RecvPacketCount = 0;
signed char XcpSendConnectCMD(unsigned int id, unsigned char xcpobjectid)
{
	CAN_MESSAGE CanMes;
    unsigned int i = 0;
	memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
	CanMes.Extended = 1;
	CanMes.Length = 1;
	CanMes.ID = id;

	CanMes.Data[0] = 0xFF;

    //printf("CanMes.Data[0] = %d\r\n",CanMes.Data[0]);
	//CanMes.Data[1] = 0x00;
	if(xcpobjectid == 0)
	{
		return Drv_bmu_can_send(&CanMes);	
	}
	else
	{
        //printf("CanMes.Data[0] = %x\r\n",CanMes.Data[0]);
		return Drv_bcu_can_send(&CanMes);
	}

}



signed char XCPCANOTAMSGParse(struct can_frame *pCANMsg, XCPStatus *pXCPStatus)
{
	if(pCANMsg == NULL || pXCPStatus== NULL)
	{
        LOG("[OTA] pCANMsg == NULL  pXCPStatus== NULL\r\n");
		return -1;
	}
    // printf("[OTA] pCANMsg->ID: 0x %x\r",pCANMsg->can_id);
	if(pCANMsg->can_dlc == 1)
	{
		if(pCANMsg->data[0] == 0xFF) //步骤3、4、5
		{
            return 0;
		}else{
            return -4;
        }
	}
	else if(pCANMsg->can_dlc == 6)//步骤2
	{
		unsigned char cmpbuf[6] ={0xFF, 0x00, 0x10, 0x00, 0x00, 0x00};
        LOG("[OTA] pCANMsg->ID: %x\r",pCANMsg->can_id);
        LOG("[OTA] pCANMsg->Data[0]: %x\r",pCANMsg->data[0]);
        LOG("[OTA] pCANMsg->Data[1]: %x\r",pCANMsg->data[1]);
        LOG("[OTA] pCANMsg->Data[2]: %x\r",pCANMsg->data[2]);
        LOG("[OTA] pCANMsg->Data[3]: %x\r",pCANMsg->data[3]);
        LOG("[OTA] pCANMsg->Data[4]: %x\r",pCANMsg->data[4]);
        LOG("[OTA] pCANMsg->Data[5]: %x\r",pCANMsg->data[5]);

		if(memcmp(cmpbuf, pCANMsg->data, 6) == 0)
		{
			LOG("[OTA] xcpstatus.DeviceCanProgramFlag -> 6\r\n");
            return 0;
		}
        else{
            return -4;
        }

	}
	else if(pCANMsg->can_dlc == 8)
	{
		if(pCANMsg->data[0] == 0xFF && pCANMsg->data[1] == 0x10)//步骤1
		{
			//xxflag = 1;
            LOG("[OTA] pCANMsg->Data[0]: %x\r",pCANMsg->data[0]);
            LOG("[OTA] pCANMsg->Data[1]: %x\r",pCANMsg->data[1]);
            LOG("[OTA] pCANMsg->Data[2]: %x\r",pCANMsg->data[2]);
            LOG("[OTA] pCANMsg->Data[3]: %x\r",pCANMsg->data[3]);
            LOG("[OTA] pCANMsg->Data[4]: %x\r",pCANMsg->data[4]);
            LOG("[OTA] pCANMsg->Data[5]: %x\r",pCANMsg->data[5]);
            LOG("[OTA] pCANMsg->Data[6]: %x\r",pCANMsg->data[6]);
            LOG("[OTA] pCANMsg->Data[7]: %x\r",pCANMsg->data[7]);
			LOG("[OTA] xcpstatus.DeviceConnectedFlag -> 8\r\n");
            return 0;

		}else{

            return -4;
        }
	}
	else
	{
		LOG("[OTA] Unsupported xcp responsed cmd!\r\n");
		return -2;
	}

    return -3;
}




signed char XcpSendQueryStatusCMD(unsigned int id, unsigned char xcpobjectid)
{
	CAN_MESSAGE CanMes;
    uint8_t i = 0;
	memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
	CanMes.Extended = 1;
	CanMes.Length = 1;
	CanMes.ID = id;
	CanMes.Data[0] = 0xFD;
	if(xcpobjectid == 0)
	{
		return Drv_bmu_can_send(&CanMes);	
	}
	else
	{
		//return CANSendMsg(BCUXCPCANID, &CanMes);

        LOG("[OTA] XcpSendQueryStatusCMD CanMes.Data[0] = 0x%x\r\n",CanMes.Data[0]);

		return Drv_bcu_can_send(&CanMes);        
	}
}



signed char XcpSendProgramMaxCMD(unsigned int id, unsigned char *pbuf, unsigned char payloaddatalen, unsigned char xcpobjectid)
{
	if(pbuf == NULL)
	{
		return -10;
	}

	if(payloaddatalen > 7)
	{
		return -9;
	}
	CAN_MESSAGE CanMes;
    unsigned int i = 0;
	memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
	CanMes.Extended = 1;
	CanMes.Length = payloaddatalen + 1;
	CanMes.ID = id;
	CanMes.Data[0] = 0xC9;

	memcpy(&CanMes.Data[1], pbuf, payloaddatalen);


	if(xcpobjectid == 0)
	{
		return Drv_bmu_can_send(&CanMes);	
	}
	else
	{
		return Drv_bcu_can_send(&CanMes);	
	}
}



static int XCPCANOTAMSGParseMult(XCPStatus *xcpstatus)
{
    int xStatus;
    int err;
    struct timespec xStartTime;
	struct can_frame canmsg;
	memset(&canmsg, 0, sizeof(canmsg));
    clock_gettime(CLOCK_MONOTONIC, &xStartTime); // 初始化起始时间戳

	while (1) 
    {
        if(get_ota_deviceType() == BMU)
        {
            xStatus = queue_pend(&Queue_BMURevData, &canmsg,&err);
        }
        else
        {
            xStatus = queue_pend(&Queue_BCURevData, &canmsg,&err);
        }
	    if (xStatus == 0)
	    {
            // printf("canmsg->data[0] :%02X\r\n ", canmsg.data[0]);
            // printf("canmsg->data[1] :%02X \r\n", canmsg.data[1]);
            // printf("canmsg.data[2] :%02X\r\n ", canmsg.data[2]);
            // printf("canmsg.data[3] :%02X \r\n", canmsg.data[3]);
            // printf("canmsg.data[4] :%02X \r\n", canmsg.data[4]);
            // printf("rame.data[5] :%02X \r\n", canmsg.data[5]);
            // printf("rame.data[6] :%02X \r\n", canmsg.data[6]);
            // printf("rame.data[7] :%02X\r\n ", canmsg.data[7]);
            // printf("rame.data[8] :%02X \r\n", canmsg.data[8]);
	    	int res = XCPCANOTAMSGParse(&canmsg, xcpstatus);
            
	        if (res == 0)
	        {
                // printf("[OTA] get_ota_deviceID(): 0x %x, OTA_RecvPacketCount = %d\r\n", get_ota_deviceID(),OTA_RecvPacketCount++);
	        	return 0;
	        }
            else{
                // LOG("[OTA] XCPCANOTAMSGParse =  %d, canmsg.can_dlc = %d \r\n",res,canmsg.can_dlc);//-4表示内部存在其他值，不是升级的反馈值，BCU不存在，BMU存在
            }
	    }

		if (GetTimeDifference_ms(xStartTime)>1000 )//50->100
		{
            LOG("[OTA] GetTimeDifference_ms(xStartTime) = %d\r\n",GetTimeDifference_ms(xStartTime));
            LOG("[OTA] XCPCANOTAMSGParseMult_timeout\r\n");
			return -2;
		}
        usleep(1000);//1ms休眠，大幅降低CPU占用率
	}
}

signed char XcpSendProgramEndCMD(unsigned int id, unsigned char xcpobjectid)
{

	CAN_MESSAGE CanMes;
	memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
	CanMes.Extended = 1;
	CanMes.Length = 2;
	CanMes.ID = id;
	CanMes.Data[0] = 0xD0;
	CanMes.Data[1] = 0x00;
    LOG("[OTA] XcpSendProgramEndCMD CanMes.Data[0] = 0x%x\r",CanMes.Data[0]);
    LOG("[OTA] XcpSendProgramEndCMD CanMes.Data[1] = 0x%x\r",CanMes.Data[1]);
	if(xcpobjectid == 0)
	{
		return Drv_bmu_can_send(&CanMes);	
	}
	else
	{
		return Drv_bcu_can_send(&CanMes);	
	}
}

signed char XcpSendProgramResetCMD(unsigned int id, unsigned char xcpobjectid)
{

	CAN_MESSAGE CanMes;
	memset(&CanMes, 0 , sizeof(CAN_MESSAGE));
	CanMes.Extended = 1;
	CanMes.Length = 1;
	CanMes.ID = id;
	CanMes.Data[0] = 0xCF;

	if(xcpobjectid == 0)
	{
		return Drv_bmu_can_send(&CanMes);	
	}
	else
	{
		return Drv_bcu_can_send(&CanMes);	
	}
}


static int XcpTryConnectDevice(XCPStatus *xcpstatus)
{
    if(xcpstatus->ErrorReg == 0)
    {

        int xStatus = 1;
        int err;
        if (xcpstatus->ErrorReg != 0)
            return -1;

        signed char res = -1;
        int Connectcount = 10;
        while(Connectcount > 0)
        {
            if (get_ota_deviceType() == BMU)
            {
                res = XcpSendConnectCMD(get_ota_deviceID(), 0);
                LOG("[OTA] XcpSendConnectCMD_res:%d\r\n",res);
            }
            else
            {
                res = XcpSendConnectCMD(get_ota_deviceID(), 1);//发送0xFF
                LOG("[OTA] XcpSendConnectCMD_end res:%d\r\n",res);
            }

            int result = XCPCANOTAMSGParseMult(xcpstatus);
            if (result == 0) {
                return 0;
            }
            else
            {
                LOG("Connectcount Connectcount =%d \r",Connectcount);//LOG 会执行printf和log两个函数，所以会执行两遍计算
                Connectcount--;
                usleep(10*1000);
                continue;
            }
        }

        memset(xcpstatus, 0, sizeof(XCPStatus));
        xcpstatus->ErrorReg |= 1 << 15;
        xcpstatus->ErrorDeviceID = get_ota_deviceID();
        return -2; // 超时错误
    }  
    return -3;
}

#if 1
static int XcpTryQueryStatusOnce(XCPStatus *xcpstatus)
{
    if(xcpstatus->ErrorReg == 0)
    {
        int xStatus = 1;
        int err;

        unsigned char tmp = 0;
        unsigned int times = 0;
        signed char res = -1;
        int Querycount = 10;
        while(Querycount > 0)
        {
            if (get_ota_deviceType() == BMU)
            {
                res = XcpSendQueryStatusCMD(get_ota_deviceID(), 0);
            }
            else
            {
                LOG("[OTA] XcpSendQueryStatusCMD\r\n");
                xcpstatus->CANStartOTA = 1;
                res = XcpSendQueryStatusCMD(get_ota_deviceID(), 1);
                LOG("[OTA] XcpSendQueryStatusCMD res :%d\r\n",res);
            }

            if (res < 0)
            {
                LOG("[OTA] XCP SendQueryStatusCMD error, Error code %d\r\n", res);
                xcpstatus->ErrorReg |= 1 << 4;
                xcpstatus->ErrorDeviceID = get_ota_deviceID();
                return -1;
            }
            LOG("[OTA] xQueueReceive_ing\r\n");

            int result = XCPCANOTAMSGParseMult(xcpstatus);
            if (result == 0) {
                return 0;
            }
            else
            { 
                LOG("Query Querycount =%d \r",Querycount);
                Querycount--;
                usleep(10*1000);
                continue;
            }
        }

        memset(xcpstatus, 0, sizeof(XCPStatus));
        xcpstatus->ErrorReg |= 1 << 15;
        xcpstatus->ErrorDeviceID = get_ota_deviceID();
        return -2; // 超时错误
    }
    return -3;
}
   #endif

static int  SendOTACommand(unsigned char *buf, unsigned int len, XCPStatus *xcpstatus, unsigned int i, unsigned int totalpack)
{
    if ( xcpstatus == NULL) {
            LOG("[OTA] Error: Null pointer passed to XCPCANOTAMSGParseMult.\n");
            return -1; 
        }

    xcpstatus->XCPCMDResponseFlag = 0;
    unsigned int per = 0;
    if (totalpack > 0) {
        per = (i * 100U) / totalpack; // map packet index to 0~99
        if (per > 99U) {
            per = 99U;
        }
    }
    
    signed char res = -1;
    if(get_ota_deviceType() == BMU)
    {
        res = XcpSendProgramMaxCMD(get_ota_deviceID(), buf, len, 0);
    }
    else
    {
        set_modbus_reg_val(OTAPPROGRESSREGADDR, per); // BCU OTA progress: 0~99, 100 is set on final success
        res = XcpSendProgramMaxCMD(get_ota_deviceID(), buf, len,1);
    }
    if (res != 0) {
        LOG("[OTA] XCP XcpSendProgramMaxCMD error, Error code %d\r\n", res);
        xcpstatus->ErrorReg |= 1 << 7;
        xcpstatus->ErrorDeviceID = get_ota_deviceID();
        return 1;
    }

    int result = XCPCANOTAMSGParseMult(xcpstatus);
    if (result == 0) {
        return 0;
    }
    else
    {
        memset(xcpstatus, 0, sizeof(XCPStatus));
        xcpstatus->ErrorReg |= 1 << 8;
        xcpstatus->ErrorDeviceID = get_ota_deviceID();

        return -2; // 超时错误
    }
}



static int SendLastPacket(FILE*rfile, unsigned char lastpackdatanum, XCPStatus *xcpstatus) {
    signed char res;
     int xStatus;
     int err;
    unsigned char buf[7] = {0};
    size_t rnum = 0;
     rnum = fread(buf, 1, lastpackdatanum, rfile);
    // printf("file read %d byte data success!\r\n", rnum);
    if (rnum != lastpackdatanum) {
        LOG("[OTA] file read %d byte data failed!\r\n", lastpackdatanum);
        xcpstatus->ErrorReg |= 1 << 6;
        xcpstatus->ErrorDeviceID = get_ota_deviceID();
        return -1;
    } else {
        LOG("[OTA] file read %d byte data success!\r\n", rnum);
    }

    xcpstatus->XCPCMDOuttimeTimes = 1;
    xcpstatus->XCPCMDOuttimeTime = XCPOVERTIME;
    xcpstatus->XCPCMDRpeatTimes = 0;

    unsigned int times = 0;

    xcpstatus->XCPCMDResponseFlag = 0;
    if(get_ota_deviceType() == BMU)
    {
        res = XcpSendProgramMaxCMD(get_ota_deviceID(), buf, lastpackdatanum, 0);
    }
    else
    {
        res = XcpSendProgramMaxCMD(get_ota_deviceID(), buf, lastpackdatanum,1);
        LOG("[OTA] SendLastPacket XcpTryProgramOnce   recv res: %d\r\n",res);
    }
    if (res != 0) {
        LOG("[OTA] XCP XcpSendProgramCMD SendLastPacket error, Error code %d\r\n", res);
        xcpstatus->ErrorReg |= 1 << 11;
        xcpstatus->ErrorDeviceID = get_ota_deviceID();
        return -1;
    }
    else
    {

    }

    int result = XCPCANOTAMSGParseMult(xcpstatus);
    if (result == 0) {
        // usleep(2*1000);
        return 0;
    }
    else
    {
        memset(xcpstatus, 0, sizeof(XCPStatus));
        xcpstatus->ErrorReg |= 1 << 12;
        xcpstatus->ErrorDeviceID = get_ota_deviceID();
        return -2; // 超时错误
    }
}

static int ReadFileAndSendData(FILE *rfile, XCPStatus *xcpstatus) 
{
    if(xcpstatus->ErrorReg == 0 && get_ota_OTAFileType() == 0)
    {
        unsigned char PrvProgramProgress = 0;
        unsigned char ProgramProgress = 0;
        int fd = fileno(rfile); 
        struct stat file_stat;
        if (fstat(fd, &file_stat) != 0) {
            perror("fstat failed");
            return 1;
        }
        unsigned int filesize = file_stat.st_size;
        LOG("[OTA] Bin ota file size %d\r\n", filesize);

        unsigned char lastpackdatanum = filesize % 7;
        LOG("[OTA] lastpackdatanum %d\r\n", lastpackdatanum);

        unsigned int totalpack = (filesize - lastpackdatanum) / 7;
        LOG("[OTA] totalpack %d\r\n", totalpack);

        lastpackdatanum = filesize - (totalpack * 7);
        LOG("[OTA] lastpackdatanum %d\r\n", lastpackdatanum); 
        fseek(rfile, 0, SEEK_SET);         

        unsigned char FileBuff[490] = {0};
        unsigned char FileCount = 0;

        if (lastpackdatanum != 0) 
        {
            totalpack += 1;
            unsigned int i = 0;
           
            for (i = 0; i < totalpack - 1; i++) 
            {
                unsigned char buf[7] = {0};
                size_t rnum = 0;
               
                if (FileCount == 0 || FileCount >= 70) 
                {
                    int remaining_data = (totalpack - i) * 7;
                    int bytes_to_read = (remaining_data < sizeof(FileBuff)) ? remaining_data : sizeof(FileBuff);
                    rnum = fread(FileBuff, 1, bytes_to_read, rfile);
                   
                    if (rnum < 7) {
                        LOG("[OTA] file read 7 byte data failed! rnum: %zu\n", rnum);
                        xcpstatus->ErrorReg |= 1 << 6;
                        xcpstatus->ErrorDeviceID = get_ota_deviceID();
                       
                        return 2;
                    }
                   
                    FileCount = 0;
                }
                memcpy(buf, &FileBuff[FileCount * 7], 7);
                rnum = 7;
                FileCount++;
                if(SendOTACommand(buf, 7, xcpstatus, i, totalpack) != 0){
                    return 1;
                }
                if(xcpstatus->ErrorReg != 0)
                {
                    LOG("[OTA] if(xcpstatus.ErrorReg != 0)");
                    return 1;
                }
                ProgramProgress = (int)((float)i/totalpack*100);

                if(ProgramProgress != PrvProgramProgress)
                {
                    //RTOSDebugPrintf(".\r\n", ProgramProgress);
                }
                PrvProgramProgress = ProgramProgress;
            }
            // printf("totalpack %d, lastpackdatanum %d\r\n", totalpack, lastpackdatanum);
            if (xcpstatus->ErrorReg == 0) 
            {
                fseek(rfile, (totalpack-1)* 7, SEEK_SET);  // 设置到最后一包开头
                SendLastPacket(rfile,lastpackdatanum, xcpstatus);
    

                ProgramProgress = (int)((float)i/totalpack*100);

                if(ProgramProgress != PrvProgramProgress)
                {
                    //RTOSDebugPrintf("program %d%!\r\n", ProgramProgress);
                }
                PrvProgramProgress = ProgramProgress;
            }

        } 
        else 
        {
            LOG("[OTA] Total programmax pack %d\r\n", totalpack);
            for (int i = 0; i < totalpack; i++) 
            {
                unsigned char buf[7] = {0};
                size_t rnum = 0;

                if (FileCount == 0 || FileCount >= 70) 
                {
                    int remaining_data = (totalpack - i) * 7;
                    int bytes_to_read = (remaining_data < sizeof(FileBuff)) ? remaining_data : sizeof(FileBuff);
                    rnum = fread(FileBuff, 1, bytes_to_read, rfile);

                    if (rnum < 7) {
                        LOG("[OTA] file read 7 byte data failed! rnum: %zu\n", rnum);
                        xcpstatus->ErrorReg |= 1 << 6;
                        xcpstatus->ErrorDeviceID = get_ota_deviceID();
                        return 3;
                    }
                    FileCount = 0;
                }
                memcpy(buf, &FileBuff[FileCount * 7], 7);
                rnum = 7;
                FileCount++;
                if(SendOTACommand(buf, 7, xcpstatus, i, totalpack) != 0){
                    return 1;
                }
                ProgramProgress = (int)((float)i/totalpack*100);

                if(ProgramProgress != PrvProgramProgress)
                {

                    if(ProgramProgress == 100)
                    {
                        //printf("program 100!\r\n");
                    }
                    //printf("program %d%!\r\n", ProgramProgress);
                }
                PrvProgramProgress = ProgramProgress;
                //printf("program %d!\r\n", (int)((float)i/totalpack*100));
            }
        }
        return 0;

    }
    else
    {
        return 4;
    }
   
}





// 封装XCP命令发送和响应接收处理逻辑
static int SendXcpProgramEndCommand(XCPStatus *xcpstatus) {
    signed char res = -1;
    xcpstatus->XCPCMDResponseFlag = 0;
    unsigned int times = 0;
    unsigned int ProgramCount = 10;

    while(ProgramCount>0)
    {
        if (get_ota_deviceType() == BMU) {
            res = XcpSendProgramEndCMD(get_ota_deviceID(), 0);
        } else {
            xcpstatus->CANStartOTA = 1; // 1126
            res = XcpSendProgramEndCMD(get_ota_deviceID(), 1);
        }

        if (res != 0) {
            LOG("[OTA]  XCP XcpSendProgramEndCMD error, Error code %d\r\n", res);
            memset(xcpstatus, 0, sizeof(XCPStatus));
            xcpstatus->ErrorReg |= 1 << 14;
            xcpstatus->ErrorDeviceID = get_ota_deviceID();
            return -1; // 返回错误代码
        }
        
        int result = XCPCANOTAMSGParseMult(xcpstatus);
        if (result == 0) {
            return 0;
        }
        else
        {
            LOG("ProgramCount ProgramCount: %d\n", ProgramCount);
            ProgramCount--;
            usleep(10*1000);
            continue;
        }
    }
    memset(xcpstatus, 0, sizeof(XCPStatus));
    xcpstatus->ErrorReg |= 1 << 15;
    xcpstatus->ErrorDeviceID = get_ota_deviceID();

    return -2; // 超时错误
}

static int HandleXcpCommunication( XCPStatus *xcpstatus) {
    if(xcpstatus->ErrorReg == 0)
    {
        xcpstatus->XCPCMDOuttimeTimes = 1;
        xcpstatus->XCPCMDOuttimeTime = XCPOVERTIME;
        xcpstatus->XCPCMDResponseFlag = 0;

        unsigned int times = 0;

        int sendResult = SendXcpProgramEndCommand(xcpstatus);// 发送XCP命令
        // printf("sendResult %d\r\n", sendResult);
        if (sendResult != 0) {
            return -1;
        }
        else{
            return  0;
        }       
    } 
    return -3;
}




signed char XcpProgramResetHandler(XCPStatus *xcpstatus)
{	
    if(xcpstatus->ErrorReg == 0)
    {
        xcpstatus->XCPCMDResponseFlag = 0;

        unsigned int times = 0;
        int xStatus;
        int err;
        signed char res = -1;
        unsigned int ResetCount = 10;

        while(ResetCount > 0)
        {
            if (get_ota_deviceType() == BMU)
            {
                res = XcpSendProgramResetCMD(get_ota_deviceID(), 0);
            }
            else
            {
                xcpstatus->CANStartOTA = 1; // 1126
                res = XcpSendProgramResetCMD(get_ota_deviceID(), 1);
            }

            if (res != 0)
            {
                LOG("[OTA] XCP XcpSendProgramResetCMD error, Error code %d\r\n", res);
                memset(xcpstatus, 0, sizeof(XCPStatus));
                xcpstatus->ErrorReg |= (1 << 9);
                xcpstatus->ErrorDeviceID = get_ota_deviceID();
                return -1; // 发送失败
            }

            int result = XCPCANOTAMSGParseMult(xcpstatus);
            if (result == 0) {
                return 0;
            }
            else
            {
                LOG("ResetCount ResetCount: %d\n", ResetCount);
                ResetCount--;
                usleep(10*1000);
                continue;
            }
        }
        memset(xcpstatus, 0, sizeof(XCPStatus));
        xcpstatus->ErrorReg |= (1 << 10);
        xcpstatus->ErrorDeviceID = get_ota_deviceID();
        return -2; // 超时错误
     
    }
    return -3;
}
void XCP_OTA(int count)
{
    int ret = 0;
    if (!get_ota_OTAStart()) return;
    if(get_ota_deviceID() != 0 &&  get_ota_OTAFilename() != 0 && get_ota_OTAFileType() != ECU && (get_ota_deviceType() == BCU || get_ota_deviceType() == BMU))
    {
        memset(&xcpstatus, 0, sizeof(XCPStatus));
        FILE *rfile = NULL;
        LOG("[OTA] OTAing.....................\r\n");
        OTA_RecvPacketCount = 0;//接收包计数为0
        if(count == 0){//BMU 不用每次都解压
            ret = unzipfile(USB_MOUNT_POINT,(unsigned int *)&xcpstatus.ErrorReg,FILE_TYPE_BIN);
            if(ret < 0){
                goto xcpcleanup;
            }
        }else{
            // do nothing
        }
        if(xcpstatus.ErrorReg == 0)
        {
            char otafilenamestr1[OTAFILENAMEMAXLENGTH + 64] = {'\0'};
            char filenametmp[256];

            strncpy(filenametmp, get_ota_OTAFilename(), sizeof(filenametmp) - 1);
            filenametmp[sizeof(filenametmp) - 1] = '\0';
            
            char *dot = strrchr(filenametmp, '.');// 直接替换扩展名
            if (dot) {
                strcpy(dot, ".bin");
            }
            snprintf(otafilenamestr1, sizeof(otafilenamestr1), "%s/%s", USB_MOUNT_POINT, filenametmp);
            
            LOG("[OTA] otafilenamestr1 %s\r\n", otafilenamestr1);
            LOG("[OTA] OTAStart:%d, deviceID:0x%x, OTAFilename:%s, OTAFileType:%d, deviceType:%d\r\n", get_ota_OTAStart(), get_ota_deviceID(), filenametmp, get_ota_OTAFileType(), get_ota_deviceType());
            rfile = fopen(otafilenamestr1, "rb");  // "rb" = 只读，二进制
            if (rfile == NULL)
            {
                LOG("[OTA] %s open error, error code %d %s\r\n",otafilenamestr1, errno, strerror(errno));
                xcpstatus.ErrorReg |= 1 << 1;
                xcpstatus.ErrorDeviceID = get_ota_deviceID();
                goto xcpcleanup;
            }
            else
            {
                LOG("[OTA] xcpota %s open success\n", filenametmp);
            }
        }

        sleep(2);
        ret = XcpTryConnectDevice(&xcpstatus);

        if(ret == 0)
        {
            LOG("[OTA] OK_First_XcpTryConnectDevice, get_ota_deviceID() = 0x %x\r\n",get_ota_deviceID());
        }
        else
        {
            LOG("[OTA] ERROR_First_XcpTryConnectDevice, error code %d\r\n", ret);
            goto xcpcleanup;
        }

        sleep(2);
        ret =  XcpTryQueryStatusOnce(&xcpstatus);

        if(ret == 0)
        {
            LOG("[OTA] OK_Second_XcpTryQueryStatusOnce, get_ota_deviceID() = 0x %x\r\n",get_ota_deviceID());
        }
        else
        {
            LOG("[OTA] ERROR_Second_XcpTryQueryStatusOnce, error code %d\r\n", ret);
            goto xcpcleanup;
        }


        sleep(2);

        ret =  ReadFileAndSendData(rfile,&xcpstatus);
        if(ret == 0)
        {
            LOG("[OTA] OK_Third_ReadFileAndSendData get_ota_deviceID() = 0x %x\r\n",get_ota_deviceID());
        }
        else
        {
            LOG("[OTA] ERROR_Third_ReadFileAndSendData, error code %d\r\n", ret);
            goto xcpcleanup;
        }

        sleep(2);  

        ret =  HandleXcpCommunication(&xcpstatus);

        if(ret == 0)
        {
            LOG("[OTA] OK_Fourth_HandleXcpCommunication get_ota_deviceID() = 0x %x\r\n",get_ota_deviceID());
        }
        else
        {
            LOG("[OTA] ERROR_Fourth_HandleXcpCommunication, error code %d\r\n", ret);
            goto xcpcleanup;
        }


        ret =  XcpProgramResetHandler(&xcpstatus);
        if(ret == 0)
        {
            LOG("[OTA] OK_Fifth_XcpProgramResetHandler get_ota_deviceID() = 0x %x\r\n",get_ota_deviceID());
        }
        else
        {
            LOG("[OTA] ERROR_Fifth_XcpProgramResetHandler, error code %d\r\n", ret);
             goto xcpcleanup;
        }
xcpcleanup:
        if(rfile != NULL)
        {
            fclose(rfile);
            rfile = NULL;
        }
    }
}



void FinshhBCUBMUOtaAndCleanup(void)
{
    set_ota_deviceType(0);//停止升级
    set_ota_OTAStart(0);
	delete_files_with_prefix(USB_MOUNT_POINT, "XC");//  这个要删除升级文件，判断xcpstatus状态，成功或者失败删除
    delete_files_with_prefix(USB_MOUNT_POINT, "md5"); // 删除升级文件
    delete_files_with_prefix(USB_MOUNT_POINT, "tar"); 

	set_ota_UpDating(0);//1130(升级结束)
	memset(&xcpstatus,0,sizeof(xcpstatus));
    set_OTA_XCPConnect(0);//删除跳转到BOOT的条件,OTA_XCPConnect为0xFF才会跳转到BOOT
    set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
	set_modbus_reg_val(OTASTATUSREGADDR, OTAIDLE);
    CANFDSendFcn_BCU_step();
}


