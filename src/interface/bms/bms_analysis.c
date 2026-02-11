#include <arpa/inet.h> // htonl
#include <byteswap.h>  // bswap_16, bswap_32
#include "bms_analysis.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/bms/bms_simulink/CANSendFcn.h"
#include "interface/bms/bms_simulink/CANFDRcvFcn_BCU.h"
#include "interface/bms/bms_simulink/CANRcvFcn_BMU.h"
#include "interface/log/log.h"
#include <stdatomic.h>
#include <pthread.h>  // 必须包含这个头文件

extern CAN_FD_MESSAGE_BUS CANSendMsg; 
// ==================== 全局变量 ====================
static CanDataBuffer bcu1_data_buffer = {0};
static CanDataBuffer bcu2_data_buffer = {0};  // 新增：bcu2缓冲区
static PostChangeBuffer post_change_buffer = {0};
static CommandTracking cmd_track = {0};
static TcuSendBuffer tcu_send_buffer = {0};
// --- 新增：变化记录状态 ---
static int change_recording_active = 0;
static CAN_FD_MESSAGE post_tcu_frames[DATA_BUFFER_SIZE];
static int post_tcu_collected = 0;
static CAN_FD_MESSAGE post_bcu2_frames[DATA_BUFFER_SIZE];  // 新增：bcu2变化后帧
static int post_bcu2_collected = 0;  // 新增：bcu2已收集计数
// 初始化缓冲区

void my_modbus_set_float_badc(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    i = htonl(i);
    dest[0] = (uint16_t)bswap_16(i >> 16);
    dest[1] = (uint16_t)bswap_16(i & 0xFFFF);
}

/**
 * 将标准 can_frame 转换为 CAN_FD_MESSAGE_BUS
 * 由于Matlab的输入只有一个CAN_FD_MESSAGE_BUS类型的，所以只能这么转换
 */
void ConvertCANToBus(const struct can_frame *frame, CAN_FD_MESSAGE_BUS *msg)
{
    if (!frame || !msg){
        LOG("[BMS] eeee Raw can_id      : 0x%08lX\n", frame->can_id);
        return;
    }

    msg->ID = frame->can_id & CAN_EFF_MASK; // 取 29 位
    msg->Extended = 1;
    msg->Remote = 0;
    msg->Error = 0;
    msg->Length = frame->can_dlc;
    
    msg->Timestamp = 0;
    memcpy(msg->Data, frame->data, frame->can_dlc);
}

void ConvertCANFDToBus(struct canfd_frame *frame, CAN_FD_MESSAGE_BUS *msg)
{
    if (!frame || !msg)
        return;
    
    msg->ID = frame->can_id & CAN_EFF_MASK;// 提取 ID，去掉扩展/远程/错误标志位
    msg->ProtocolMode = 1; // 1 表示 CAN FD
    msg->Extended = 1;
    msg->Remote = 0;
    msg->Error = 0;
    msg->BRS = 0;
    msg->ESI = 0;
    msg->Length = frame->len;
    msg->DLC = frame->len;
    msg->Reserved = 0;
    msg->Timestamp = 0;
    memcpy(msg->Data, frame->data, frame->len);
}

void ConvertBusToCANFD(const CAN_FD_MESSAGE_BUS *msg, struct canfd_frame *frame)
{
    if (!msg || !frame)
        return;

    // 清空目标结构体
    memset(frame, 0, sizeof(*frame));

    // 设置 CAN ID 和标志位
    frame->can_id = msg->ID;
    // 添加扩展帧标志（EFF）
    if (msg->Extended)
    {
        frame->can_id |= CAN_EFF_FLAG;
    }

    // 添加远程帧标志（RTR）
    if (msg->Remote)
    {
        frame->can_id |= CAN_RTR_FLAG;
    }

    // 添加错误帧标志（ERR）
    if (msg->Error)
    {
        frame->can_id |= CAN_ERR_FLAG;
    }

    // 设置数据长度
    frame->len = msg->Length;// 设置 CAN FD 特有标志（BRS 和 ESI）
    
    frame->flags = 0;
    if (msg->BRS)
    {
        frame->flags |= CANFD_BRS;
    }
    if (msg->ESI)
    {
        frame->flags |= CANFD_ESI;// canfd 的错误帧状态
    }
    memcpy(frame->data, msg->Data, msg->Length);

}

void Convert_CAN_MESSAGE_to_can_frame(const CAN_MESSAGE *msg, struct can_frame *frame)
{

    memset(frame, 0, sizeof(struct can_frame));

    frame->can_id = msg->ID;

    if (msg->Extended)
    {
        frame->can_id |= CAN_EFF_FLAG;
    }

    if (msg->Remote)
    {
        frame->can_id |= CAN_RTR_FLAG;
    }

    if (msg->Error)
    {
        frame->can_id |= CAN_ERR_FLAG;
    }

    if (msg->Length <= 8)
    {
        frame->can_dlc = msg->Length;
    }
    else
    {
        frame->can_dlc = 8;
    }

    memcpy(frame->data, msg->Data, frame->can_dlc);
}

void Convert_canfd_frame_to_CAN_MESSAGE(const struct canfd_frame *frame, CAN_MESSAGE *msg)
{
    memset(msg, 0, sizeof(CAN_MESSAGE));
    msg->ID = frame->can_id & CAN_EFF_MASK;// 提取 CAN ID

    msg->Extended = 1;
    msg->Remote = 0;
    msg->Error = 0;

    msg->Length = (frame->len > 8) ? 8 : frame->len;

    msg->Timestamp = 0;

    // 拷贝数据
    memcpy(msg->Data, frame->data, msg->Length);
}

void Convert_canfd_frame_to_can_fram(const struct canfd_frame *frame, struct can_frame *msg)
{
    memset(msg, 0, sizeof(struct can_frame));
    msg->can_id = frame->can_id & CAN_EFF_MASK;// 提取 CAN ID

    msg->__res0 = 0;
    msg->__pad = 0;
    msg->__res1 = 0;

    msg->can_dlc = (frame->len > 8) ? 8 : frame->len;

    // 拷贝数据
    memcpy(msg->data, frame->data, msg->can_dlc);
}

void Convert_can_frame_to_CAN_MESSAGE(const struct can_frame *frame, CAN_MESSAGE *msg)
{
    memset(msg, 0, sizeof(CAN_MESSAGE)); 
    msg->ID = frame->can_id & CAN_EFF_MASK;// 提取 ID，去掉扩展/远程/错误标志位

    msg->Extended = 1;
    msg->Remote = 0;
    msg->Error = 0;

    // 数据长度控制在 0~8 之间
    if (frame->can_dlc <= 8)
    {
        msg->Length = frame->can_dlc;
    }
    else
    {
        msg->Length = 8;
    }

    memcpy(msg->Data, frame->data, frame->can_dlc);

#ifndef TIMESTAMP_NOT_REQUIRED
    // 如果定义了时间戳字段，初始化为 0.0（无实际来源）
    msg->Timestamp = 0.0;
#endif
}



void Set_BCU_Voltage(float voltage)
{
    uint16_t temp[2] = {0};
    // printf("voltage: %f\n", voltage);
    float adjusted_voltage = voltage;
    my_modbus_set_float_badc(adjusted_voltage, temp);
    set_modbus_reg_val(MDBUS_ADDR_DC_VOL, temp[0]);
    set_modbus_reg_val(MDBUS_ADDR_DC_VOL + 1, temp[1]);
}

void Set_BCU_Current(float current)
{
    uint16_t temp[2] = {0};
    // printf("current: %f\n", current);
    float adjusted_current = current;
    my_modbus_set_float_badc(adjusted_current, temp);
    set_modbus_reg_val(MDBUS_ADDR_DC_CUR, temp[0]);
    set_modbus_reg_val(MDBUS_ADDR_DC_CUR + 1, temp[1]);
}

void Set_BCU_Power(int32_t power_watt)
{
    // float power_kw = (float)power_watt / 1000.0f;
    // printf("power_watt: %f\n", power_watt);
    uint16_t temp[2] = {0};
    my_modbus_set_float_badc(power_watt, temp);
    set_modbus_reg_val(MDBUS_ADDR_DC_POW, temp[0]);
    set_modbus_reg_val(MDBUS_ADDR_DC_POW + 1, temp[1]);
}

void Set_BCU_PositiveEnergy(float energy_wh)
{
    uint32_t energy_mwh = energy_wh * 1000;
    // printf("energy_mwh: %d\n", energy_mwh);
    set_modbus_reg_val(MDBUS_ADDR_P_ENERGY, energy_mwh & 0xFFFF);
    set_modbus_reg_val(MDBUS_ADDR_P_ENERGY + 1, energy_mwh >> 16);
}

void Set_BCU_NegativeEnergy(float energy_wh)
{
    uint32_t energy_mwh = energy_wh * 1000;
    // printf("energy_mwh: %d\n", energy_mwh);
    set_modbus_reg_val(MDBUS_ADDR_N_ENERGY, energy_mwh & 0xFFFF);
    set_modbus_reg_val(MDBUS_ADDR_N_ENERGY + 1, energy_mwh >> 16);
}

void set_OTA_XCPConnect(uint8_T value) { OTA_XCPConnect = value; }
uint8_T get_OTA_XCPConnect(void) { return OTA_XCPConnect; }

void set_TCU_ACMuteSet(uint8_T value) { TCU_ACMuteSet = value; }
uint8_T get_TCU_ACMuteSet(void) { return TCU_ACMuteSet; }

void set_TCU_BCUEINSet(uint32_T value) { TCU_BCUEINSet = value; }
uint32_T get_TCU_BCUEINSet(void) { return TCU_BCUEINSet; }

void set_TCU_ChargerWorkSts(uint8_T value) { TCU_ChargerWorkSts = value; }
uint8_T get_TCU_ChargerWorkSts(void) { return TCU_ChargerWorkSts; }


void set_TCU_ECOMode(uint8_T value) { TCU_ECOMode = value; }
uint8_T get_TCU_ECOMode(void) { return TCU_ECOMode; }


void set_TCU_LifeCounter(uint8_T value) { TCU_LifeCounter = value; }
uint8_T get_TCU_LifeCounter(void) { return TCU_LifeCounter; }

void set_TCU_PowerUpCmd(uint8_T value) { TCU_PowerUpCmd = value; }
uint8_T get_TCU_PowerUpCmd(void) { return TCU_PowerUpCmd; }

void set_TCU_TimeCalFlg(uint8_T value) { TCU_TimeCalFlg = value; }
uint8_T get_TCU_TimeCalFlg(void) { return TCU_TimeCalFlg; }

void set_TCU_TimeDay(uint8_T value) { TCU_TimeDay = value; }
uint8_T get_TCU_TimeDay(void) { return TCU_TimeDay; }

void set_TCU_TimeHour(uint8_T value) { TCU_TimeHour = value; }
uint8_T get_TCU_TimeHour(void) { return TCU_TimeHour; }

void set_TCU_TimeMinute(uint8_T value) { TCU_TimeMinute = value; }
uint8_T get_TCU_TimeMinute(void) { return TCU_TimeMinute; }

void set_TCU_TimeMonth(uint8_T value) { TCU_TimeMonth = value; }
uint8_T get_TCU_TimeMonth(void) { return TCU_TimeMonth; }

void set_TCU_TimeSecond(uint8_T value) { TCU_TimeSecond = value; }
uint8_T get_TCU_TimeSecond(void) { return TCU_TimeSecond; }

void set_TCU_TimeWeek(uint8_T value) { TCU_TimeWeek = value; }
uint8_T get_TCU_TimeWeek(void) { return TCU_TimeWeek; }

void set_TCU_TimeYear(uint8_T value) { TCU_TimeYear = value; }
uint8_T get_TCU_TimeYear(void) { return TCU_TimeYear; }

void set_TCU_FcnStopSet(uint8_T value) { TCU_FcnStopSet = value; }
uint8_T get_TCU_FcnStopSet(void) { return TCU_FcnStopSet; }

void set_TCU_HighVoltType(uint8_T value) { TCU_HighVoltType = value; }
uint8_T get_TCU_HighVoltType(void) { return TCU_HighVoltType; }

void set_TCU_HighVoltValue(uint16_T value) { TCU_HighVoltValue = value; }
uint16_T get_TCU_HighVoltValue(void) { return TCU_HighVoltValue; }


uint8_T get_BCU_TimeYearValue(void) { return BCU_TimeYear; }
uint8_T get_BCU_TimeMonthValue(void) { return BCU_TimeMonth; }
uint8_T get_BCU_TimeDayValue(void) { return BCU_TimeDay; }
uint8_T get_BCU_TimeHourValue(void) { return BCU_TimeHour; }
uint8_T get_BCU_TimeMinuteValue(void) { return BCU_TimeMinute; }
uint8_T get_BCU_TimeSencondValue(void) { return BCU_TimeSencond; }



uint32_T get_BCU_FaultInfoLv1Value(void) { return BCU_FaultInfoLv1; }
uint32_T get_BCU_FaultInfoLv2Value(void) { return BCU_FaultInfoLv2; }
uint32_T get_BCU_FaultInfoLv3Value(void) { return BCU_FaultInfoLv3; }
uint32_T get_BCU_FaultInfoLv4Value(void) { return BCU_FaultInfoLv4; }
uint16_T get_BCU_SOCValue(void) { return BCU_SOC; }
uint16_T get_BCU_SystemWorkModeValue(void) { return BCU_SystemWorkMode; }


uint16_T *get_BCU_usSingleBatVal(void) {
    return (uint16_T *)&usSingleBatVal[0];
}
uint16_T *get_BCU_usSingleBatTemp(void) {
    return (uint16_T *)&usSingleBatTemp[0];
}

uint16_T *get_BMU_DAq_version(void){
    return (uint16_T *)&DAq_version[0];
}

uint32_T get_BMU_DAqX_FaultCode1_at(int idx) {
    return DAqX_FaultCode1[idx];
}
uint32_T get_BMU_DAqX_FaultCode2_at(int idx) {
    return DAqX_FaultCode2[idx];
}

int32_T get_BCU_iDcPower(void) {

    return (int)BCU_RealtimePower;  /* 返回 */
}
unsigned long long get_BCU_ullPosEleQuantity(void) {

    return (unsigned long long)BCU_EngryAccumulateDisChrg;  /* 返回 */
}
unsigned long long get_BCU_ullNegEleQuantity(void) {
    return (unsigned long long)BCU_EngryAccumulateChrg;  /* 返回 */
}
uint16_T get_BCU_usAirState(void) {
    return Chiller_ModeFb;  /* 返回 */
}
uint16_T get_BCU_usAirPumpState(void) {
    return Chiller_PumpStatus;  /* 返回 */
}
uint16_T get_BCU_usAirCompressorSta(void) {
    return Chiller_CompressorStatus;  /* 返回 */
}
uint16_T get_BCU_uiAirErrorfaultCode(void){
    return Chiller_Fault;  /* xxxxxxxxxxx 待定*/
}

uint16_T get_usBmuH2MaxConcentration(){
    return BCU_FasH2MaxValue;  /* xxxxxxxxxxx 待定*/
}
uint16_T get_usBmuCOMaxConcentration(){
    return BCU_FasCOMaxValue;  /* xxxxxxxxxxx 待定*/
}
uint16_T get_usBmuPressureMax(){
    return BCU_FasPressMaxValue;  /* xxxxxxxxxxx 待定*/
}
uint16_T get_usBmuLightMax(){
    return BCU_FasLightMaxValue;  /* xxxxxxxxxxx 待定*/
}
uint16_T get_usBmuH2MaxIndex(){
    return BCU_FasLightMaxValue;  /* xxxxxxxxxxx 待定*/
}
uint16_T get_usBmuCOMaxIndex(){
    return BCU_FasCOMaxIdx;  /* xxxxxxxxxxx 待定*/
}
uint16_T get_usBmuPressureMaxIndex(){
    return BCU_FasPressMaxIdx;  /* xxxxxxxxxxx 待定*/
}

uint16_T get_usBmuLightMaxIndex(){
    return BCU_FasLightMaxIdx;  /* xxxxxxxxxxx 待定*/
}

uint16_T get_usAirEnergyMode(){
    uint16_t value = 0;  // 分配实际内存
    get_modbus_reg_val(MDBUS_ENESAV_STA, &value);
    return value;  /* xxxxxxxxxxx 待定*/
}

uint16_T get_usAirInletPressure(){
    return Chiller_InletPressure;  /* xxxxxxxxxxx 待定*/
}

uint16_T get_usAirCoolSetTemp(){
    return ThermCtrl_ACWarmGoal;
}


uint16_T get_usAirOutWaterTemp(){
    return Chiller_TempOutlet;
}

uint16_T  get_usAirReturnWaterTemp(){
    return Chiller_TempInlet;
}


uint16_T get_usBatMaxVoltCellIndex(){
    return BCU_VoltMaxIdx;
}
uint16_T get_usBatMinVoltCellIndex(){
    return BCU_VoltMinIdx;
}
uint16_T  get_usBatMaxTempCellIndex(){
    return BCU_TempMaxIdx;
}
uint16_T  get_usBatMinTempCellIndex(){
    return BCU_TempMinIdx;
}

uint16_T get_usBatCellVoltMax(){
    return BCU_VoltMaxCellValue;
}
uint16_T get_usBatCellVoltMin(){
    return BCU_VoltMinCellValue;
}

uint16_T get_usBatCellTempMax(){
    return BCU_TempMaxValue;
}
uint16_T get_usBatCellTempMin(){
    return BCU_VoltMinCellValue;
}

// ==================== 辅助函数 ====================
// 初始化所有缓冲区
void init_can_buffer(void) {
    // 初始化BCU缓冲区
    memset(&bcu1_data_buffer, 0, sizeof(bcu1_data_buffer));
    bcu1_data_buffer.start = 0;
    bcu1_data_buffer.end = 0;
    bcu1_data_buffer.count = 0;
    bcu1_data_buffer.is_full = 0;
    
    // 初始化bcu2缓冲区
    memset(&bcu2_data_buffer, 0, sizeof(bcu2_data_buffer));
    bcu2_data_buffer.start = 0;
    bcu2_data_buffer.end = 0;
    bcu2_data_buffer.count = 0;
    bcu2_data_buffer.is_full = 0;
    
    memset(&post_change_buffer, 0, sizeof(post_change_buffer));
    memset(&cmd_track, 0, sizeof(cmd_track));
    
    memset(&tcu_send_buffer, 0, sizeof(tcu_send_buffer));
    tcu_send_buffer.start = 0;
    tcu_send_buffer.end = 0;
    tcu_send_buffer.count = 0;
    tcu_send_buffer.is_full = 0;
    
    change_recording_active = 0;
    post_tcu_collected = 0;
    post_bcu2_collected = 0;  // 新增
}

// 通用函数：向指定缓冲区添加帧
static void add_to_can_buffer_main(const CAN_FD_MESSAGE *msg, CanDataBuffer *buffer) {
    if (!msg || !buffer) return;
    memcpy(&buffer->frames[buffer->end], msg, sizeof(CAN_FD_MESSAGE));
    buffer->end = (buffer->end + 1) % DATA_BUFFER_SIZE;
    if (buffer->is_full) {
        buffer->start = (buffer->start + 1) % DATA_BUFFER_SIZE;
    } else {
        buffer->count++;
        if (buffer->count >= DATA_BUFFER_SIZE) {
            buffer->is_full = 1;
        }
    }
}

// 通用函数：获取指定缓冲区中最新的 N 帧（新 → 旧）
static void get_latest_frames(CanDataBuffer *buffer, CAN_FD_MESSAGE *output, int n) {
    if (!buffer || n <= 0 || n > DATA_BUFFER_SIZE) return;
    int count = (buffer->count < n) ? buffer->count : n;
    for (int i = 0; i < count; i++) {
        int idx = (buffer->start + buffer->count - 1 - i) % DATA_BUFFER_SIZE;
        if (idx < 0) idx += DATA_BUFFER_SIZE;
        memcpy(&output[i], &buffer->frames[idx], sizeof(CAN_FD_MESSAGE));
    }
}

// 打印 bcu2 帧
static void print_bcu2_frame(const CAN_FD_MESSAGE *frame, int seq) {
    char data_str[256] = {0};
    int offset = 0;
    int dlc = (frame->DLC > 64) ? 64 : frame->DLC;
    for (int j = 0; j < dlc; j++) {
        offset += snprintf(data_str + offset, sizeof(data_str) - offset,
                          "%02X%s", frame->Data[j], (j < dlc - 1) ? " " : "");
    }
    LOG("[RECORD] BCU2 Frame %d: ID=0x%08X, DLC=%d, Data=%s\r", seq, frame->ID, frame->DLC, data_str);
}

// 向 TCU 缓冲区添加帧（并支持变化后收集）
void record_tcu_send_frame(void)
{
    CAN_FD_MESSAGE msg = {0};
    msg.ID = 0x1801E410; // TCU 发送 ID（可自定义）
    msg.DLC = 64;
    memcpy(msg.Data, CANSendMsg.Data, 64); // 假设 CANSendMsg 是全局变量

    // 如果正在记录“变化后”的 TCU 帧
    if (change_recording_active && post_tcu_collected < DATA_BUFFER_SIZE) {
        memcpy(&post_tcu_frames[post_tcu_collected], &msg, sizeof(CAN_FD_MESSAGE));
        post_tcu_collected++;

        // 检查是否 BCU 和 TCU 都收满
        if (post_tcu_collected >= DATA_BUFFER_SIZE && 
            post_change_buffer.collected_count >= DATA_BUFFER_SIZE && 
            post_bcu2_collected >= DATA_BUFFER_SIZE) {  // 修改判断条件
            complete_change_recording();
        }
        // 注意：仍加入主 TCU 缓冲区（用于下次变化前记录）
    }

    // 正常加入 TCU 循环缓冲区
    memcpy(&tcu_send_buffer.frames[tcu_send_buffer.end], &msg, sizeof(CAN_FD_MESSAGE));
    tcu_send_buffer.end = (tcu_send_buffer.end + 1) % DATA_BUFFER_SIZE;
    if (tcu_send_buffer.is_full) {
        tcu_send_buffer.start = (tcu_send_buffer.start + 1) % DATA_BUFFER_SIZE;
    } else {
        tcu_send_buffer.count++;
        if (tcu_send_buffer.count >= DATA_BUFFER_SIZE) {
            tcu_send_buffer.is_full = 1;
        }
    }
}

// 获取 TCU 缓冲区中最新的 N 帧（新 → 旧）
static void get_latest_tcu_frames(CAN_FD_MESSAGE *output, int n) {
    if (n <= 0 || n > DATA_BUFFER_SIZE) return;
    int count = (tcu_send_buffer.count < n) ? tcu_send_buffer.count : n;
    for (int i = 0; i < count; i++) {
        int idx = (tcu_send_buffer.start + tcu_send_buffer.count - 1 - i) % DATA_BUFFER_SIZE;
        if (idx < 0) idx += DATA_BUFFER_SIZE;
        memcpy(&output[i], &tcu_send_buffer.frames[idx], sizeof(CAN_FD_MESSAGE));
    }
}

// 打印 BCU 帧
static void print_bcu_frame(const CAN_FD_MESSAGE *frame, int seq) {
    char data_str[256] = {0};
    int offset = 0;
    int dlc = (frame->DLC > 64) ? 64 : frame->DLC;
    for (int j = 0; j < dlc; j++) {
        offset += snprintf(data_str + offset, sizeof(data_str) - offset,
                          "%02X%s", frame->Data[j], (j < dlc - 1) ? " " : "");
    }
    LOG("[RECORD] BCU Frame %d: ID=0x%08X, DLC=%d, Data=%s\r", seq, frame->ID, frame->DLC, data_str);
}

// 打印 TCU 帧
static void print_tcu_frame(const CAN_FD_MESSAGE *frame, int seq) {
    char data_str[256] = {0};
    int offset = 0;
    int dlc = frame->DLC;
    if (dlc > 20) dlc = 20;  // ← 关键：限制最多只取前20字节

    for (int j = 0; j < dlc; j++) {
        offset += snprintf(data_str + offset, sizeof(data_str) - offset,
                          "%02X%s", frame->Data[j], (j < dlc - 1) ? " " : "");
    }
    LOG("[RECORD] TCU Frame %d: ID=0x%08X, DLC=%d, Data=%s\r", seq, frame->ID, frame->DLC, data_str);
}

// 完成变化记录并打印完整报告
static void complete_change_recording(void)
{
    LOG("[RECORD] ===== SIGNAL CHANGE DETECTED - FULL RECORDING =====\r");
    
    // 打印 BCU 变化后
    LOG("[RECORD] ----- BCU1 AFTER CHANGE (%d frames) -----\r", DATA_BUFFER_SIZE);
    for (int i = 0; i < DATA_BUFFER_SIZE; i++) {
        print_bcu_frame(&post_change_buffer.frames[i], i + 1);
    }
    
    // 打印 BCU2 变化后
    LOG("[RECORD] ----- BCU2 AFTER CHANGE (%d frames) -----\r", DATA_BUFFER_SIZE);
    for (int i = 0; i < DATA_BUFFER_SIZE; i++) {
        print_bcu2_frame(&post_bcu2_frames[i], i + 1);
    }
    
    // 打印 TCU 变化后
    LOG("[RECORD] ----- TCU AFTER CHANGE (%d frames) -----\r", DATA_BUFFER_SIZE);
    for (int i = 0; i < DATA_BUFFER_SIZE; i++) {
        print_tcu_frame(&post_tcu_frames[i], i + 1);
    }
    
    LOG("[RECORD] ===========================================\n");
    
    // 重置状态
    change_recording_active = 0;
    post_change_buffer.is_collecting = 0;
    post_change_buffer.collected_count = 0;
    post_tcu_collected = 0;
    post_bcu2_collected = 0;
}

// ==================== 主记录函数 ====================

void Log_Bcu_Data(const CAN_FD_MESSAGE *msg)
{
    static unsigned int BCU_FaultInfoLv1_LAST = 0;
    static unsigned int BCU_FaultInfoLv2_LAST = 0;
    static unsigned int BCU_FaultInfoLv3_LAST = 0;
    static unsigned int BCU_FaultInfoLv4_LAST = 0;
    static unsigned short BCU_SystemWorkMode_LAST = 0;

    static unsigned char OTA_XCPConnect_LAST = 0;
    static unsigned char TCU_ACMuteSet_LAST = 0;
    static unsigned int TCU_BCUEINSet_LAST = 0;
    static unsigned char TCU_ChargerWorkSts_LAST = 0;
    static unsigned char TCU_ECOMode_LAST = 0;
    static unsigned char TCU_FcnStopSet_LAST = 0;
    static unsigned char TCU_HighVoltType_LAST = 0;
    static unsigned short TCU_HighVoltValue_LAST = 0;
    static unsigned short TCU_PowerUpCmd_LAST = 0;
    if (!msg) {
        return;
    }

    if(msg->ID == 0x180110E4){
         add_to_can_buffer_main(msg, &bcu1_data_buffer);
        
        // 如果正在收集"变化后"BCU帧
        if (post_change_buffer.is_collecting) {
            if (post_change_buffer.collected_count < DATA_BUFFER_SIZE) {
                memcpy(&post_change_buffer.frames[post_change_buffer.collected_count],
                       msg, sizeof(CAN_FD_MESSAGE));
                post_change_buffer.collected_count++;
                
                if (post_change_buffer.collected_count >= DATA_BUFFER_SIZE && 
                    post_tcu_collected >= DATA_BUFFER_SIZE && 
                    post_bcu2_collected >= DATA_BUFFER_SIZE) {  // 修改判断条件
                    complete_change_recording();
                }
            }
            return;
        }

         // 提取当前值（需确保这些函数存在）
        unsigned short current_workmode = get_BCU_SystemWorkModeValue();
        unsigned int current_fault_lv1 = get_BCU_FaultInfoLv1Value();
        unsigned int current_fault_lv2 = get_BCU_FaultInfoLv2Value();
        unsigned int current_fault_lv3 = get_BCU_FaultInfoLv3Value();
        unsigned int current_fault_lv4 = get_BCU_FaultInfoLv4Value();

        unsigned char current_OTA_XCPConnect = get_OTA_XCPConnect();
        unsigned char current_TCU_ACMuteSet = get_TCU_ACMuteSet();
        unsigned int current_TCU_BCUEINSet = get_TCU_BCUEINSet();
        unsigned char current_TCU_ChargerWorkSts = get_TCU_ChargerWorkSts();
        unsigned char current_TCU_ECOMode = get_TCU_ECOMode();
        unsigned char current_TCU_FcnStopSet = get_TCU_FcnStopSet();
        unsigned char current_TCU_HighVoltType = get_TCU_HighVoltType();
        unsigned short current_TCU_HighVoltValue = get_TCU_HighVoltValue();
        unsigned char current_TCU_PowerUpCmd = get_TCU_PowerUpCmd();
        // printf("current_fault_lv1 = %d\n",current_fault_lv1);
        // printf("get_BCU_FaultInfoLv1Value() = %d\n",get_BCU_FaultInfoLv1Value());
        // 检查变化
        int changed = 0;
        if (BCU_SystemWorkMode_LAST != current_workmode) {changed = 1;LOG("BCU_SystemWorkMode Change from [0x%x] to [0x%x]\r",BCU_SystemWorkMode_LAST,current_workmode);}
        if (BCU_FaultInfoLv1_LAST != current_fault_lv1) {changed = 1;LOG("BCU_FaultInfoLv1 Change from [0x%x] to [0x%x]\r",BCU_FaultInfoLv1_LAST,current_fault_lv1);}
        if (BCU_FaultInfoLv2_LAST != current_fault_lv2) {changed = 1;LOG("BCU_FaultInfoLv2 Change from [0x%x] to [0x%x]\r",BCU_FaultInfoLv2_LAST,current_fault_lv2);}
        if (BCU_FaultInfoLv3_LAST != current_fault_lv3){ changed = 1;LOG("BCU_FaultInfoLv3 Change from [0x%x] to [0x%x]\r",BCU_FaultInfoLv3_LAST,current_fault_lv3);}
        if (BCU_FaultInfoLv4_LAST != current_fault_lv4) {changed = 1;LOG("BCU_FaultInfoLv4 Change from [0x%x] to [0x%x]\r",BCU_FaultInfoLv4_LAST,current_fault_lv4);}
        if (OTA_XCPConnect_LAST != current_OTA_XCPConnect) {changed = 1;LOG("OTA_XCPConnect Change from [0x%x] to [0x%x]\r",OTA_XCPConnect_LAST,current_OTA_XCPConnect);}
        if (TCU_ACMuteSet_LAST != current_TCU_ACMuteSet) {changed = 1;LOG("TCU_ACMuteSet Change from [0x%x] to [0x%x]\r",TCU_ACMuteSet_LAST,current_TCU_ACMuteSet);}
        if (TCU_BCUEINSet_LAST != current_TCU_BCUEINSet) {changed = 1;LOG("TCU_BCUEINSet Change from [0x%x] to [0x%x]\r",TCU_BCUEINSet_LAST,current_TCU_BCUEINSet);}
        if (TCU_ChargerWorkSts_LAST != current_TCU_ChargerWorkSts) {changed = 1;LOG("TCU_ChargerWorkSts Change from [0x%x] to [0x%x]\r",TCU_ChargerWorkSts_LAST,current_TCU_ChargerWorkSts);}
        if (TCU_ECOMode_LAST != current_TCU_ECOMode) {changed = 1;LOG("TCU_ECOMode Change from [0x%x] to [0x%x]\r",TCU_ECOMode_LAST,current_TCU_ECOMode);}
        if (TCU_FcnStopSet_LAST != current_TCU_FcnStopSet) {changed = 1;LOG("TCU_FcnStopSet Change from [0x%x] to [0x%x]\r",TCU_FcnStopSet_LAST,current_TCU_FcnStopSet);}
        if (TCU_HighVoltType_LAST != current_TCU_HighVoltType) {changed = 1;LOG("TCU_HighVoltType Change from [0x%x] to [0x%x]\r",TCU_HighVoltType_LAST,current_TCU_HighVoltType);}
        if (TCU_HighVoltValue_LAST != current_TCU_HighVoltValue) {changed = 1;LOG("TCU_HighVoltValue Change from [0x%x] to [0x%x]\r",TCU_HighVoltValue_LAST,current_TCU_HighVoltValue);}
        if (TCU_PowerUpCmd_LAST != current_TCU_PowerUpCmd){ changed = 1;LOG("TCU_PowerUpCmd Change from [0x%x] to [0x%x]\r",TCU_PowerUpCmd_LAST,current_TCU_PowerUpCmd);}
        // 触发记录
        if (changed && !change_recording_active) {
            change_recording_active = 1;
            post_change_buffer.is_collecting = 1;
            post_change_buffer.collected_count = 0;
            post_tcu_collected = 0;
            post_bcu2_collected = 0;  // 新增
            
            // --- BCU 变化前 ---
            int pre_bcu_count = (bcu1_data_buffer.count < DATA_BUFFER_SIZE) ? bcu1_data_buffer.count : DATA_BUFFER_SIZE;
            CAN_FD_MESSAGE pre_bcu[DATA_BUFFER_SIZE];
            get_latest_frames(&bcu1_data_buffer, pre_bcu, pre_bcu_count);
            LOG("[RECORD] ----- BCU1 BEFORE CHANGE (%d frames) -----\r", pre_bcu_count);
            for (int i = pre_bcu_count - 1; i >= 0; i--) {
                print_bcu_frame(&pre_bcu[i], pre_bcu_count - i);
            }
            
            // --- BCU2 变化前 ---
            int pre_bcu2_count = (bcu2_data_buffer.count < DATA_BUFFER_SIZE) ? bcu2_data_buffer.count : DATA_BUFFER_SIZE;
            CAN_FD_MESSAGE pre_bcu2[DATA_BUFFER_SIZE];
            get_latest_frames(&bcu2_data_buffer, pre_bcu2, pre_bcu2_count);
            LOG("[RECORD] ----- BCU2 BEFORE CHANGE (%d frames) -----\r", pre_bcu2_count);
            for (int i = pre_bcu2_count - 1; i >= 0; i--) {
                print_bcu2_frame(&pre_bcu2[i], pre_bcu2_count - i);
            }
            
            // --- TCU 变化前 ---
            int pre_tcu_count = (tcu_send_buffer.count < DATA_BUFFER_SIZE) ? tcu_send_buffer.count : DATA_BUFFER_SIZE;
            CAN_FD_MESSAGE pre_tcu[DATA_BUFFER_SIZE];
            get_latest_tcu_frames(pre_tcu, pre_tcu_count);
            LOG("[RECORD] ----- TCU BEFORE CHANGE (%d frames) -----\r", pre_tcu_count);
            for (int i = pre_tcu_count - 1; i >= 0; i--) {
                print_tcu_frame(&pre_tcu[i], pre_tcu_count - i);
            }
            
            // 当前 BCU 帧作为"变化后"第1帧
            memcpy(&post_change_buffer.frames[0], msg, sizeof(CAN_FD_MESSAGE));
            post_change_buffer.collected_count = 1;
        }
        // 更新 LAST 值
        BCU_SystemWorkMode_LAST = current_workmode;
        BCU_FaultInfoLv1_LAST = current_fault_lv1;
        BCU_FaultInfoLv2_LAST = current_fault_lv2;
        BCU_FaultInfoLv3_LAST = current_fault_lv3;
        BCU_FaultInfoLv4_LAST = current_fault_lv4;

        OTA_XCPConnect_LAST = current_OTA_XCPConnect;
        TCU_ACMuteSet_LAST = current_TCU_ACMuteSet;
        TCU_BCUEINSet_LAST = current_TCU_BCUEINSet;
        TCU_ChargerWorkSts_LAST = current_TCU_ChargerWorkSts;
        TCU_ECOMode_LAST = current_TCU_ECOMode;
        TCU_FcnStopSet_LAST = current_TCU_FcnStopSet;
        TCU_HighVoltType_LAST = current_TCU_HighVoltType;
        TCU_HighVoltValue_LAST = current_TCU_HighVoltValue;
        TCU_PowerUpCmd_LAST = current_TCU_PowerUpCmd;
    }
    else if (msg->ID == 0x180210E4) 
    {
        // BCU2数据
        add_to_can_buffer_main(msg, &bcu2_data_buffer);
        
        // 如果正在收集"变化后"bcu2帧
        if (change_recording_active && post_bcu2_collected < DATA_BUFFER_SIZE) {
            memcpy(&post_bcu2_frames[post_bcu2_collected], msg, sizeof(CAN_FD_MESSAGE));
            post_bcu2_collected++;
            
            // 检查是否所有缓冲区都收满
            if (post_change_buffer.collected_count >= DATA_BUFFER_SIZE && 
                post_tcu_collected >= DATA_BUFFER_SIZE && 
                post_bcu2_collected >= DATA_BUFFER_SIZE) {
                complete_change_recording();
            }
        }
    }else{     
        return; // 其他ID的消息，不处理
    }


}