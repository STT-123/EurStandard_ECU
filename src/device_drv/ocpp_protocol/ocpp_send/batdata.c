#include "batdata.h"


void convert_tBatData_to_big_endian(tBatData *dst, const tBatData *src) {
    // dst->uiTimeStamp = htonl(src->uiTimeStamp);
    dst->uiTimeStamp = src->uiTimeStamp;
    for (int i = 0; i < 240; ++i)
        dst->usSingleBatVal[i] = htons(src->usSingleBatVal[i]);

    for (int i = 0; i < 120; ++i)
        dst->usSingleBatTemp[i] = htons(src->usSingleBatTemp[i]);

    for (int i = 0; i < 15; ++i) {
        dst->uiBmuErrorNum[i] = htonl(src->uiBmuErrorNum[i]);
        dst->uiBmuExErrorNum[i] = htonl(src->uiBmuExErrorNum[i]);
    }

    dst->iDcPower = htonl(src->iDcPower);

    // 手动拆分 64 位整数（unsigned long long）为大端存储
    dst->ullPosEleQuantity = ((uint64_t)htonl(src->ullPosEleQuantity >> 32)) |
                             ((uint64_t)htonl(src->ullPosEleQuantity & 0xFFFFFFFF) << 32);
    dst->ullNegEleQuantity = ((uint64_t)htonl(src->ullNegEleQuantity >> 32)) |
                             ((uint64_t)htonl(src->ullNegEleQuantity & 0xFFFFFFFF) << 32);

    dst->usAirState = htons(src->usAirState);
    dst->usAirPumpState = htons(src->usAirPumpState);
    dst->usAirCompressorSta = htons(src->usAirCompressorSta);
    dst->uiAirErrorLv1 = htonl(src->uiAirErrorLv1);
    dst->uiAirErrorLv2 = htonl(src->uiAirErrorLv2);
    dst->uiAirErrorLv3 = htonl(src->uiAirErrorLv3);
    dst->usTempInside = htons(src->usTempInside);
    dst->usTempOutside = htons(src->usTempOutside);
    dst->usHumidityInside = htons(src->usHumidityInside);

    dst->usBmuH2MaxValue = htons(src->usBmuH2MaxValue);
    dst->usBmuH2MinValue = htons(src->usBmuH2MinValue);
    dst->usBmuCOMaxValue = htons(src->usBmuCOMaxValue);
    dst->usBmuCOMinValue = htons(src->usBmuCOMinValue);
    dst->usBmuPressureMaxValue = htons(src->usBmuPressureMaxValue);
    dst->usBmuPressureMinValue = htons(src->usBmuPressureMinValue);
    dst->usBmuLightMaxValue = htons(src->usBmuLightMaxValue);
    dst->usBmuLightMinValue = htons(src->usBmuLightMinValue);
    dst->usBmuH2MaxIndex = htons(src->usBmuH2MaxIndex);
    dst->usBmuH2MinIndex = htons(src->usBmuH2MinIndex);
    dst->usBmuCOMaxIndex = htons(src->usBmuCOMaxIndex);
    dst->usBmuCOMinIndex = htons(src->usBmuCOMinIndex);
    dst->usBmuPressureMaxIndex = htons(src->usBmuPressureMaxIndex);
    dst->usBmuPressureMinIndex = htons(src->usBmuPressureMinIndex);
    dst->usBmuLightMaxIndex = htons(src->usBmuLightMaxIndex);
    dst->usBmuLightMinIndex = htons(src->usBmuLightMinIndex);

    dst->usAirEnergyMode = htons(src->usAirEnergyMode);
    dst->usAirInletPressure = htons(src->usAirInletPressure);
    dst->usAirCoolSetTemp = htons(src->usAirCoolSetTemp);
    dst->usAirHeatSetTemp = htons(src->usAirHeatSetTemp);
    dst->usAirOutWaterTemp = htons(src->usAirOutWaterTemp);
    dst->usAirReturnWaterTemp = htons(src->usAirReturnWaterTemp);

    dst->usBatMaxVoltCellIndex = htons(src->usBatMaxVoltCellIndex);
    dst->usBatMinVoltCellIndex = htons(src->usBatMinVoltCellIndex);
    dst->usBatMaxTempCellIndex = htons(src->usBatMaxTempCellIndex);
    dst->usBatMinTempCellIndex = htons(src->usBatMinTempCellIndex);
    dst->usBatCellVoltMaxValue = htons(src->usBatCellVoltMaxValue);
    dst->usBatCellVoltMinValue = htons(src->usBatCellVoltMinValue);
    dst->usBatMaxTempCellVolt = htons(src->usBatMaxTempCellVolt);
    dst->usBatMinTempCellVolt = htons(src->usBatMinTempCellVolt);
    dst->usBatCellTempMaxValue = htons(src->usBatCellTempMaxValue);
    dst->usBatCellTempMinValue = htons(src->usBatCellTempMinValue);
    dst->usBatMaxVoltCellTemp = htons(src->usBatMaxVoltCellTemp);
    dst->usBatMinVoltCellTemp = htons(src->usBatMinVoltCellTemp);
}


tBatData BatData;