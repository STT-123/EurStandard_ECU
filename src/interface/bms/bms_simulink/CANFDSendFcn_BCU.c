/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: CANFDSendFcn_BCU.c
 *
 * Code generated for Simulink model 'CANFDSendFcn_BCU'.
 *
 * Model version                  : 6.4
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Tue Aug 18 17:01:21 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CANFDSendFcn_BCU.h"
#include "rtwtypes.h"
#include "CANFDSendFcn_BCU_types.h"

/* Exported block signals */
uint8_T OTA_XCPConnect;                /* '<Root>/OTA_XCPConnect' */
uint8_T TCU_ACMuteSet;                 /* '<Root>/TCU_ACMuteSet' */
uint8_T TCU_ChargerWorkSts;            /* '<Root>/TCU_ChargerWorkSts' */
uint8_T TCU_ECOMode;                   /* '<Root>/TCU_ECOMode' */
uint8_T TCU_FcnStopSet;                /* '<Root>/TCU_FcnStopSet' */
uint8_T TCU_HighVoltType;              /* '<Root>/TCU_HighVoltType' */
uint16_T TCU_HighVoltValue;            /* '<Root>/TCU_HighVoltValue' */
uint8_T TCU_LifeCounter;               /* '<Root>/TCU_LifeCounter' */
uint8_T TCU_PowerUpCmd;                /* '<Root>/TCU_PowerUpCmd' */
uint8_T TCU_TimeCalFlg;                /* '<Root>/TCU_TimeCalFlg' */
uint8_T TCU_TimeDay;                   /* '<Root>/TCU_TimeDay' */
uint8_T TCU_TimeHour;                  /* '<Root>/TCU_TimeHour' */
uint8_T TCU_TimeMinute;                /* '<Root>/TCU_TimeMinute' */
uint8_T TCU_TimeMonth;                 /* '<Root>/TCU_TimeMonth' */
uint8_T TCU_TimeSecond;                /* '<Root>/TCU_TimeSecond' */
uint8_T TCU_TimeWeek;                  /* '<Root>/TCU_TimeWeek' */
uint8_T TCU_TimeYear;                  /* '<Root>/TCU_TimeYear' */
uint8_T TCU_PHYError;                  /* '<Root>/TCU_PHYError' */
CAN_FD_MESSAGE_BUS CANSendMsg;         /* '<Root>/CANSendMsg' */

/* Block signals (default storage) */
B_CANFDSendFcn_BCU_T CANFDSendFcn_BCU_B;

/* Block states (default storage) */
DW_CANFDSendFcn_BCU_T CANFDSendFcn_BCU_DW;

/* Real-time model */
static RT_MODEL_CANFDSendFcn_BCU_T CANFDSendFcn_BCU_M_;
RT_MODEL_CANFDSendFcn_BCU_T *const CANFDSendFcn_BCU_M = &CANFDSendFcn_BCU_M_;

/* Model step function */
void CANFDSendFcn_BCU_step(void)
{
  /* Gain: '<Root>/Gain' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   *  Inport: '<Root>/TCU_HighVoltValue'
   */
  CANFDSendFcn_BCU_B.Gain = 0.1F * (real32_T)TCU_HighVoltValue;

  /* S-Function (scanfdpack): '<Root>/CAN FD Pack' incorporates:
   *  Inport: '<Root>/OTA_XCPConnect'
   *  Inport: '<Root>/TCU_ACMuteSet'
   *  Inport: '<Root>/TCU_ChargerWorkSts'
   *  Inport: '<Root>/TCU_ECOMode'
   *  Inport: '<Root>/TCU_FcnStopSet'
   *  Inport: '<Root>/TCU_HighVoltType'
   *  Inport: '<Root>/TCU_LifeCounter'
   *  Inport: '<Root>/TCU_PHYError'
   *  Inport: '<Root>/TCU_PowerUpCmd'
   *  Inport: '<Root>/TCU_TimeCalFlg'
   *  Inport: '<Root>/TCU_TimeDay'
   *  Inport: '<Root>/TCU_TimeHour'
   *  Inport: '<Root>/TCU_TimeMinute'
   *  Inport: '<Root>/TCU_TimeMonth'
   *  Inport: '<Root>/TCU_TimeSecond'
   *  Inport: '<Root>/TCU_TimeWeek'
   *  Inport: '<Root>/TCU_TimeYear'
   *  Outport: '<Root>/CANSendMsg'
   */
  /* S-Function (scanfdpack): '<Root>/CAN FD Pack' */
  CANSendMsg.ID = 0x1801E410;
  CANSendMsg.Length = 64U;
  CANSendMsg.Extended = 1U;
  CANSendMsg.Remote = 0;
  CANSendMsg.BRS = 1;
  CANSendMsg.ProtocolMode = 1;
  CANSendMsg.DLC = 15U;
  CANSendMsg.Data[0] = 0;
  CANSendMsg.Data[1] = 0;
  CANSendMsg.Data[2] = 0;
  CANSendMsg.Data[3] = 0;
  CANSendMsg.Data[4] = 0;
  CANSendMsg.Data[5] = 0;
  CANSendMsg.Data[6] = 0;
  CANSendMsg.Data[7] = 0;
  CANSendMsg.Data[8] = 0;
  CANSendMsg.Data[9] = 0;
  CANSendMsg.Data[10] = 0;
  CANSendMsg.Data[11] = 0;
  CANSendMsg.Data[12] = 0;
  CANSendMsg.Data[13] = 0;
  CANSendMsg.Data[14] = 0;
  CANSendMsg.Data[15] = 0;
  CANSendMsg.Data[16] = 0;
  CANSendMsg.Data[17] = 0;
  CANSendMsg.Data[18] = 0;
  CANSendMsg.Data[19] = 0;
  CANSendMsg.Data[20] = 0;
  CANSendMsg.Data[21] = 0;
  CANSendMsg.Data[22] = 0;
  CANSendMsg.Data[23] = 0;
  CANSendMsg.Data[24] = 0;
  CANSendMsg.Data[25] = 0;
  CANSendMsg.Data[26] = 0;
  CANSendMsg.Data[27] = 0;
  CANSendMsg.Data[28] = 0;
  CANSendMsg.Data[29] = 0;
  CANSendMsg.Data[30] = 0;
  CANSendMsg.Data[31] = 0;
  CANSendMsg.Data[32] = 0;
  CANSendMsg.Data[33] = 0;
  CANSendMsg.Data[34] = 0;
  CANSendMsg.Data[35] = 0;
  CANSendMsg.Data[36] = 0;
  CANSendMsg.Data[37] = 0;
  CANSendMsg.Data[38] = 0;
  CANSendMsg.Data[39] = 0;
  CANSendMsg.Data[40] = 0;
  CANSendMsg.Data[41] = 0;
  CANSendMsg.Data[42] = 0;
  CANSendMsg.Data[43] = 0;
  CANSendMsg.Data[44] = 0;
  CANSendMsg.Data[45] = 0;
  CANSendMsg.Data[46] = 0;
  CANSendMsg.Data[47] = 0;
  CANSendMsg.Data[48] = 0;
  CANSendMsg.Data[49] = 0;
  CANSendMsg.Data[50] = 0;
  CANSendMsg.Data[51] = 0;
  CANSendMsg.Data[52] = 0;
  CANSendMsg.Data[53] = 0;
  CANSendMsg.Data[54] = 0;
  CANSendMsg.Data[55] = 0;
  CANSendMsg.Data[56] = 0;
  CANSendMsg.Data[57] = 0;
  CANSendMsg.Data[58] = 0;
  CANSendMsg.Data[59] = 0;
  CANSendMsg.Data[60] = 0;
  CANSendMsg.Data[61] = 0;
  CANSendMsg.Data[62] = 0;
  CANSendMsg.Data[63] = 0;

  {
    /* --------------- START Packing signal 0 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (OTA_XCPConnect);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[5] = CANSendMsg.Data[5] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 1 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_ACMuteSet);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (uint8_T)(1)) {
          packedValue = (uint8_T) 1;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            CANSendMsg.Data[2] = CANSendMsg.Data[2] | (uint8_T)((uint8_T)
              (packedValue & (uint8_T)0x1U));
          }
        }
      }
    }

    /* --------------- START Packing signal 2 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_ChargerWorkSts);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[3] = CANSendMsg.Data[3] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 3 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_ECOMode);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (uint8_T)(1)) {
          packedValue = (uint8_T) 1;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            CANSendMsg.Data[2] = CANSendMsg.Data[2] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x1U) << 2));
          }
        }
      }
    }

    /* --------------- START Packing signal 4 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_FcnStopSet);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[4] = CANSendMsg.Data[4] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 5 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_HighVoltType);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (uint8_T)(3)) {
          packedValue = (uint8_T) 3;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            CANSendMsg.Data[2] = CANSendMsg.Data[2] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0x3U) << 3));
          }
        }
      }
    }

    /* --------------- START Packing signal 6 ------------------*/
    {
      real32_T outValue = 0;

      {
        real32_T result = CANFDSendFcn_BCU_B.Gain;

        /* no offset to apply */
        result = result * (1 / 0.1F);

        /* round to closest integer value for integer CAN signal */
        outValue = roundf(result);
      }

      {
        uint16_T packedValue;
        if (outValue > (real32_T)(65535)) {
          packedValue = (uint16_T) 65535;
        } else if (outValue < (real32_T)(0)) {
          packedValue = (uint16_T) 0;
        } else {
          packedValue = (uint16_T) (outValue);
        }

        {
          {
            CANSendMsg.Data[19] = CANSendMsg.Data[19] | (uint8_T)((uint16_T)
              (packedValue & (uint16_T)0xFFU));
            CANSendMsg.Data[18] = CANSendMsg.Data[18] | (uint8_T)((uint16_T)
              ((uint16_T)(packedValue & (uint16_T)0xFF00U) >> 8));
          }
        }
      }
    }

    /* --------------- START Packing signal 7 ------------------*/
    {
      static uint8_T lifeCounter = 0;  // 静态变量，自动递增
      real_T outValue = 0;

      // 模拟 TCU_LifeCounter 自增
      lifeCounter++;
      if (lifeCounter >= 255) {
        lifeCounter = 0;
      }

      TCU_LifeCounter = (real_T)lifeCounter;

      // printf("TCU_LifeCounter = %.0f\n", TCU_LifeCounter);
      outValue = round(TCU_LifeCounter);

      uint8_T packedValue;
      if (outValue > 255.0) {
        packedValue = 255;
      } else if (outValue < 0.0) {
        packedValue = 0;
      } else {
        packedValue = (uint8_T)outValue;
      }
      CANSendMsg.Data[0] = CANSendMsg.Data[0] | (uint8_T)(packedValue);
    }

    /* --------------- START Packing signal 8 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_PHYError);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        if (packingValue > (uint8_T)(15)) {
          packedValue = (uint8_T) 15;
        } else {
          packedValue = (uint8_T) (packingValue);
        }

        {
          {
            CANSendMsg.Data[21] = CANSendMsg.Data[21] | (uint8_T)((uint8_T)
              ((uint8_T)(packedValue & (uint8_T)0xFU) << 4));
          }
        }
      }
    }

    /* --------------- START Packing signal 9 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_PowerUpCmd);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[1] = CANSendMsg.Data[1] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 10 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_TimeCalFlg);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[17] = CANSendMsg.Data[17] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 11 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_TimeDay);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[10] = CANSendMsg.Data[10] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 12 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_TimeHour);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[11] = CANSendMsg.Data[11] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 13 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_TimeMinute);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[12] = CANSendMsg.Data[12] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 14 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_TimeMonth);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[13] = CANSendMsg.Data[13] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 15 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_TimeSecond);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[14] = CANSendMsg.Data[14] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 16 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_TimeWeek);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[15] = CANSendMsg.Data[15] | (uint8_T)(packedValue);
          }
        }
      }
    }

    /* --------------- START Packing signal 17 ------------------*/
    {
      uint32_T packingValue = 0;

      {
        uint32_T result = (uint32_T) (TCU_TimeYear);

        /* no scaling required */
        packingValue = result;
      }

      {
        uint8_T packedValue;
        packedValue = (uint8_T) (packingValue);

        {
          {
            CANSendMsg.Data[16] = CANSendMsg.Data[16] | (uint8_T)(packedValue);
          }
        }
      }
    }
  }
  
  CANSendMsg.Extended =1;
  CANSendMsg.Remote =0;
  CANSendMsg.Error =0;
  CANSendMsg.ESI  =0;
  CANSendMsg.BRS  =0;
  int ret = Drv_bcu_canfd_send(&CANSendMsg);
}

/* Model initialize function */
void CANFDSendFcn_BCU_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void CANFDSendFcn_BCU_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
