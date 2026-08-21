/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: CANFDRcvFcn_BCU.c
 *
 * Code generated for Simulink model 'CANFDRcvFcn_BCU'.
 *
 * Model version                  : 6.9
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Tue Aug 18 16:53:38 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CANFDRcvFcn_BCU.h"
#include "rtwtypes.h"
#include "CANFDRcvFcn_BCU_private.h"
#include <math.h>
#include <string.h>
#include "CANFDRcvFcn_BCU_types.h"
#include <float.h>

/* Exported block signals */
CAN_FD_MESSAGE_BUS CANFDRcvMsg;        /* '<Root>/CANFDRcvMsg' */

/* Block signals (default storage) */
B_CANFDRcvFcn_BCU_T CANFDRcvFcn_BCU_B;

/* Block states (default storage) */
DW_CANFDRcvFcn_BCU_T CANFDRcvFcn_BCU_DW;

/* Real-time model */
static RT_MODEL_CANFDRcvFcn_BCU_T CANFDRcvFcn_BCU_M_;
RT_MODEL_CANFDRcvFcn_BCU_T *const CANFDRcvFcn_BCU_M = &CANFDRcvFcn_BCU_M_;

/* Exported data definition */

/* Definition for custom storage class: Default */
uint16_T BCU_BCUVersion;               /* '<S1>/C Function14' */
uint8_T BCU_BCUVersion_H;              /* '<S1>/CAN FD Unpack2' */
uint8_T BCU_BCUVersion_L;              /* '<S1>/CAN FD Unpack2' */
uint16_T BCU_BMUConnErrNum;            /* '<S1>/CAN FD Unpack' */
uint16_T BCU_BatteryStatus;            /* '<S1>/CAN FD Unpack1' */
uint16_T BCU_Curr;                     /* '<S1>/Data Type Conversion46' */
uint32_T BCU_EngryAccumulateChrg;      /* '<S1>/Data Type Conversion18' */
uint32_T BCU_EngryAccumulateDisChrg;   /* '<S1>/Data Type Conversion20' */
uint16_T BCU_FasCOMaxIdx;              /* '<S1>/CAN FD Unpack' */
uint16_T BCU_FasCOMaxValue;            /* '<S1>/CAN FD Unpack' */
uint16_T BCU_FasH2MaxIdx;              /* '<S1>/CAN FD Unpack' */
uint16_T BCU_FasH2MaxValue;            /* '<S1>/CAN FD Unpack' */
uint16_T BCU_FasLightMaxIdx;           /* '<S1>/CAN FD Unpack' */
uint16_T BCU_FasLightMaxValue;         /* '<S1>/CAN FD Unpack' */
uint16_T BCU_FasPressMaxIdx;           /* '<S1>/CAN FD Unpack' */
uint16_T BCU_FasPressMaxValue;         /* '<S1>/CAN FD Unpack' */
uint32_T BCU_FaultInfoLv1;             /* '<S1>/CAN FD Unpack1' */
uint32_T BCU_FaultInfoLv2;             /* '<S1>/CAN FD Unpack1' */
uint32_T BCU_FaultInfoLv3;             /* '<S1>/CAN FD Unpack1' */
uint32_T BCU_FaultInfoLv4;             /* '<S1>/CAN FD Unpack1' */
uint32_T BCU_RealtimePower;            /* '<S1>/Data Type Conversion6' */
uint16_T BCU_SNCode[32];               /* '<S1>/Reshape' */
uint16_T BCU_SOC;                      /* '<S1>/Data Type Conversion35' */
uint16_T BCU_SystemWorkMode;           /* '<S1>/CAN FD Unpack1' */
uint16_T BCU_TempMaxIdx;               /* '<S8>/Data Type Conversion44' */
uint16_T BCU_TempMaxValue;             /* '<S1>/Data Type Conversion58' */
uint16_T BCU_TempMinIdx;               /* '<S9>/Data Type Conversion45' */
uint16_T BCU_TempMinValue;             /* '<S1>/Data Type Conversion59' */
uint16_T BCU_TemperatureInBox;         /* '<S1>/Data Type Conversion14' */
uint16_T BCU_TemperatureOutBox;        /* '<S1>/Data Type Conversion38' */
uint8_T BCU_TimeDay;                   /* '<S1>/CAN FD Unpack' */
uint8_T BCU_TimeHour;                  /* '<S1>/CAN FD Unpack' */
uint8_T BCU_TimeMinute;                /* '<S1>/CAN FD Unpack' */
uint8_T BCU_TimeMonth;                 /* '<S1>/CAN FD Unpack' */
uint8_T BCU_TimeSencond;               /* '<S1>/CAN FD Unpack' */
uint8_T BCU_TimeWeek;                  /* '<S1>/CAN FD Unpack' */
uint8_T BCU_TimeYear;                  /* '<S1>/CAN FD Unpack' */
uint16_T BCU_VoltMaxCellValue;         /* '<S1>/CAN FD Unpack2' */
uint16_T BCU_VoltMaxIdx;               /* '<S11>/Data Type Conversion41' */
uint16_T BCU_VoltMinCellValue;         /* '<S1>/CAN FD Unpack2' */
uint16_T BCU_VoltMinIdx;               /* '<S12>/Data Type Conversion42' */
uint16_T Chiller_CompressorStatus;     /* '<S1>/Data Type Conversion42' */
uint16_T Chiller_Fault;                /* '<S1>/CAN FD Unpack3' */
uint16_T Chiller_InletPressure;        /* '<S1>/Data Type Conversion34' */
uint16_T Chiller_ModeFb;               /* '<S1>/CAN FD Unpack3' */
uint16_T Chiller_PumpStatus;           /* '<S1>/CAN FD Unpack7' */
uint16_T Chiller_TempInlet;            /* '<S1>/Data Type Conversion45' */
uint16_T Chiller_TempOutlet;           /* '<S1>/Data Type Conversion60' */
uint16_T ThermCtrl_ACWarmGoal;         /* '<S1>/Data Type Conversion11' */
uint16_T ThermCtrl_Fault;              /* '<S5>/C Function11' */
uint16_T usSingleBatTemp[120];         /* '<S3>/MATLAB Function' */
uint16_T usSingleBatVal[240];          /* '<S4>/MATLAB Function2' */

/*
 * Output and update for atomic system:
 *    '<S1>/SocProcess'
 *    '<S1>/SocProcess1'
 */
void CANFDRcvFcn_BCU_SocProcess(uint16_T rtu_soc, uint16_T *rty_soc_process)
{
  if ((rtu_soc > 0) && (rtu_soc < 100)) {
    *rty_soc_process = 100U;
  } else if ((rtu_soc > 9900) && (rtu_soc < 10000)) {
    *rty_soc_process = 9900U;
  } else {
    *rty_soc_process = rtu_soc;
  }
}

real_T rt_modd(real_T u0, real_T u1)
{
  real_T y;
  y = u0;
  if (u1 == 0.0) {
    if (u0 == 0.0) {
      y = u1;
    }
  } else if (u0 == 0.0) {
    y = 0.0 / u1;
  } else {
    boolean_T yEq;
    y = fmod(u0, u1);
    yEq = (y == 0.0);
    if (!yEq && (u1 > floor(u1))) {
      real_T q;
      q = fabs(u0 / u1);
      yEq = (fabs(q - floor(q + 0.5)) <= DBL_EPSILON * q);
    }

    if (yEq) {
      y = 0.0;
    } else if ((u0 < 0.0) != (u1 < 0.0)) {
      y += u1;
    }
  }

  return y;
}

/* Model step function */
void CANFDRcvFcn_BCU_step(void)
{
  real_T tmp_0;
  int32_T i;
  real32_T tmp;
  real32_T tmp_1;
  real32_T u;
  real32_T v;
  uint32_T port_index;
  uint32_T port_len;
  uint32_T q0;
  uint32_T qY;
  uint32_T rtb_DataTypeConversion40;
  uint16_T rtb_TmpSignalConversionAtSFunct[513];
  uint16_T rtb_TmpSignalConversionAtSFun_a[60];
  uint16_T rtb_TmpSignalConversionAtSFu_kh[30];
  uint16_T CFunction1_o1;
  uint16_T CFunction1_o2;
  uint16_T CFunction_o1;
  uint16_T CFunction_o2;
  uint16_T b;
  uint16_T c;
  uint16_T rtb_DataTypeConversion29;
  uint16_T rtb_ShiftArithmetic3;
  uint8_T b_0;
  uint8_T c_0;
  uint8_T rtb_Saturation2;

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack1' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x180110E4== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[44]) & (uint8_T)(0x30U)) >> 4);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[44]) & (uint8_T)(0xC0U)) >> 6);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o2 = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[44]) & (uint8_T)(0xCU)) >> 2);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o3 = result;
            }
          }

          /* --------------- START Unpacking signal 3 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_SystemWorkMode = result;
            }
          }

          /* --------------- START Unpacking signal 4 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[62]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_BatteryStatus = result;
            }
          }

          /* --------------- START Unpacking signal 5 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[11]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[10]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = (result * 0.1F) + -3200.0F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o6 = result;
            }
          }

          /* --------------- START Unpacking signal 6 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[13]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[12]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = (result * 0.1F) + -3200.0F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o7 = result;
            }
          }

          /* --------------- START Unpacking signal 7 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[2]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[1]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o8 = result;
            }
          }

          /* --------------- START Unpacking signal 8 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[4]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[3]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o9 = result;
            }
          }

          /* --------------- START Unpacking signal 9 ------------------*/
          {
            uint32_T outValue = 0;

            {
              uint32_T unpackedValue = 0;

              {
                uint32_T tempValue = (uint32_T) (0);

                {
                  tempValue = tempValue | (uint32_T)(CANFDRcvMsg.Data[31]);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[30]) << 8);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[29]) << 16);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[28]) << 24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              BCU_FaultInfoLv1 = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------*/
          {
            uint32_T outValue = 0;

            {
              uint32_T unpackedValue = 0;

              {
                uint32_T tempValue = (uint32_T) (0);

                {
                  tempValue = tempValue | (uint32_T)(CANFDRcvMsg.Data[35]);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[34]) << 8);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[33]) << 16);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[32]) << 24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              BCU_FaultInfoLv2 = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------*/
          {
            uint32_T outValue = 0;

            {
              uint32_T unpackedValue = 0;

              {
                uint32_T tempValue = (uint32_T) (0);

                {
                  tempValue = tempValue | (uint32_T)(CANFDRcvMsg.Data[39]);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[38]) << 8);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[37]) << 16);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[36]) << 24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              BCU_FaultInfoLv3 = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------*/
          {
            uint32_T outValue = 0;

            {
              uint32_T unpackedValue = 0;

              {
                uint32_T tempValue = (uint32_T) (0);

                {
                  tempValue = tempValue | (uint32_T)(CANFDRcvMsg.Data[43]);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[42]) << 8);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[41]) << 16);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[40]) << 24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              BCU_FaultInfoLv4 = result;
            }
          }

          /* --------------- START Unpacking signal 13 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[47]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[46]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o14 = result;
            }
          }

          /* --------------- START Unpacking signal 14 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[49]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[48]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o15 = result;
            }
          }

          /* --------------- START Unpacking signal 15 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[51]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[50]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o16 = result;
            }
          }

          /* --------------- START Unpacking signal 16 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[63]) & (uint8_T)(0xCU)) >> 2);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o17 = result;
            }
          }

          /* --------------- START Unpacking signal 17 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)(CANFDRcvMsg.Data
                    [63]) & (uint8_T)(0x3U));
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o18 = result;
            }
          }

          /* --------------- START Unpacking signal 18 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[63]) & (uint8_T)(0x30U)) >> 4);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o19 = result;
            }
          }

          /* --------------- START Unpacking signal 19 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint32_T unpackedValue = 0;

              {
                uint32_T tempValue = (uint32_T) (0);

                {
                  tempValue = tempValue | (uint32_T)(CANFDRcvMsg.Data[9]);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[8]) << 8);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[7]) << 16);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result + -8.388608E+6F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o20 = result;
            }
          }

          /* --------------- START Unpacking signal 20 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[21]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[20]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o21 = result;
            }
          }

          /* --------------- START Unpacking signal 21 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[23]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[22]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o22 = result;
            }
          }

          /* --------------- START Unpacking signal 22 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[27]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[26]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o23 = result;
            }
          }

          /* --------------- START Unpacking signal 23 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[25]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[24]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o24 = result;
            }
          }

          /* --------------- START Unpacking signal 24 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[63]) & (uint8_T)(0x80U)) >> 7);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o25 = result;
            }
          }

          /* --------------- START Unpacking signal 25 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[63]) & (uint8_T)(0x40U)) >> 6);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o26 = result;
            }
          }

          /* --------------- START Unpacking signal 26 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[15]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[14]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o27 = result;
            }
          }

          /* --------------- START Unpacking signal 27 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[17]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[16]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o28 = result;
            }
          }

          /* --------------- START Unpacking signal 28 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[19]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[18]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o29 = result;
            }
          }

          /* --------------- START Unpacking signal 29 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[6]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[5]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o30 = result;
            }
          }

          /* --------------- START Unpacking signal 30 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[56]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o31 = result;
            }
          }

          /* --------------- START Unpacking signal 31 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[57]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o32 = result;
            }
          }

          /* --------------- START Unpacking signal 32 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[58]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o33 = result;
            }
          }

          /* --------------- START Unpacking signal 33 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[59]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o34 = result;
            }
          }
        }
      }
    }
  }

  tmp_1 = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o6), 65536.0F);
  BCU_Curr = (uint16_T)(tmp_1 < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)
                        -tmp_1 : (int32_T)(uint16_T)tmp_1);
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_value =
    CANFDRcvFcn_BCU_B.CANFDUnpack1_o7; // ����ת����

  // ����1��ʹ�� memcpy������ָ��������⣬�Ƽ���
  memcpy(&CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits,
         &CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_value, sizeof(float));

  // ����2��ֱ��ָ��ת�����������ϸ�������⣬���Ƽ���
  // float_bits = *(uint32_t *)&float_value;

  // ��ȡ�ֽڣ������ DCBA��
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteD =
    (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits >> 24) & 0xFF;// �����Ч�ֽ�
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteC =
    (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits >> 16) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteB =
    (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits >> 8) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteA =
    CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits & 0xFF;// �����Ч�ֽ�

  // ��ϳ����� 16 λ�Ĵ���
  CANFDRcvFcn_BCU_B.BCU_Curr2_H = (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteD <<
    8) | CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteC;// DC �Ĵ���
  CANFDRcvFcn_BCU_B.BCU_Curr2_L = (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteB <<
    8) | CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteA;// BA �Ĵ���

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack2' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x180210E4== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_BCUVersion_H = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_BCUVersion_L = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[2]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o3 = result;
            }
          }

          /* --------------- START Unpacking signal 3 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[4]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[3]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o4 = result;
            }
          }

          /* --------------- START Unpacking signal 4 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[5]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o5 = result;
            }
          }

          /* --------------- START Unpacking signal 5 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[6]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o6 = result;
            }
          }

          /* --------------- START Unpacking signal 6 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[7]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o7 = result;
            }
          }

          /* --------------- START Unpacking signal 7 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[8]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o8 = result;
            }
          }

          /* --------------- START Unpacking signal 8 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[12]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[11]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o9 = result;
            }
          }

          /* --------------- START Unpacking signal 9 ------------------*/
          {
            real_T outValue = 0;

            {
              uint32_T unpackedValue = 0;

              {
                uint32_T tempValue = (uint32_T) (0);

                {
                  tempValue = tempValue | (uint32_T)(CANFDRcvMsg.Data[16]);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[15]) << 8);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[14]) << 16);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[13]) << 24);
                }

                unpackedValue = tempValue;
              }

              outValue = (real_T) (unpackedValue);
            }

            {
              real_T result = (real_T) outValue;
              result = result * 0.1;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o10 = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[18]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[17]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o11 = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[20]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[19]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o12 = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------*/
          {
            real_T outValue = 0;

            {
              uint32_T unpackedValue = 0;

              {
                uint32_T tempValue = (uint32_T) (0);

                {
                  tempValue = tempValue | (uint32_T)(CANFDRcvMsg.Data[24]);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[23]) << 8);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[22]) << 16);
                  tempValue = tempValue | (uint32_T)((uint32_T)
                    (CANFDRcvMsg.Data[21]) << 24);
                }

                unpackedValue = tempValue;
              }

              outValue = (real_T) (unpackedValue);
            }

            {
              real_T result = (real_T) outValue;
              result = result * 0.1;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o13 = result;
            }
          }

          /* --------------- START Unpacking signal 13 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[26]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[25]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o14 = result;
            }
          }

          /* --------------- START Unpacking signal 14 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[28]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[27]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o15 = result;
            }
          }

          /* --------------- START Unpacking signal 15 ------------------*/
          /*
           * Signal is not connected or connected to terminator.
           * No unpacking code generated.
           */

          /* --------------- START Unpacking signal 16 ------------------*/
          /*
           * Signal is not connected or connected to terminator.
           * No unpacking code generated.
           */

          /* --------------- START Unpacking signal 17 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[61]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o18 = result;
            }
          }

          /* --------------- START Unpacking signal 18 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[33]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o19 = result;
            }
          }

          /* --------------- START Unpacking signal 19 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[34]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o20 = result;
            }
          }

          /* --------------- START Unpacking signal 20 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[35]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o21 = result;
            }
          }

          /* --------------- START Unpacking signal 21 ------------------*/
          {
            real_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[31]);
                }

                unpackedValue = tempValue;
              }

              outValue = (real_T) (unpackedValue);
            }

            {
              real_T result = (real_T) outValue;
              result = result + -40.0;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o22 = result;
            }
          }

          /* --------------- START Unpacking signal 22 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[32]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o23 = result;
            }
          }

          /* --------------- START Unpacking signal 23 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[36]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o24 = result;
            }
          }

          /* --------------- START Unpacking signal 24 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[37]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o25 = result;
            }
          }

          /* --------------- START Unpacking signal 25 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[38]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o26 = result;
            }
          }

          /* --------------- START Unpacking signal 26 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[39]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o27 = result;
            }
          }

          /* --------------- START Unpacking signal 27 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[40]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o28 = result;
            }
          }

          /* --------------- START Unpacking signal 28 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[41]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o29 = result;
            }
          }

          /* --------------- START Unpacking signal 29 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[42]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o30 = result;
            }
          }

          /* --------------- START Unpacking signal 30 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[43]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o31 = result;
            }
          }

          /* --------------- START Unpacking signal 31 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[44]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o32 = result;
            }
          }

          /* --------------- START Unpacking signal 32 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[45]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o33 = result;
            }
          }

          /* --------------- START Unpacking signal 33 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[52]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o34 = result;
            }
          }

          /* --------------- START Unpacking signal 34 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[49]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o35 = result;
            }
          }

          /* --------------- START Unpacking signal 35 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[48]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o36 = result;
            }
          }

          /* --------------- START Unpacking signal 36 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[47]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[46]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o37 = result;
            }
          }

          /* --------------- START Unpacking signal 37 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[51]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[50]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o38 = result;
            }
          }

          /* --------------- START Unpacking signal 38 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[53]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o39 = result;
            }
          }

          /* --------------- START Unpacking signal 39 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[54]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o40 = result;
            }
          }

          /* --------------- START Unpacking signal 40 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[56]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[55]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_VoltMaxCellValue = result;
            }
          }

          /* --------------- START Unpacking signal 41 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[57]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o42 = result;
            }
          }

          /* --------------- START Unpacking signal 42 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[58]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o43 = result;
            }
          }

          /* --------------- START Unpacking signal 43 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[60]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[59]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_VoltMinCellValue = result;
            }
          }
        }
      }
    }
  }

  tmp_0 = rt_modd(1000.0 * CANFDRcvFcn_BCU_B.CANFDUnpack2_o10, 4.294967E+9);
  if (tmp_0 < 4.294967296E+9) {
    BCU_EngryAccumulateChrg = (uint32_T)tmp_0;
  } else {
    BCU_EngryAccumulateChrg = MAX_uint32_T;
  }

  CFunction_o1 = (uint16_T)(BCU_EngryAccumulateChrg >> 16);
  CFunction_o2 = (uint16_T)(BCU_EngryAccumulateChrg & 65535U);
  tmp_0 = rt_modd(1000.0 * CANFDRcvFcn_BCU_B.CANFDUnpack2_o13, 4.294967E+9);
  if (tmp_0 < 4.294967296E+9) {
    BCU_EngryAccumulateDisChrg = (uint32_T)tmp_0;
  } else {
    BCU_EngryAccumulateDisChrg = MAX_uint32_T;
  }

  CFunction1_o1 = (uint16_T)(BCU_EngryAccumulateDisChrg >> 16);
  CFunction1_o2 = (uint16_T)(BCU_EngryAccumulateDisChrg & 65535U);
  tmp_1 = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o15),
                4.2949673E+9F);
  rtb_DataTypeConversion40 = tmp_1 < 0.0F ? (uint32_T)-(int32_T)(uint32_T)-tmp_1
    : (uint32_T)tmp_1;

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x180310E4== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o2 = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[2]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o3 = result;
            }
          }

          /* --------------- START Unpacking signal 3 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[3]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o4 = result;
            }
          }

          /* --------------- START Unpacking signal 4 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[4]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o5 = result;
            }
          }

          /* --------------- START Unpacking signal 5 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[5]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o6 = result;
            }
          }

          /* --------------- START Unpacking signal 6 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[6]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o7 = result;
            }
          }

          /* --------------- START Unpacking signal 7 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[7]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o8 = result;
            }
          }

          /* --------------- START Unpacking signal 8 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[8]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o9 = result;
            }
          }

          /* --------------- START Unpacking signal 9 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[9]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o10 = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[10]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o11 = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[11]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o12 = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[12]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o13 = result;
            }
          }

          /* --------------- START Unpacking signal 13 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[13]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o14 = result;
            }
          }

          /* --------------- START Unpacking signal 14 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[14]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o15 = result;
            }
          }

          /* --------------- START Unpacking signal 15 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[39]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[38]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_BMUConnErrNum = result;
            }
          }

          /* --------------- START Unpacking signal 16 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[26]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_FasCOMaxIdx = result;
            }
          }

          /* --------------- START Unpacking signal 17 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[25]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[24]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_FasCOMaxValue = result;
            }
          }

          /* --------------- START Unpacking signal 18 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[23]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_FasH2MaxIdx = result;
            }
          }

          /* --------------- START Unpacking signal 19 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[22]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[21]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_FasH2MaxValue = result;
            }
          }

          /* --------------- START Unpacking signal 20 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[17]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_FasLightMaxIdx = result;
            }
          }

          /* --------------- START Unpacking signal 21 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[16]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[15]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_FasLightMaxValue = result;
            }
          }

          /* --------------- START Unpacking signal 22 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[20]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_FasPressMaxIdx = result;
            }
          }

          /* --------------- START Unpacking signal 23 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[19]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[18]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              BCU_FasPressMaxValue = result;
            }
          }

          /* --------------- START Unpacking signal 24 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[35]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[34]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o25 = result;
            }
          }

          /* --------------- START Unpacking signal 25 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[37]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[36]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack_o26 = result;
            }
          }

          /* --------------- START Unpacking signal 26 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[27]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_TimeWeek = result;
            }
          }

          /* --------------- START Unpacking signal 27 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[30]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_TimeDay = result;
            }
          }

          /* --------------- START Unpacking signal 28 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[31]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_TimeHour = result;
            }
          }

          /* --------------- START Unpacking signal 29 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[32]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_TimeMinute = result;
            }
          }

          /* --------------- START Unpacking signal 30 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[29]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_TimeMonth = result;
            }
          }

          /* --------------- START Unpacking signal 31 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[33]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_TimeSencond = result;
            }
          }

          /* --------------- START Unpacking signal 32 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[28]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              BCU_TimeYear = result;
            }
          }
        }
      }
    }
  }

  CANFDRcvFcn_BCU_SocProcess(CANFDRcvFcn_BCU_B.CANFDUnpack1_o22,
    &rtb_ShiftArithmetic3);
  u = 0.00999999F * (real32_T)rtb_ShiftArithmetic3;
  if (u >= 0.5F) {
    i = (int32_T)floorf(u + 0.5F);
  } else {
    i = 0;
  }

  BCU_SOC = (uint16_T)fmodf((real32_T)i, 65536.0F);
  BCU_BCUVersion = (uint16_T)((BCU_BCUVersion_H << 8) + BCU_BCUVersion_L);
  if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o27 > 16) {
    b_0 = 16U;
  } else if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o27 < 1) {
    b_0 = 1U;
  } else {
    b_0 = CANFDRcvFcn_BCU_B.CANFDUnpack2_o27;
  }

  BCU_TempMaxIdx = (uint16_T)((int32_T)((uint32_T)((b_0 - 1) << 7) + ((uint32_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o28 << 3)) >> 3);
  BCU_TempMaxValue = (uint16_T)CANFDRcvFcn_BCU_B.CANFDUnpack2_o29;
  if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o30 > 16) {
    b_0 = 16U;
  } else if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o30 < 1) {
    b_0 = 1U;
  } else {
    b_0 = CANFDRcvFcn_BCU_B.CANFDUnpack2_o30;
  }

  BCU_TempMinIdx = (uint16_T)((int32_T)((uint32_T)((b_0 - 1) << 7) + ((uint32_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o31 << 3)) >> 3);
  BCU_TempMinValue = (uint16_T)CANFDRcvFcn_BCU_B.CANFDUnpack2_o32;
  u = 10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o29;
  v = fabsf(u);
  if (v < 8.388608E+6F) {
    if (v >= 0.5F) {
      u = floorf(u + 0.5F);
    } else {
      u = 0.0F;
    }
  }

  tmp_1 = fmodf(u, 65536.0F);
  rtb_DataTypeConversion29 = (uint16_T)(tmp_1 < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-tmp_1 : (int32_T)(uint16_T)tmp_1);
  BCU_VoltMinIdx = (uint16_T)((int32_T)((uint32_T)((uint8_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack2_o42 - 1) << 7) + ((uint32_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o43 << 3)) >> 3);

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack4' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x1B0110E4== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[2]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o2 = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[3]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o3 = result;
            }
          }

          /* --------------- START Unpacking signal 3 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[4]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o4 = result;
            }
          }

          /* --------------- START Unpacking signal 4 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[5]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o5 = result;
            }
          }

          /* --------------- START Unpacking signal 5 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[6]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o6 = result;
            }
          }

          /* --------------- START Unpacking signal 6 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[7]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o7 = result;
            }
          }

          /* --------------- START Unpacking signal 7 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[8]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o8 = result;
            }
          }

          /* --------------- START Unpacking signal 8 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[9]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o9 = result;
            }
          }

          /* --------------- START Unpacking signal 9 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[10]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o10 = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[11]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o11 = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[12]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o12 = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[13]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o13 = result;
            }
          }

          /* --------------- START Unpacking signal 13 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[14]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o14 = result;
            }
          }

          /* --------------- START Unpacking signal 14 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[15]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o15 = result;
            }
          }

          /* --------------- START Unpacking signal 15 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[16]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o16 = result;
            }
          }

          /* --------------- START Unpacking signal 16 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[17]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o17 = result;
            }
          }

          /* --------------- START Unpacking signal 17 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[18]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o18 = result;
            }
          }

          /* --------------- START Unpacking signal 18 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[19]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o19 = result;
            }
          }

          /* --------------- START Unpacking signal 19 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[20]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o20 = result;
            }
          }

          /* --------------- START Unpacking signal 20 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[21]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o21 = result;
            }
          }

          /* --------------- START Unpacking signal 21 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[22]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o22 = result;
            }
          }

          /* --------------- START Unpacking signal 22 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[23]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o23 = result;
            }
          }

          /* --------------- START Unpacking signal 23 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[24]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o24 = result;
            }
          }

          /* --------------- START Unpacking signal 24 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[25]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o25 = result;
            }
          }

          /* --------------- START Unpacking signal 25 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[26]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o26 = result;
            }
          }

          /* --------------- START Unpacking signal 26 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[27]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o27 = result;
            }
          }

          /* --------------- START Unpacking signal 27 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[28]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o28 = result;
            }
          }

          /* --------------- START Unpacking signal 28 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[29]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o29 = result;
            }
          }

          /* --------------- START Unpacking signal 29 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[30]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o30 = result;
            }
          }

          /* --------------- START Unpacking signal 30 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[31]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o31 = result;
            }
          }

          /* --------------- START Unpacking signal 31 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[32]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o32 = result;
            }
          }

          /* --------------- START Unpacking signal 32 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[33]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o33 = result;
            }
          }

          /* --------------- START Unpacking signal 33 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[34]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o34 = result;
            }
          }

          /* --------------- START Unpacking signal 34 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[35]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o35 = result;
            }
          }

          /* --------------- START Unpacking signal 35 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[36]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o36 = result;
            }
          }

          /* --------------- START Unpacking signal 36 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[37]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o37 = result;
            }
          }

          /* --------------- START Unpacking signal 37 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[38]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o38 = result;
            }
          }

          /* --------------- START Unpacking signal 38 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[39]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o39 = result;
            }
          }

          /* --------------- START Unpacking signal 39 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[40]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o40 = result;
            }
          }

          /* --------------- START Unpacking signal 40 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[41]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o41 = result;
            }
          }

          /* --------------- START Unpacking signal 41 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[42]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o42 = result;
            }
          }

          /* --------------- START Unpacking signal 42 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[43]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o43 = result;
            }
          }

          /* --------------- START Unpacking signal 43 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[44]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o44 = result;
            }
          }

          /* --------------- START Unpacking signal 44 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[45]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o45 = result;
            }
          }

          /* --------------- START Unpacking signal 45 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[46]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o46 = result;
            }
          }

          /* --------------- START Unpacking signal 46 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[47]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o47 = result;
            }
          }

          /* --------------- START Unpacking signal 47 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[48]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o48 = result;
            }
          }

          /* --------------- START Unpacking signal 48 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[49]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o49 = result;
            }
          }

          /* --------------- START Unpacking signal 49 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[50]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o50 = result;
            }
          }

          /* --------------- START Unpacking signal 50 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[51]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o51 = result;
            }
          }

          /* --------------- START Unpacking signal 51 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[52]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o52 = result;
            }
          }

          /* --------------- START Unpacking signal 52 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[53]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o53 = result;
            }
          }

          /* --------------- START Unpacking signal 53 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[54]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o54 = result;
            }
          }

          /* --------------- START Unpacking signal 54 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[55]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o55 = result;
            }
          }

          /* --------------- START Unpacking signal 55 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[56]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o56 = result;
            }
          }

          /* --------------- START Unpacking signal 56 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[57]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o57 = result;
            }
          }

          /* --------------- START Unpacking signal 57 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[58]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o58 = result;
            }
          }

          /* --------------- START Unpacking signal 58 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[59]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o59 = result;
            }
          }

          /* --------------- START Unpacking signal 59 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[60]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o60 = result;
            }
          }

          /* --------------- START Unpacking signal 60 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack4_o61 = result;
            }
          }
        }
      }
    }
  }

  if ((uint8_T)(CANFDRcvFcn_BCU_B.CANFDUnpack4_o61 - 1) <= 1) {
    port_index = (uint8_T)(CANFDRcvFcn_BCU_B.CANFDUnpack4_o61 - 1);
  } else {
    port_index = 1U;
  }

  rtb_TmpSignalConversionAtSFun_a[0] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o1;
  rtb_TmpSignalConversionAtSFun_a[1] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o2;
  rtb_TmpSignalConversionAtSFun_a[2] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o3;
  rtb_TmpSignalConversionAtSFun_a[3] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o4;
  rtb_TmpSignalConversionAtSFun_a[4] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o5;
  rtb_TmpSignalConversionAtSFun_a[5] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o6;
  rtb_TmpSignalConversionAtSFun_a[6] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o7;
  rtb_TmpSignalConversionAtSFun_a[7] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o8;
  rtb_TmpSignalConversionAtSFun_a[8] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o9;
  rtb_TmpSignalConversionAtSFun_a[9] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o10;
  rtb_TmpSignalConversionAtSFun_a[10] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o11;
  rtb_TmpSignalConversionAtSFun_a[11] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o12;
  rtb_TmpSignalConversionAtSFun_a[12] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o13;
  rtb_TmpSignalConversionAtSFun_a[13] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o14;
  rtb_TmpSignalConversionAtSFun_a[14] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o15;
  rtb_TmpSignalConversionAtSFun_a[15] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o16;
  rtb_TmpSignalConversionAtSFun_a[16] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o17;
  rtb_TmpSignalConversionAtSFun_a[17] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o18;
  rtb_TmpSignalConversionAtSFun_a[18] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o19;
  rtb_TmpSignalConversionAtSFun_a[19] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o20;
  rtb_TmpSignalConversionAtSFun_a[20] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o21;
  rtb_TmpSignalConversionAtSFun_a[21] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o22;
  rtb_TmpSignalConversionAtSFun_a[22] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o23;
  rtb_TmpSignalConversionAtSFun_a[23] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o24;
  rtb_TmpSignalConversionAtSFun_a[24] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o25;
  rtb_TmpSignalConversionAtSFun_a[25] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o26;
  rtb_TmpSignalConversionAtSFun_a[26] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o27;
  rtb_TmpSignalConversionAtSFun_a[27] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o28;
  rtb_TmpSignalConversionAtSFun_a[28] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o29;
  rtb_TmpSignalConversionAtSFun_a[29] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o30;
  rtb_TmpSignalConversionAtSFun_a[30] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o31;
  rtb_TmpSignalConversionAtSFun_a[31] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o32;
  rtb_TmpSignalConversionAtSFun_a[32] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o33;
  rtb_TmpSignalConversionAtSFun_a[33] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o34;
  rtb_TmpSignalConversionAtSFun_a[34] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o35;
  rtb_TmpSignalConversionAtSFun_a[35] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o36;
  rtb_TmpSignalConversionAtSFun_a[36] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o37;
  rtb_TmpSignalConversionAtSFun_a[37] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o38;
  rtb_TmpSignalConversionAtSFun_a[38] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o39;
  rtb_TmpSignalConversionAtSFun_a[39] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o40;
  rtb_TmpSignalConversionAtSFun_a[40] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o41;
  rtb_TmpSignalConversionAtSFun_a[41] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o42;
  rtb_TmpSignalConversionAtSFun_a[42] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o43;
  rtb_TmpSignalConversionAtSFun_a[43] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o44;
  rtb_TmpSignalConversionAtSFun_a[44] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o45;
  rtb_TmpSignalConversionAtSFun_a[45] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o46;
  rtb_TmpSignalConversionAtSFun_a[46] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o47;
  rtb_TmpSignalConversionAtSFun_a[47] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o48;
  rtb_TmpSignalConversionAtSFun_a[48] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o49;
  rtb_TmpSignalConversionAtSFun_a[49] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o50;
  rtb_TmpSignalConversionAtSFun_a[50] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o51;
  rtb_TmpSignalConversionAtSFun_a[51] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o52;
  rtb_TmpSignalConversionAtSFun_a[52] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o53;
  rtb_TmpSignalConversionAtSFun_a[53] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o54;
  rtb_TmpSignalConversionAtSFun_a[54] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o55;
  rtb_TmpSignalConversionAtSFun_a[55] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o56;
  rtb_TmpSignalConversionAtSFun_a[56] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o57;
  rtb_TmpSignalConversionAtSFun_a[57] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o58;
  rtb_TmpSignalConversionAtSFun_a[58] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o59;
  rtb_TmpSignalConversionAtSFun_a[59] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack4_o60;
  if ((uint8_T)(CANFDRcvFcn_BCU_B.CANFDUnpack4_o61 - 1) <= 1) {
    port_len = (uint8_T)(CANFDRcvFcn_BCU_B.CANFDUnpack4_o61 - 1);
  } else {
    port_len = 1U;
  }

  port_len = port_len * 60U + 1U;
  b = (uint16_T)(port_index * 60U + 1U);
  c = (uint16_T)(port_index * 60U + 60U);
  for (rtb_ShiftArithmetic3 = b; rtb_ShiftArithmetic3 <= c; rtb_ShiftArithmetic3
       ++) {
    CANFDRcvFcn_BCU_DW.tmp_k[rtb_ShiftArithmetic3 - 1] =
      rtb_TmpSignalConversionAtSFun_a[rtb_ShiftArithmetic3 - (int32_T)port_len];
  }

  memcpy(&usSingleBatTemp[0], &CANFDRcvFcn_BCU_DW.tmp_k[0], 120U * sizeof
         (uint16_T));

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack5' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x1A0110E4== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[2]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[1]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[4]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[3]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o2 = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[6]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[5]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o3 = result;
            }
          }

          /* --------------- START Unpacking signal 3 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[8]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[7]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o4 = result;
            }
          }

          /* --------------- START Unpacking signal 4 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[10]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[9]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o5 = result;
            }
          }

          /* --------------- START Unpacking signal 5 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[12]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[11]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o6 = result;
            }
          }

          /* --------------- START Unpacking signal 6 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[14]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[13]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o7 = result;
            }
          }

          /* --------------- START Unpacking signal 7 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[16]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[15]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o8 = result;
            }
          }

          /* --------------- START Unpacking signal 8 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[18]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[17]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o9 = result;
            }
          }

          /* --------------- START Unpacking signal 9 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[20]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[19]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o10 = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[22]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[21]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o11 = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[24]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[23]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o12 = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[26]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[25]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o13 = result;
            }
          }

          /* --------------- START Unpacking signal 13 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[28]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[27]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o14 = result;
            }
          }

          /* --------------- START Unpacking signal 14 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[30]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[29]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o15 = result;
            }
          }

          /* --------------- START Unpacking signal 15 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[32]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[31]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o16 = result;
            }
          }

          /* --------------- START Unpacking signal 16 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[34]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[33]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o17 = result;
            }
          }

          /* --------------- START Unpacking signal 17 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[36]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[35]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o18 = result;
            }
          }

          /* --------------- START Unpacking signal 18 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[38]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[37]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o19 = result;
            }
          }

          /* --------------- START Unpacking signal 19 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[40]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[39]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o20 = result;
            }
          }

          /* --------------- START Unpacking signal 20 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[42]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[41]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o21 = result;
            }
          }

          /* --------------- START Unpacking signal 21 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[44]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[43]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o22 = result;
            }
          }

          /* --------------- START Unpacking signal 22 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[46]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[45]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o23 = result;
            }
          }

          /* --------------- START Unpacking signal 23 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[48]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[47]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o24 = result;
            }
          }

          /* --------------- START Unpacking signal 24 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[50]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[49]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o25 = result;
            }
          }

          /* --------------- START Unpacking signal 25 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[52]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[51]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o26 = result;
            }
          }

          /* --------------- START Unpacking signal 26 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[54]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[53]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o27 = result;
            }
          }

          /* --------------- START Unpacking signal 27 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[56]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[55]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o28 = result;
            }
          }

          /* --------------- START Unpacking signal 28 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[58]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[57]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o29 = result;
            }
          }

          /* --------------- START Unpacking signal 29 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[60]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[59]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o30 = result;
            }
          }

          /* --------------- START Unpacking signal 30 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack5_o31 = result;
            }
          }
        }
      }
    }
  }

  if ((uint8_T)(CANFDRcvFcn_BCU_B.CANFDUnpack5_o31 - 1) <= 7) {
    rtb_Saturation2 = (uint8_T)(CANFDRcvFcn_BCU_B.CANFDUnpack5_o31 - 1);
  } else {
    rtb_Saturation2 = 7U;
  }

  rtb_TmpSignalConversionAtSFu_kh[0] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o1) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[1] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o2) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[2] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o3) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[3] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o4) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[4] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o5) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[5] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o6) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[6] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o7) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[7] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o8) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[8] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o9) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[9] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o10) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[10] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o11) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[11] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o12) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[12] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o13) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[13] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o14) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[14] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o15) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[15] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o16) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[16] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o17) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[17] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o18) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[18] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o19) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[19] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o20) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[20] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o21) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[21] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o22) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[22] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o23) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[23] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o24) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[24] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o25) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[25] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o26) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[26] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o27) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[27] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o28) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[28] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o29) >> 19);
  rtb_TmpSignalConversionAtSFu_kh[29] = (uint16_T)((52429U *
    CANFDRcvFcn_BCU_B.CANFDUnpack5_o30) >> 19);
  port_len = rtb_Saturation2 * 30U + 1U;
  b_0 = (uint8_T)((uint8_T)(rtb_Saturation2 * 30) + 1);
  c_0 = (uint8_T)((uint8_T)(rtb_Saturation2 * 30) + 30);
  for (rtb_Saturation2 = b_0; rtb_Saturation2 <= c_0; rtb_Saturation2++) {
    CANFDRcvFcn_BCU_DW.tmp[rtb_Saturation2 - 1] =
      rtb_TmpSignalConversionAtSFu_kh[rtb_Saturation2 - (uint8_T)port_len];
  }

  memcpy(&usSingleBatVal[0], &CANFDRcvFcn_BCU_DW.tmp[0], 240U * sizeof(uint16_T));
  BCU_VoltMaxIdx = (uint16_T)((int32_T)((uint32_T)((uint8_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack2_o39 - 1) << 7) + ((uint32_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o40 << 3)) >> 3);
  ThermCtrl_ACWarmGoal = (uint16_T)CANFDRcvFcn_BCU_B.CANFDUnpack1_o32;
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967294U) |
    CANFDRcvFcn_BCU_ConstB.DataTypeConversion27);
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967293U) | (uint32_T)
    (CANFDRcvFcn_BCU_ConstB.DataTypeConversion28 << 1));
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967291U) | (uint32_T)
    (CANFDRcvFcn_BCU_ConstB.DataTypeConversion29 << 2));
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967287U) | (uint32_T)
    (CANFDRcvFcn_BCU_ConstB.DataTypeConversion30 << 3));
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967279U) | (uint32_T)
    ((CANFDRcvFcn_BCU_B.CANFDUnpack1_o33 != 0) << 4));
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967263U) | (uint32_T)
    (CANFDRcvFcn_BCU_ConstB.DataTypeConversion38 << 5));
  CANFDRcvFcn_BCU_SocProcess(CANFDRcvFcn_BCU_B.CANFDUnpack1_o21,
    &rtb_ShiftArithmetic3);
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_value =
    CANFDRcvFcn_BCU_B.CANFDUnpack1_o20;// ����ת����

  // ����1��ʹ�� memcpy������ָ��������⣬�Ƽ���
  memcpy(&CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits,
         &CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_value, sizeof(float));

  // ����2��ֱ��ָ��ת�����������ϸ�������⣬���Ƽ���
  // float_bits = *(uint32_t *)&float_value;

  // ��ȡ�ֽڣ������ DCBA��
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteD =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits >> 24) & 0xFF;// �����Ч�ֽ�
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteC =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits >> 16) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteB =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits >> 8) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteA =
    CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits & 0xFF;// �����Ч�ֽ�

  // ��ϳ����� 16 λ�Ĵ���
  CANFDRcvFcn_BCU_B.BCU_RealtimePower_H =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteD << 8) |
    CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteC;// DC �Ĵ���
  CANFDRcvFcn_BCU_B.BCU_RealtimePower_L =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteB << 8) |
    CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteA;// BA �Ĵ���
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_value = (uint32_T)
    rtb_DataTypeConversion29 /10.0f;   // ����ת����

  // ����1��ʹ�� memcpy������ָ��������⣬�Ƽ���
  memcpy(&CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits,
         &CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_value, sizeof(float));

  // ����2��ֱ��ָ��ת�����������ϸ�������⣬���Ƽ���
  // float_bits = *(uint32_t *)&float_value;

  // ��ȡ�ֽڣ������ DCBA��
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteD =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits >> 24) & 0xFF;// �����Ч�ֽ�
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteC =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits >> 16) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteB =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits >> 8) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteA =
    CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits & 0xFF;// �����Ч�ֽ�

  // ��ϳ����� 16 λ�Ĵ���
  CANFDRcvFcn_BCU_B.BCU_V4_L = (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteD << 8) |
    CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteC;// DC �Ĵ���
  CANFDRcvFcn_BCU_B.BCU_V4_H = (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteB << 8) |
    CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteA;// BA �Ĵ���
  tmp_1 = fmodf(floorf(CANFDRcvFcn_BCU_B.CANFDUnpack1_o25), 65536.0F);
  tmp = fmodf(floorf(CANFDRcvFcn_BCU_B.CANFDUnpack1_o26), 65536.0F);

  /* Bit to Integer Conversion */
  /* Input bit order is MSB first */
  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack3' */
    if ((8 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x18FFC13A== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[3]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.AmbientTemp = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[7]) & (uint8_T)(0xC0U)) >> 6);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.DTCLevel = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)(CANFDRcvMsg.Data[7])
                    & (uint8_T)(0x3FU));
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              Chiller_Fault = result;
            }
          }

          /* --------------- START Unpacking signal 3 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[4]);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.InWaterPressure = result;
            }
          }

          /* --------------- START Unpacking signal 4 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[2]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.InWaterTemp = result;
            }
          }

          /* --------------- START Unpacking signal 5 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[0]) & (uint8_T)(0x8U)) >> 3);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.K1_State = result;
            }
          }

          /* --------------- START Unpacking signal 6 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[0]) & (uint8_T)(0x10U)) >> 4);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.K2_State = result;
            }
          }

          /* --------------- START Unpacking signal 7 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[5]);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.OutWaterPressure = result;
            }
          }

          /* --------------- START Unpacking signal 8 ------------------*/
          {
            int16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (int16_T) (unpackedValue);
            }

            {
              int16_T result = (int16_T) outValue;
              result = result + -40;
              CANFDRcvFcn_BCU_B.OutWaterTemp = result;
            }
          }

          /* --------------- START Unpacking signal 9 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[0]) & (uint8_T)(0x40U)) >> 6);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.PreHeating_FB = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)(CANFDRcvMsg.Data[0])
                    & (uint8_T)(0x7U));
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              Chiller_ModeFb = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[0]) & (uint8_T)(0x80U)) >> 7);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.Sclience_Mode = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------*/
          {
            real32_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[6]);
                }

                unpackedValue = tempValue;
              }

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.TMS_Power_Req = result;
            }
          }
        }
      }
    }
  }

  BCU_TemperatureOutBox = (uint16_T)CANFDRcvFcn_BCU_B.AmbientTemp;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.InWaterPressure), 65536.0F);
  Chiller_InletPressure = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)-(int16_T)
    (uint16_T)-u : (int32_T)(uint16_T)u);
  Chiller_TempInlet = (uint16_T)(10 * CANFDRcvFcn_BCU_B.InWaterTemp);
  Chiller_TempOutlet = (uint16_T)(10 * CANFDRcvFcn_BCU_B.OutWaterTemp);

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack6' */
    if ((8 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x18FFC13B== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[6]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.FanPWM_Rx = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[3]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[2]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.HighPress = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint16_T unpackedValue = 0;

              {
                uint16_T tempValue = (uint16_T) (0);

                {
                  tempValue = tempValue | (uint16_T)(CANFDRcvMsg.Data[5]);
                  tempValue = tempValue | (uint16_T)((uint16_T)
                    (CANFDRcvMsg.Data[4]) << 8);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.LowPress = result;
            }
          }
        }
      }
    }
  }

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack7' */
    if ((8 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x18FFC13C== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint8_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[2]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANFDRcvFcn_BCU_B.ACP_RPM = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)((uint8_T)((uint8_T)
                    (CANFDRcvMsg.Data[3]) & (uint8_T)(0x2U)) >> 1);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              Chiller_PumpStatus = result;
            }
          }
        }
      }
    }
  }

  Chiller_CompressorStatus = (uint16_T)(100U * CANFDRcvFcn_BCU_B.ACP_RPM >= 500U);

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack8' */
    if ((8 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x18FFC13D== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[7]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.AC_SWVersion = result;
            }
          }
        }
      }
    }
  }

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack9' */
    if ((8 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x18FAE6E2== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.ACP_Ver_Major = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.ACP_Ver_Minor = result;
            }
          }
        }
      }
    }
  }

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack10' */
    if ((8 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x18FA78F5== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.DCDC_Ver_Major = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.DCDC_Ver_Minor = result;
            }
          }
        }
      }
    }
  }

  BCU_TemperatureInBox = (uint16_T)CANFDRcvFcn_BCU_B.CANFDUnpack2_o18;

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack11' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((0x180410E4== CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o2 = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[2]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o3 = result;
            }
          }

          /* --------------- START Unpacking signal 3 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[3]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o4 = result;
            }
          }

          /* --------------- START Unpacking signal 4 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[4]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o5 = result;
            }
          }

          /* --------------- START Unpacking signal 5 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[5]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o6 = result;
            }
          }

          /* --------------- START Unpacking signal 6 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[6]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o7 = result;
            }
          }

          /* --------------- START Unpacking signal 7 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[7]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o8 = result;
            }
          }

          /* --------------- START Unpacking signal 8 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[8]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o9 = result;
            }
          }

          /* --------------- START Unpacking signal 9 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[9]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o10 = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[10]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o11 = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[11]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o12 = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[12]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o13 = result;
            }
          }

          /* --------------- START Unpacking signal 13 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[13]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o14 = result;
            }
          }

          /* --------------- START Unpacking signal 14 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[14]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o15 = result;
            }
          }

          /* --------------- START Unpacking signal 15 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[15]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o16 = result;
            }
          }

          /* --------------- START Unpacking signal 16 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[16]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o17 = result;
            }
          }

          /* --------------- START Unpacking signal 17 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[17]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o18 = result;
            }
          }

          /* --------------- START Unpacking signal 18 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[18]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o19 = result;
            }
          }

          /* --------------- START Unpacking signal 19 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[19]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o20 = result;
            }
          }

          /* --------------- START Unpacking signal 20 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[20]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o21 = result;
            }
          }

          /* --------------- START Unpacking signal 21 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[21]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o22 = result;
            }
          }

          /* --------------- START Unpacking signal 22 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[22]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o23 = result;
            }
          }

          /* --------------- START Unpacking signal 23 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[23]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o24 = result;
            }
          }

          /* --------------- START Unpacking signal 24 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[24]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o25 = result;
            }
          }

          /* --------------- START Unpacking signal 25 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[25]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o26 = result;
            }
          }

          /* --------------- START Unpacking signal 26 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[26]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o27 = result;
            }
          }

          /* --------------- START Unpacking signal 27 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[27]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o28 = result;
            }
          }

          /* --------------- START Unpacking signal 28 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[28]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o29 = result;
            }
          }

          /* --------------- START Unpacking signal 29 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[29]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o30 = result;
            }
          }

          /* --------------- START Unpacking signal 30 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[30]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o31 = result;
            }
          }

          /* --------------- START Unpacking signal 31 ------------------*/
          {
            uint16_T outValue = 0;

            {
              uint8_T unpackedValue = 0;

              {
                uint8_T tempValue = (uint8_T) (0);

                {
                  tempValue = tempValue | (uint8_T)(CANFDRcvMsg.Data[31]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint16_T) (unpackedValue);
            }

            {
              uint16_T result = (uint16_T) outValue;
              CANFDRcvFcn_BCU_B.CANFDUnpack11_o32 = result;
            }
          }
        }
      }
    }
  }

  rtb_TmpSignalConversionAtSFunct[0] = BCU_SystemWorkMode;
  rtb_TmpSignalConversionAtSFunct[1] = BCU_Curr;
  rtb_TmpSignalConversionAtSFunct[2] = BCU_Curr;
  rtb_TmpSignalConversionAtSFunct[3] = CANFDRcvFcn_BCU_B.BCU_Curr2_H;
  rtb_TmpSignalConversionAtSFunct[4] = CANFDRcvFcn_BCU_B.BCU_Curr2_L;
  rtb_TmpSignalConversionAtSFunct[5] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o4;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o8), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[6] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o9), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[7] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[8] = (uint16_T)((uint16_T)((uint16_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack1_o17 << 2) |
    CANFDRcvFcn_BCU_B.CANFDUnpack1_o18) | (uint16_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack1_o19 << 4));
  rtb_TmpSignalConversionAtSFunct[9] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o6;
  rtb_TmpSignalConversionAtSFunct[10] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o7;
  rtb_TmpSignalConversionAtSFunct[11] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o3;
  rtb_TmpSignalConversionAtSFunct[12] = CFunction_o1;
  rtb_TmpSignalConversionAtSFunct[13] = CFunction_o2;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o11), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[14] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[15] = CFunction1_o1;
  rtb_TmpSignalConversionAtSFunct[16] = CFunction1_o2;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o14), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[17] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o12), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[18] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[19] = (uint16_T)(rtb_DataTypeConversion40 &
    65535U);
  rtb_TmpSignalConversionAtSFunct[20] = (uint16_T)(rtb_DataTypeConversion40 >>
    16);
  rtb_TmpSignalConversionAtSFunct[21] = BCU_FasCOMaxIdx;
  rtb_TmpSignalConversionAtSFunct[22] = BCU_FasCOMaxValue;
  rtb_TmpSignalConversionAtSFunct[23] = BCU_FasH2MaxIdx;
  rtb_TmpSignalConversionAtSFunct[24] = BCU_FasH2MaxValue;
  rtb_TmpSignalConversionAtSFunct[25] = BCU_FasLightMaxIdx;
  rtb_TmpSignalConversionAtSFunct[26] = BCU_FasLightMaxValue;
  rtb_TmpSignalConversionAtSFunct[27] = BCU_FasPressMaxIdx;
  rtb_TmpSignalConversionAtSFunct[28] = BCU_FasPressMaxValue;
  rtb_TmpSignalConversionAtSFunct[29] = (uint16_T)(BCU_FaultInfoLv1 >> 16);
  rtb_TmpSignalConversionAtSFunct[30] = (uint16_T)(BCU_FaultInfoLv1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[31] = (uint16_T)(BCU_FaultInfoLv2 >> 16);
  rtb_TmpSignalConversionAtSFunct[32] = (uint16_T)(BCU_FaultInfoLv2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[33] = (uint16_T)(BCU_FaultInfoLv3 >> 16);
  rtb_TmpSignalConversionAtSFunct[34] = (uint16_T)(BCU_FaultInfoLv3 & 65535U);
  rtb_TmpSignalConversionAtSFunct[35] = (uint16_T)(BCU_FaultInfoLv4 >> 16);
  rtb_TmpSignalConversionAtSFunct[36] = (uint16_T)(BCU_FaultInfoLv4 & 65535U);
  rtb_TmpSignalConversionAtSFunct[37] = CANFDRcvFcn_BCU_B.CANFDUnpack1_o16;
  rtb_TmpSignalConversionAtSFunct[38] = CANFDRcvFcn_BCU_B.CANFDUnpack1_o14;
  rtb_TmpSignalConversionAtSFunct[39] = CANFDRcvFcn_BCU_B.CANFDUnpack1_o15;
  rtb_TmpSignalConversionAtSFunct[40] = BCU_SOC;
  u = 0.00999999F * (real32_T)CANFDRcvFcn_BCU_B.CANFDUnpack1_o24;
  if (u >= 0.5F) {
    i = (int32_T)floorf(u + 0.5F);
  } else {
    i = 0;
  }

  rtb_TmpSignalConversionAtSFunct[41] = (uint16_T)fmodf((real32_T)i, 65536.0F);
  rtb_TmpSignalConversionAtSFunct[42] = BCU_BCUVersion;
  tmp_0 = fmod(floor(CANFDRcvFcn_BCU_B.CANFDUnpack2_o22), 65536.0);
  rtb_TmpSignalConversionAtSFunct[43] = (uint16_T)(tmp_0 < 0.0 ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-tmp_0 : (int32_T)(uint16_T)tmp_0);
  rtb_TmpSignalConversionAtSFunct[44] = (uint16_T)((uint16_T)((uint16_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack2_o19 - 1) << 4) +
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o20);
  rtb_TmpSignalConversionAtSFunct[45] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o21;
  rtb_TmpSignalConversionAtSFunct[46] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o23;
  rtb_TmpSignalConversionAtSFunct[47] = BCU_TempMaxIdx;
  rtb_TmpSignalConversionAtSFunct[48] = BCU_TempMaxValue;
  rtb_TmpSignalConversionAtSFunct[49] = BCU_TempMinIdx;
  rtb_TmpSignalConversionAtSFunct[50] = BCU_TempMinValue;
  rtb_TmpSignalConversionAtSFunct[51] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o33;
  u = 10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o27;
  v = fabsf(u);
  if (v < 8.388608E+6F) {
    if (v >= 0.5F) {
      u = floorf(u + 0.5F);
    } else {
      u = 0.0F;
    }
  }

  u = fmodf(u, 65536.0F);
  rtb_TmpSignalConversionAtSFunct[52] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  u = 10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o28;
  v = fabsf(u);
  if (v < 8.388608E+6F) {
    if (v >= 0.5F) {
      u = floorf(u + 0.5F);
    } else {
      u = 0.0F;
    }
  }

  u = fmodf(u, 65536.0F);
  rtb_TmpSignalConversionAtSFunct[53] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[54] = rtb_DataTypeConversion29;
  rtb_TmpSignalConversionAtSFunct[55] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o37;
  rtb_TmpSignalConversionAtSFunct[56] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o38;
  rtb_TmpSignalConversionAtSFunct[57] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o36;
  if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o34 > 16) {
    b_0 = 16U;
  } else if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o34 < 1) {
    b_0 = 1U;
  } else {
    b_0 = CANFDRcvFcn_BCU_B.CANFDUnpack2_o34;
  }

  rtb_TmpSignalConversionAtSFunct[58] = (uint16_T)((int32_T)((uint32_T)((b_0 - 1)
    << 7) + ((uint32_T)CANFDRcvFcn_BCU_B.CANFDUnpack2_o35 << 3)) >> 3);
  rtb_TmpSignalConversionAtSFunct[59] = BCU_VoltMaxCellValue;
  rtb_TmpSignalConversionAtSFunct[60] = BCU_VoltMinCellValue;
  rtb_TmpSignalConversionAtSFunct[61] = BCU_VoltMinIdx;
  memcpy(&rtb_TmpSignalConversionAtSFunct[62], &usSingleBatTemp[0], 120U *
         sizeof(uint16_T));
  memcpy(&rtb_TmpSignalConversionAtSFunct[182], &usSingleBatVal[0], 240U *
         sizeof(uint16_T));
  rtb_TmpSignalConversionAtSFunct[422] = BCU_VoltMaxIdx;
  rtb_TmpSignalConversionAtSFunct[423] = ThermCtrl_ACWarmGoal;
  rtb_TmpSignalConversionAtSFunct[424] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack1_o31;
  rtb_TmpSignalConversionAtSFunct[425] = ThermCtrl_Fault;
  u = 0.00999999F * (real32_T)rtb_ShiftArithmetic3;
  if (u >= 0.5F) {
    i = (int32_T)floorf(u + 0.5F);
  } else {
    i = 0;
  }

  rtb_TmpSignalConversionAtSFunct[426] = (uint16_T)fmodf((real32_T)i, 65536.0F);
  rtb_TmpSignalConversionAtSFunct[427] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o25;
  rtb_TmpSignalConversionAtSFunct[428] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o26;
  rtb_TmpSignalConversionAtSFunct[429] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o24;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o30), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[430] = (uint16_T)(u < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[431] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o1;
  rtb_TmpSignalConversionAtSFunct[432] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o2;
  rtb_TmpSignalConversionAtSFunct[433] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o3;
  rtb_TmpSignalConversionAtSFunct[434] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o4;
  rtb_TmpSignalConversionAtSFunct[435] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o5;
  rtb_TmpSignalConversionAtSFunct[436] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o6;
  rtb_TmpSignalConversionAtSFunct[437] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o7;
  rtb_TmpSignalConversionAtSFunct[438] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o8;
  rtb_TmpSignalConversionAtSFunct[439] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o9;
  rtb_TmpSignalConversionAtSFunct[440] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o10;
  rtb_TmpSignalConversionAtSFunct[441] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o11;
  rtb_TmpSignalConversionAtSFunct[442] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o12;
  rtb_TmpSignalConversionAtSFunct[443] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o13;
  rtb_TmpSignalConversionAtSFunct[444] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o14;
  rtb_TmpSignalConversionAtSFunct[445] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o15;
  rtb_TmpSignalConversionAtSFunct[446] = CANFDRcvFcn_BCU_B.BCU_RealtimePower_H;
  rtb_TmpSignalConversionAtSFunct[447] = CANFDRcvFcn_BCU_B.BCU_RealtimePower_L;
  rtb_TmpSignalConversionAtSFunct[448] = CANFDRcvFcn_BCU_B.BCU_V4_L;
  rtb_TmpSignalConversionAtSFunct[449] = CANFDRcvFcn_BCU_B.BCU_V4_H;
  rtb_TmpSignalConversionAtSFunct[450] = CFunction_o1;
  rtb_TmpSignalConversionAtSFunct[451] = CFunction_o2;
  rtb_TmpSignalConversionAtSFunct[452] = CFunction1_o1;
  rtb_TmpSignalConversionAtSFunct[453] = CFunction1_o2;
  rtb_TmpSignalConversionAtSFunct[454] = (uint16_T)((uint32_T)(tmp_1 < 0.0F ?
    (int32_T)(uint16_T)-(int16_T)(uint16_T)-tmp_1 : (int32_T)(uint16_T)tmp_1) <<
    1U | (uint32_T)(tmp < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)-tmp :
                    (int32_T)(uint16_T)tmp));
  rtb_TmpSignalConversionAtSFunct[455] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o5;
  rtb_TmpSignalConversionAtSFunct[456] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o8;
  rtb_TmpSignalConversionAtSFunct[457] = BCU_TemperatureOutBox;
  rtb_TmpSignalConversionAtSFunct[458] = Chiller_Fault;
  rtb_TmpSignalConversionAtSFunct[459] = Chiller_InletPressure;
  rtb_TmpSignalConversionAtSFunct[460] = Chiller_TempInlet;
  tmp_1 = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.OutWaterPressure), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[461] = (uint16_T)(tmp_1 < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-tmp_1 : (int32_T)(uint16_T)tmp_1);
  rtb_TmpSignalConversionAtSFunct[462] = Chiller_TempOutlet;
  rtb_TmpSignalConversionAtSFunct[463] = Chiller_ModeFb;
  rtb_TmpSignalConversionAtSFunct[464] = CANFDRcvFcn_BCU_B.Sclience_Mode;
  tmp_1 = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.TMS_Power_Req), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[465] = (uint16_T)(tmp_1 < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-tmp_1 : (int32_T)(uint16_T)tmp_1);
  rtb_TmpSignalConversionAtSFunct[466] = CANFDRcvFcn_BCU_B.FanPWM_Rx;
  rtb_TmpSignalConversionAtSFunct[467] = CANFDRcvFcn_BCU_B.HighPress;
  rtb_TmpSignalConversionAtSFunct[468] = CANFDRcvFcn_BCU_B.LowPress;
  rtb_TmpSignalConversionAtSFunct[469] = Chiller_CompressorStatus;
  rtb_TmpSignalConversionAtSFunct[470] = Chiller_PumpStatus;
  rtb_TmpSignalConversionAtSFunct[471] = CANFDRcvFcn_BCU_B.AC_SWVersion;
  rtb_TmpSignalConversionAtSFunct[472] = (uint16_T)((uint32_T)
    (CANFDRcvFcn_BCU_B.ACP_Ver_Major << 8) | CANFDRcvFcn_BCU_B.ACP_Ver_Minor);
  rtb_TmpSignalConversionAtSFunct[473] = (uint16_T)((uint32_T)
    (CANFDRcvFcn_BCU_B.DCDC_Ver_Major << 8) | CANFDRcvFcn_BCU_B.DCDC_Ver_Minor);
  rtb_TmpSignalConversionAtSFunct[474] = BCU_TemperatureOutBox;
  rtb_TmpSignalConversionAtSFunct[475] = BCU_TemperatureInBox;
  rtb_TmpSignalConversionAtSFunct[476] = BCU_BatteryStatus;
  rtb_TmpSignalConversionAtSFunct[477] = (uint16_T)((uint16_T)((uint16_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack1_o1 << 2) | CANFDRcvFcn_BCU_B.CANFDUnpack1_o2)
    | (uint16_T)(CANFDRcvFcn_BCU_B.CANFDUnpack1_o3 << 4));
  rtb_TmpSignalConversionAtSFunct[478] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o1;
  rtb_TmpSignalConversionAtSFunct[479] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o2;
  rtb_TmpSignalConversionAtSFunct[480] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o3;
  rtb_TmpSignalConversionAtSFunct[481] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o4;
  rtb_TmpSignalConversionAtSFunct[482] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o5;
  rtb_TmpSignalConversionAtSFunct[483] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o6;
  rtb_TmpSignalConversionAtSFunct[484] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o7;
  rtb_TmpSignalConversionAtSFunct[485] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o8;
  rtb_TmpSignalConversionAtSFunct[486] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o9;
  rtb_TmpSignalConversionAtSFunct[487] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o10;
  rtb_TmpSignalConversionAtSFunct[488] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o11;
  rtb_TmpSignalConversionAtSFunct[489] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o12;
  rtb_TmpSignalConversionAtSFunct[490] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o13;
  rtb_TmpSignalConversionAtSFunct[491] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o14;
  rtb_TmpSignalConversionAtSFunct[492] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o15;
  rtb_TmpSignalConversionAtSFunct[493] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o16;
  rtb_TmpSignalConversionAtSFunct[494] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o17;
  rtb_TmpSignalConversionAtSFunct[495] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o18;
  rtb_TmpSignalConversionAtSFunct[496] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o19;
  rtb_TmpSignalConversionAtSFunct[497] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o20;
  rtb_TmpSignalConversionAtSFunct[498] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o21;
  rtb_TmpSignalConversionAtSFunct[499] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o22;
  rtb_TmpSignalConversionAtSFunct[500] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o23;
  rtb_TmpSignalConversionAtSFunct[501] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o24;
  rtb_TmpSignalConversionAtSFunct[502] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o25;
  rtb_TmpSignalConversionAtSFunct[503] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o26;
  rtb_TmpSignalConversionAtSFunct[504] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o27;
  rtb_TmpSignalConversionAtSFunct[505] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o28;
  rtb_TmpSignalConversionAtSFunct[506] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o29;
  rtb_TmpSignalConversionAtSFunct[507] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o30;
  rtb_TmpSignalConversionAtSFunct[508] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o31;
  rtb_TmpSignalConversionAtSFunct[509] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o32;
  rtb_TmpSignalConversionAtSFunct[510] = CANFDRcvFcn_BCU_B.CANFDUnpack_o25;
  rtb_TmpSignalConversionAtSFunct[511] = CANFDRcvFcn_BCU_B.CANFDUnpack_o26;
  rtb_TmpSignalConversionAtSFunct[512] = BCU_BMUConnErrNum;
  rtb_DataTypeConversion40 = 0U;
  port_index = 0U;
  port_len = sizeof(uint32_T);
  if (port_len == 0U) {
    port_len = MAX_uint32_T;

    /* Divide by zero handler */
  } else {
    port_len = sizeof(uint32_T [110]) / port_len;
  }

  while (port_index < port_len) {
    if (CANFDRcvFcn_BCU_ConstP.portDimensions_Value[port_index] == 1U) {
      modbusBuff[CANFDRcvFcn_BCU_ConstP.index_Value[port_index]] =
        rtb_TmpSignalConversionAtSFunct[rtb_DataTypeConversion40];
      qY = rtb_DataTypeConversion40 + 1U;
      if (rtb_DataTypeConversion40 + 1U < rtb_DataTypeConversion40) {
        qY = MAX_uint32_T;
      }

      rtb_DataTypeConversion40 = qY;
    } else {
      rtb_ShiftArithmetic3 = 0U;
      while (rtb_ShiftArithmetic3 <
             CANFDRcvFcn_BCU_ConstP.portDimensions_Value[port_index]) {
        q0 = CANFDRcvFcn_BCU_ConstP.index_Value[port_index];
        qY = q0 + rtb_ShiftArithmetic3;
        if (qY < q0) {
          qY = MAX_uint32_T;
        }

        modbusBuff[qY] =
          rtb_TmpSignalConversionAtSFunct[rtb_DataTypeConversion40];
        qY = rtb_DataTypeConversion40 + 1U;
        if (rtb_DataTypeConversion40 + 1U < rtb_DataTypeConversion40) {
          qY = MAX_uint32_T;
        }

        rtb_DataTypeConversion40 = qY;
        i = rtb_ShiftArithmetic3 + 1;
        if (rtb_ShiftArithmetic3 + 1 > 65535) {
          i = 65535;
        }

        rtb_ShiftArithmetic3 = (uint16_T)i;
      }
    }

    port_index++;
  }

  BCU_SNCode[0] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o1;
  BCU_SNCode[1] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o2;
  BCU_SNCode[2] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o3;
  BCU_SNCode[3] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o4;
  BCU_SNCode[4] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o5;
  BCU_SNCode[5] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o6;
  BCU_SNCode[6] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o7;
  BCU_SNCode[7] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o8;
  BCU_SNCode[8] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o9;
  BCU_SNCode[9] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o10;
  BCU_SNCode[10] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o11;
  BCU_SNCode[11] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o12;
  BCU_SNCode[12] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o13;
  BCU_SNCode[13] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o14;
  BCU_SNCode[14] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o15;
  BCU_SNCode[15] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o16;
  BCU_SNCode[16] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o17;
  BCU_SNCode[17] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o18;
  BCU_SNCode[18] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o19;
  BCU_SNCode[19] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o20;
  BCU_SNCode[20] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o21;
  BCU_SNCode[21] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o22;
  BCU_SNCode[22] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o23;
  BCU_SNCode[23] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o24;
  BCU_SNCode[24] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o25;
  BCU_SNCode[25] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o26;
  BCU_SNCode[26] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o27;
  BCU_SNCode[27] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o28;
  BCU_SNCode[28] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o29;
  BCU_SNCode[29] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o30;
  BCU_SNCode[30] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o31;
  BCU_SNCode[31] = CANFDRcvFcn_BCU_B.CANFDUnpack11_o32;
  tmp_1 = fmodf(floorf(CANFDRcvFcn_BCU_B.CANFDUnpack1_o20), 4.2949673E+9F);
  BCU_RealtimePower = tmp_1 < 0.0F ? (uint32_T)-(int32_T)(uint32_T)-tmp_1 :
    (uint32_T)tmp_1;
}

/* Model initialize function */
void CANFDRcvFcn_BCU_initialize(void)
{
  /*-----------S-Function Block: <S1>/CAN FD Unpack1 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack2 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack4 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack5 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack3 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack6 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack7 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack8 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack9 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack10 -----------------*/

  /*-----------S-Function Block: <S1>/CAN FD Unpack11 -----------------*/
}

/* Model terminate function */
void CANFDRcvFcn_BCU_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
