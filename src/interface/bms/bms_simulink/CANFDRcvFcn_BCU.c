/*
 * File: CANFDRcvFcn_BCU.c
 *
 * Code generated for Simulink model 'CANFDRcvFcn_BCU'.
 *
 * Model version                  : 5.251
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Wed Mar 11 17:16:29 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CANFDRcvFcn_BCU.h"
#include <math.h>
#include <string.h>
#include "CANFDRcvFcn_BCU_types.h"
#include "rtwtypes.h"
#include "CANFDRcvFcn_BCU_private.h"

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

/* Model step function */
void CANFDRcvFcn_BCU_step(void)
{
  real_T tmp;
  int32_T i_1;
  real32_T u;
  real32_T v;
  uint32_T port_index;
  uint32_T port_len;
  uint32_T q0;
  uint32_T qY;
  uint32_T rtb_DataTypeConversion40;
  uint16_T rtb_TmpSignalConversionAtSFunct[477];
  uint16_T rtb_TmpSignalConversionAtSFun_a[60];
  uint16_T rtb_TmpSignalConversionAtSFu_kh[30];
  uint16_T CFunction1_o1;
  uint16_T CFunction1_o2;
  uint16_T CFunction_o1;
  uint16_T CFunction_o2;
  uint16_T b;
  uint16_T c;
  uint16_T i;
  uint16_T rtb_DataTypeConversion29;
  uint8_T b_0;
  uint8_T i_0;
  uint8_T rtb_Saturation2;

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack1' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((402723044 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 0
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 88
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = -3200.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o2 = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------
           *  startBit                = 104
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = -3200.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o3 = result;
            }
          }

          /* --------------- START Unpacking signal 3 ------------------
           *  startBit                = 16
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o4 = result;
            }
          }

          /* --------------- START Unpacking signal 4 ------------------
           *  startBit                = 32
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o5 = result;
            }
          }

          /* --------------- START Unpacking signal 5 ------------------
           *  startBit                = 248
           *  length                  = 32
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 6 ------------------
           *  startBit                = 280
           *  length                  = 32
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 7 ------------------
           *  startBit                = 312
           *  length                  = 32
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 8 ------------------
           *  startBit                = 344
           *  length                  = 32
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 9 ------------------
           *  startBit                = 376
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o10 = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------
           *  startBit                = 392
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o11 = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------
           *  startBit                = 408
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o12 = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------
           *  startBit                = 506
           *  length                  = 2
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o13 = result;
            }
          }

          /* --------------- START Unpacking signal 13 ------------------
           *  startBit                = 504
           *  length                  = 2
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o14 = result;
            }
          }

          /* --------------- START Unpacking signal 14 ------------------
           *  startBit                = 508
           *  length                  = 2
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o15 = result;
            }
          }

          /* --------------- START Unpacking signal 15 ------------------
           *  startBit                = 72
           *  length                  = 24
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -8.388608E+6
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o16 = result;
            }
          }

          /* --------------- START Unpacking signal 16 ------------------
           *  startBit                = 168
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o17 = result;
            }
          }

          /* --------------- START Unpacking signal 17 ------------------
           *  startBit                = 184
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o18 = result;
            }
          }

          /* --------------- START Unpacking signal 18 ------------------
           *  startBit                = 216
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o19 = result;
            }
          }

          /* --------------- START Unpacking signal 19 ------------------
           *  startBit                = 200
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o20 = result;
            }
          }

          /* --------------- START Unpacking signal 20 ------------------
           *  startBit                = 511
           *  length                  = 1
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o21 = result;
            }
          }

          /* --------------- START Unpacking signal 21 ------------------
           *  startBit                = 510
           *  length                  = 1
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o22 = result;
            }
          }

          /* --------------- START Unpacking signal 22 ------------------
           *  startBit                = 120
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o23 = result;
            }
          }

          /* --------------- START Unpacking signal 23 ------------------
           *  startBit                = 136
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o24 = result;
            }
          }

          /* --------------- START Unpacking signal 24 ------------------
           *  startBit                = 152
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o25 = result;
            }
          }

          /* --------------- START Unpacking signal 25 ------------------
           *  startBit                = 48
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o26 = result;
            }
          }

          /* --------------- START Unpacking signal 26 ------------------
           *  startBit                = 448
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o27 = result;
            }
          }

          /* --------------- START Unpacking signal 27 ------------------
           *  startBit                = 464
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o28 = result;
            }
          }

          /* --------------- START Unpacking signal 28 ------------------
           *  startBit                = 456
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack1_o29 = result;
            }
          }
        }
      }
    }
  }

  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o2), 65536.0F);
  BCU_Curr = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)-u :
                        (int32_T)(uint16_T)u);
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_value =
    CANFDRcvFcn_BCU_B.CANFDUnpack1_o3; // 锟斤拷锟斤拷转锟斤拷锟斤拷

  // 锟斤拷锟斤拷1锟斤拷使锟斤拷 memcpy锟斤拷锟斤拷锟斤拷指锟斤拷锟斤拷锟斤拷锟斤拷猓拷萍锟斤拷锟�
  memcpy(&CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits,
         &CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_value, sizeof(float));

  // 锟斤拷锟斤拷2锟斤拷直锟斤拷指锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟较革拷锟斤拷锟斤拷锟斤拷猓拷锟斤拷萍锟斤拷锟�
  // float_bits = *(uint32_t *)&float_value;

  // 锟斤拷取锟街节ｏ拷锟斤拷锟斤拷锟� DCBA锟斤拷
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteD =
    (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits >> 24) & 0xFF;// 锟斤拷锟斤拷锟叫э拷纸锟�
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteC =
    (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits >> 16) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteB =
    (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits >> 8) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteA =
    CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_float_bits & 0xFF;// 锟斤拷锟斤拷锟叫э拷纸锟�

  // 锟斤拷铣锟斤拷锟斤拷锟� 16 位锟侥达拷锟斤拷
  CANFDRcvFcn_BCU_B.BCU_Curr2_H = (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteD <<
    8) | CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteC;// DC 锟侥达拷锟斤拷
  CANFDRcvFcn_BCU_B.BCU_Curr2_L = (CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteB <<
    8) | CANFDRcvFcn_BCU_DW.U32_to_Folat_Curr_byteA;// BA 锟侥达拷锟斤拷

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack2' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((402788580 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 0
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 8
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o2 = result;
            }
          }

          /* --------------- START Unpacking signal 2 ------------------
           *  startBit                = 16
           *  length                  = 8
           *  desiredSignalByteLayout = LITTLEENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 3 ------------------
           *  startBit                = 32
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 4 ------------------
           *  startBit                = 40
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 5 ------------------
           *  startBit                = 48
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 6 ------------------
           *  startBit                = 56
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 7 ------------------
           *  startBit                = 64
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 8 ------------------
           *  startBit                = 80
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o9 = result;
            }
          }

          /* --------------- START Unpacking signal 9 ------------------
           *  startBit                = 96
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o10 = result;
            }
          }

          /* --------------- START Unpacking signal 10 ------------------
           *  startBit                = 128
           *  length                  = 32
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
          {
            real32_T outValue = 0;

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

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o11 = result;
            }
          }

          /* --------------- START Unpacking signal 11 ------------------
           *  startBit                = 144
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o12 = result;
            }
          }

          /* --------------- START Unpacking signal 12 ------------------
           *  startBit                = 160
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o13 = result;
            }
          }

          /* --------------- START Unpacking signal 13 ------------------
           *  startBit                = 192
           *  length                  = 32
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
          {
            real32_T outValue = 0;

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

              outValue = (real32_T) (unpackedValue);
            }

            {
              real32_T result = (real32_T) outValue;
              result = result * 0.1F;
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o14 = result;
            }
          }

          /* --------------- START Unpacking signal 14 ------------------
           *  startBit                = 208
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o15 = result;
            }
          }

          /* --------------- START Unpacking signal 15 ------------------
           *  startBit                = 224
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o16 = result;
            }
          }

          /* --------------- START Unpacking signal 16 ------------------
           *  startBit                = 232
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
          /*
           * Signal is not connected or connected to terminator.
           * No unpacking code generated.
           */

          /* --------------- START Unpacking signal 17 ------------------
           *  startBit                = 240
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
          /*
           * Signal is not connected or connected to terminator.
           * No unpacking code generated.
           */

          /* --------------- START Unpacking signal 18 ------------------
           *  startBit                = 488
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o19 = result;
            }
          }

          /* --------------- START Unpacking signal 19 ------------------
           *  startBit                = 264
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o20 = result;
            }
          }

          /* --------------- START Unpacking signal 20 ------------------
           *  startBit                = 272
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o21 = result;
            }
          }

          /* --------------- START Unpacking signal 21 ------------------
           *  startBit                = 280
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o22 = result;
            }
          }

          /* --------------- START Unpacking signal 22 ------------------
           *  startBit                = 248
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o23 = result;
            }
          }

          /* --------------- START Unpacking signal 23 ------------------
           *  startBit                = 256
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o24 = result;
            }
          }

          /* --------------- START Unpacking signal 24 ------------------
           *  startBit                = 288
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o25 = result;
            }
          }

          /* --------------- START Unpacking signal 25 ------------------
           *  startBit                = 296
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o26 = result;
            }
          }

          /* --------------- START Unpacking signal 26 ------------------
           *  startBit                = 304
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o27 = result;
            }
          }

          /* --------------- START Unpacking signal 27 ------------------
           *  startBit                = 312
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o28 = result;
            }
          }

          /* --------------- START Unpacking signal 28 ------------------
           *  startBit                = 320
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o29 = result;
            }
          }

          /* --------------- START Unpacking signal 29 ------------------
           *  startBit                = 328
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o30 = result;
            }
          }

          /* --------------- START Unpacking signal 30 ------------------
           *  startBit                = 336
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o31 = result;
            }
          }

          /* --------------- START Unpacking signal 31 ------------------
           *  startBit                = 344
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o32 = result;
            }
          }

          /* --------------- START Unpacking signal 32 ------------------
           *  startBit                = 352
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o33 = result;
            }
          }

          /* --------------- START Unpacking signal 33 ------------------
           *  startBit                = 360
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o34 = result;
            }
          }

          /* --------------- START Unpacking signal 34 ------------------
           *  startBit                = 416
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o35 = result;
            }
          }

          /* --------------- START Unpacking signal 35 ------------------
           *  startBit                = 392
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o36 = result;
            }
          }

          /* --------------- START Unpacking signal 36 ------------------
           *  startBit                = 384
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o37 = result;
            }
          }

          /* --------------- START Unpacking signal 37 ------------------
           *  startBit                = 376
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o38 = result;
            }
          }

          /* --------------- START Unpacking signal 38 ------------------
           *  startBit                = 408
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o39 = result;
            }
          }

          /* --------------- START Unpacking signal 39 ------------------
           *  startBit                = 424
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o40 = result;
            }
          }

          /* --------------- START Unpacking signal 40 ------------------
           *  startBit                = 432
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o41 = result;
            }
          }

          /* --------------- START Unpacking signal 41 ------------------
           *  startBit                = 448
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 42 ------------------
           *  startBit                = 456
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o43 = result;
            }
          }

          /* --------------- START Unpacking signal 43 ------------------
           *  startBit                = 464
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
              CANFDRcvFcn_BCU_B.CANFDUnpack2_o44 = result;
            }
          }

          /* --------------- START Unpacking signal 44 ------------------
           *  startBit                = 480
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

  u = 1000.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o11;
  if (u < 4.2949673E+9F) {
    if (u >= 0.0F) {
      BCU_EngryAccumulateChrg = (uint32_T)u;
    } else {
      BCU_EngryAccumulateChrg = 0U;
    }
  } else {
    BCU_EngryAccumulateChrg = MAX_uint32_T;
  }

  CFunction_o1 = (uint16_T)(BCU_EngryAccumulateChrg >> 16);
  CFunction_o2 = (uint16_T)(BCU_EngryAccumulateChrg & 65535U);
  u = 1000.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o14;
  if (u < 4.2949673E+9F) {
    if (u >= 0.0F) {
      BCU_EngryAccumulateDisChrg = (uint32_T)u;
    } else {
      BCU_EngryAccumulateDisChrg = 0U;
    }
  } else {
    BCU_EngryAccumulateDisChrg = MAX_uint32_T;
  }

  CFunction1_o1 = (uint16_T)(BCU_EngryAccumulateDisChrg >> 16);
  CFunction1_o2 = (uint16_T)(BCU_EngryAccumulateDisChrg & 65535U);
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o16), 4.2949673E+9F);
  rtb_DataTypeConversion40 = u < 0.0F ? (uint32_T)-(int32_T)(uint32_T)-u :
    (uint32_T)u;

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((402854116 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 0
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 8
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 2 ------------------
           *  startBit                = 16
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 3 ------------------
           *  startBit                = 24
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 4 ------------------
           *  startBit                = 32
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 5 ------------------
           *  startBit                = 40
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 6 ------------------
           *  startBit                = 48
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 7 ------------------
           *  startBit                = 56
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 8 ------------------
           *  startBit                = 64
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 9 ------------------
           *  startBit                = 72
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 10 ------------------
           *  startBit                = 80
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 11 ------------------
           *  startBit                = 88
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 12 ------------------
           *  startBit                = 96
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 13 ------------------
           *  startBit                = 104
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 14 ------------------
           *  startBit                = 112
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 15 ------------------
           *  startBit                = 208
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 16 ------------------
           *  startBit                = 200
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 17 ------------------
           *  startBit                = 184
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 18 ------------------
           *  startBit                = 176
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 19 ------------------
           *  startBit                = 136
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 20 ------------------
           *  startBit                = 128
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 21 ------------------
           *  startBit                = 160
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 22 ------------------
           *  startBit                = 152
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 23 ------------------
           *  startBit                = 216
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 24 ------------------
           *  startBit                = 240
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 25 ------------------
           *  startBit                = 248
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 26 ------------------
           *  startBit                = 256
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 27 ------------------
           *  startBit                = 232
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 28 ------------------
           *  startBit                = 264
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 29 ------------------
           *  startBit                = 224
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

  u = 0.00999999F * (real32_T)CANFDRcvFcn_BCU_B.CANFDUnpack1_o18;
  if (u >= 0.5F) {
    i_1 = (int32_T)floorf(u + 0.5F);
  } else {
    i_1 = 0;
  }

  BCU_SOC = (uint16_T)fmodf((real32_T)i_1, 65536.0F);
  if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o28 > 16) {
    b_0 = 16U;
  } else if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o28 < 1) {
    b_0 = 1U;
  } else {
    b_0 = CANFDRcvFcn_BCU_B.CANFDUnpack2_o28;
  }

  BCU_TempMaxIdx = (uint16_T)((int32_T)((uint32_T)((b_0 - 1) << 7) + ((uint32_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o29 << 3)) >> 3);
  BCU_TempMaxValue = (uint16_T)CANFDRcvFcn_BCU_B.CANFDUnpack2_o30;
  if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o31 > 16) {
    b_0 = 16U;
  } else if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o31 < 1) {
    b_0 = 1U;
  } else {
    b_0 = CANFDRcvFcn_BCU_B.CANFDUnpack2_o31;
  }

  BCU_TempMinIdx = (uint16_T)((int32_T)((uint32_T)((b_0 - 1) << 7) + ((uint32_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o32 << 3)) >> 3);
  BCU_TempMinValue = (uint16_T)CANFDRcvFcn_BCU_B.CANFDUnpack2_o33;
  u = 10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o25;
  v = fabsf(u);
  if (v < 8.388608E+6F) {
    if (v >= 0.5F) {
      u = floorf(u + 0.5F);
    } else {
      u = 0.0F;
    }
  }

  u = fmodf(u, 65536.0F);
  rtb_DataTypeConversion29 = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)-(int16_T)
    (uint16_T)-u : (int32_T)(uint16_T)u);
  BCU_VoltMinIdx = (uint16_T)((int32_T)((uint32_T)((uint8_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack2_o43 - 1) << 7) + ((uint32_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o44 << 3)) >> 3);

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack4' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((453054692 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 8
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 16
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 2 ------------------
           *  startBit                = 24
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 3 ------------------
           *  startBit                = 32
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 4 ------------------
           *  startBit                = 40
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 5 ------------------
           *  startBit                = 48
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 6 ------------------
           *  startBit                = 56
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 7 ------------------
           *  startBit                = 64
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 8 ------------------
           *  startBit                = 72
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 9 ------------------
           *  startBit                = 80
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 10 ------------------
           *  startBit                = 88
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 11 ------------------
           *  startBit                = 96
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 12 ------------------
           *  startBit                = 104
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 13 ------------------
           *  startBit                = 112
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 14 ------------------
           *  startBit                = 120
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 15 ------------------
           *  startBit                = 128
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 16 ------------------
           *  startBit                = 136
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 17 ------------------
           *  startBit                = 144
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 18 ------------------
           *  startBit                = 152
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 19 ------------------
           *  startBit                = 160
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 20 ------------------
           *  startBit                = 168
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 21 ------------------
           *  startBit                = 176
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 22 ------------------
           *  startBit                = 184
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 23 ------------------
           *  startBit                = 192
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 24 ------------------
           *  startBit                = 200
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 25 ------------------
           *  startBit                = 208
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 26 ------------------
           *  startBit                = 216
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 27 ------------------
           *  startBit                = 224
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 28 ------------------
           *  startBit                = 232
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 29 ------------------
           *  startBit                = 240
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 30 ------------------
           *  startBit                = 248
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 31 ------------------
           *  startBit                = 256
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 32 ------------------
           *  startBit                = 264
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 33 ------------------
           *  startBit                = 272
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 34 ------------------
           *  startBit                = 280
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 35 ------------------
           *  startBit                = 288
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 36 ------------------
           *  startBit                = 296
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 37 ------------------
           *  startBit                = 304
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 38 ------------------
           *  startBit                = 312
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 39 ------------------
           *  startBit                = 320
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 40 ------------------
           *  startBit                = 328
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 41 ------------------
           *  startBit                = 336
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 42 ------------------
           *  startBit                = 344
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 43 ------------------
           *  startBit                = 352
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 44 ------------------
           *  startBit                = 360
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 45 ------------------
           *  startBit                = 368
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 46 ------------------
           *  startBit                = 376
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 47 ------------------
           *  startBit                = 384
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 48 ------------------
           *  startBit                = 392
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 49 ------------------
           *  startBit                = 400
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 50 ------------------
           *  startBit                = 408
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 51 ------------------
           *  startBit                = 416
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 52 ------------------
           *  startBit                = 424
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 53 ------------------
           *  startBit                = 432
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 54 ------------------
           *  startBit                = 440
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 55 ------------------
           *  startBit                = 448
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 56 ------------------
           *  startBit                = 456
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 57 ------------------
           *  startBit                = 464
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 58 ------------------
           *  startBit                = 472
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 59 ------------------
           *  startBit                = 480
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 60 ------------------
           *  startBit                = 0
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
  for (i = b; i <= c; i++) {
    CANFDRcvFcn_BCU_DW.tmp_a[i - 1] = rtb_TmpSignalConversionAtSFun_a[i -
      (int32_T)port_len];
  }

  memcpy(&usSingleBatTemp[0], &CANFDRcvFcn_BCU_DW.tmp_a[0], 120U * sizeof
         (uint16_T));

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack5' */
    if ((64 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((436277476 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 16
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 32
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 2 ------------------
           *  startBit                = 48
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 3 ------------------
           *  startBit                = 64
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 4 ------------------
           *  startBit                = 80
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 5 ------------------
           *  startBit                = 96
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 6 ------------------
           *  startBit                = 112
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 7 ------------------
           *  startBit                = 128
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 8 ------------------
           *  startBit                = 144
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 9 ------------------
           *  startBit                = 160
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 10 ------------------
           *  startBit                = 176
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 11 ------------------
           *  startBit                = 192
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 12 ------------------
           *  startBit                = 208
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 13 ------------------
           *  startBit                = 224
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 14 ------------------
           *  startBit                = 240
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 15 ------------------
           *  startBit                = 256
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 16 ------------------
           *  startBit                = 272
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 17 ------------------
           *  startBit                = 288
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 18 ------------------
           *  startBit                = 304
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 19 ------------------
           *  startBit                = 320
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 20 ------------------
           *  startBit                = 336
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 21 ------------------
           *  startBit                = 352
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 22 ------------------
           *  startBit                = 368
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 23 ------------------
           *  startBit                = 384
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 24 ------------------
           *  startBit                = 400
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 25 ------------------
           *  startBit                = 416
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 26 ------------------
           *  startBit                = 432
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 27 ------------------
           *  startBit                = 448
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 28 ------------------
           *  startBit                = 464
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 29 ------------------
           *  startBit                = 480
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 30 ------------------
           *  startBit                = 0
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
  rtb_Saturation2 = (uint8_T)((uint8_T)(rtb_Saturation2 * 30) + 30);
  for (i_0 = b_0; i_0 <= rtb_Saturation2; i_0++) {
    CANFDRcvFcn_BCU_DW.tmp[i_0 - 1] = rtb_TmpSignalConversionAtSFu_kh[i_0 -
      (uint8_T)port_len];
  }

  memcpy(&usSingleBatVal[0], &CANFDRcvFcn_BCU_DW.tmp[0], 240U * sizeof(uint16_T));
  BCU_VoltMaxIdx = (uint16_T)((int32_T)((uint32_T)((uint8_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack2_o40 - 1) << 7) + ((uint32_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o41 << 3)) >> 3);
  ThermCtrl_ACWarmGoal = (uint16_T)CANFDRcvFcn_BCU_B.CANFDUnpack1_o29;
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967294U) |
    CANFDRcvFcn_BCU_ConstB.DataTypeConversion27);
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967293U) | (uint32_T)
    (CANFDRcvFcn_BCU_ConstB.DataTypeConversion28 << 1));
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967291U) | (uint32_T)
    (CANFDRcvFcn_BCU_ConstB.DataTypeConversion29 << 2));
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967287U) | (uint32_T)
    (CANFDRcvFcn_BCU_ConstB.DataTypeConversion30 << 3));
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967279U) | (uint32_T)
    ((CANFDRcvFcn_BCU_B.CANFDUnpack1_o28 != 0) << 4));
  ThermCtrl_Fault = (uint16_T)((ThermCtrl_Fault & 4294967263U) | (uint32_T)
    (CANFDRcvFcn_BCU_ConstB.DataTypeConversion38 << 5));
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_value =
    CANFDRcvFcn_BCU_B.CANFDUnpack1_o16;// 锟斤拷锟斤拷转锟斤拷锟斤拷

  // 锟斤拷锟斤拷1锟斤拷使锟斤拷 memcpy锟斤拷锟斤拷锟斤拷指锟斤拷锟斤拷锟斤拷锟斤拷猓拷萍锟斤拷锟�
  memcpy(&CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits,
         &CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_value, sizeof(float));

  // 锟斤拷锟斤拷2锟斤拷直锟斤拷指锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟较革拷锟斤拷锟斤拷锟斤拷猓拷锟斤拷萍锟斤拷锟�
  // float_bits = *(uint32_t *)&float_value;

  // 锟斤拷取锟街节ｏ拷锟斤拷锟斤拷锟� DCBA锟斤拷
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteD =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits >> 24) & 0xFF;// 锟斤拷锟斤拷锟叫э拷纸锟�
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteC =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits >> 16) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteB =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits >> 8) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteA =
    CANFDRcvFcn_BCU_DW.U32_to_F32_Power_float_bits & 0xFF;// 锟斤拷锟斤拷锟叫э拷纸锟�

  // 锟斤拷铣锟斤拷锟斤拷锟� 16 位锟侥达拷锟斤拷
  CANFDRcvFcn_BCU_B.BCU_RealtimePower_H =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteD << 8) |
    CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteC;// DC 锟侥达拷锟斤拷
  CANFDRcvFcn_BCU_B.BCU_RealtimePower_L =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteB << 8) |
    CANFDRcvFcn_BCU_DW.U32_to_F32_Power_byteA;// BA 锟侥达拷锟斤拷
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_value = (uint32_T)
    rtb_DataTypeConversion29 /10.0f;   // 锟斤拷锟斤拷转锟斤拷锟斤拷

  // 锟斤拷锟斤拷1锟斤拷使锟斤拷 memcpy锟斤拷锟斤拷锟斤拷指锟斤拷锟斤拷锟斤拷锟斤拷猓拷萍锟斤拷锟�
  memcpy(&CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits,
         &CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_value, sizeof(float));

  // 锟斤拷锟斤拷2锟斤拷直锟斤拷指锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟较革拷锟斤拷锟斤拷锟斤拷猓拷锟斤拷萍锟斤拷锟�
  // float_bits = *(uint32_t *)&float_value;

  // 锟斤拷取锟街节ｏ拷锟斤拷锟斤拷锟� DCBA锟斤拷
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteD =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits >> 24) & 0xFF;// 锟斤拷锟斤拷锟叫э拷纸锟�
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteC =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits >> 16) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteB =
    (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits >> 8) & 0xFF;
  CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteA =
    CANFDRcvFcn_BCU_DW.U32_to_F32_V3_float_bits & 0xFF;// 锟斤拷锟斤拷锟叫э拷纸锟�

  // 锟斤拷铣锟斤拷锟斤拷锟� 16 位锟侥达拷锟斤拷
  CANFDRcvFcn_BCU_B.BCU_V4_L = (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteD << 8) |
    CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteC;// DC 锟侥达拷锟斤拷
  CANFDRcvFcn_BCU_B.BCU_V4_H = (CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteB << 8) |
    CANFDRcvFcn_BCU_DW.U32_to_F32_V3_byteA;// BA 锟侥达拷锟斤拷
  u = fmodf(floorf(CANFDRcvFcn_BCU_B.CANFDUnpack1_o21), 65536.0F);
  v = fmodf(floorf(CANFDRcvFcn_BCU_B.CANFDUnpack1_o22), 65536.0F);

  /* Bit to Integer Conversion */
  port_index = 0U;

  /* Input bit order is LSB first */
  if ((u < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)-u : (int32_T)(uint16_T)
       u) != 0) {
    port_index = 1U;
  }

  /* Input bit order is LSB first */
  if ((v < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)-v : (int32_T)(uint16_T)
       v) != 0) {
    port_index |= 2U;
  }

  {
    /* S-Function (scanfdunpack): '<S1>/CAN FD Unpack3' */
    if ((8 == CANFDRcvMsg.Length) && (CANFDRcvMsg.ID != INVALID_CAN_ID) ) {
      if ((419414330 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 62
           *  length                  = 2
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 2 ------------------
           *  startBit                = 56
           *  length                  = 6
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 3 ------------------
           *  startBit                = 32
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 4 ------------------
           *  startBit                = 16
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 5 ------------------
           *  startBit                = 3
           *  length                  = 1
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 6 ------------------
           *  startBit                = 4
           *  length                  = 1
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 7 ------------------
           *  startBit                = 40
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 8 ------------------
           *  startBit                = 8
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = -40.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 9 ------------------
           *  startBit                = 6
           *  length                  = 1
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 10 ------------------
           *  startBit                = 0
           *  length                  = 3
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 11 ------------------
           *  startBit                = 7
           *  length                  = 1
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 12 ------------------
           *  startBit                = 48
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 0.1
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
      if ((419414331 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 48
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 24
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 2 ------------------
           *  startBit                = 40
           *  length                  = 16
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
      if ((419414332 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 16
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 25
           *  length                  = 1
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
      if ((419414333 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 56
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
      if ((419096290 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 0
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 8
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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
      if ((419068149 == CANFDRcvMsg.ID) && (1U == CANFDRcvMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 8
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 0
           *  length                  = 8
           *  desiredSignalByteLayout = BIGENDIAN
           *  dataType                = UNSIGNED
           *  factor                  = 1.0
           *  offset                  = 0.0
           * -----------------------------------------------------------------------*/
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

  BCU_TemperatureInBox = (uint16_T)CANFDRcvFcn_BCU_B.CANFDUnpack2_o19;
  rtb_TmpSignalConversionAtSFunct[0] = BCU_SystemWorkMode;
  rtb_TmpSignalConversionAtSFunct[1] = BCU_Curr;
  rtb_TmpSignalConversionAtSFunct[2] = BCU_Curr;
  rtb_TmpSignalConversionAtSFunct[3] = CANFDRcvFcn_BCU_B.BCU_Curr2_H;
  rtb_TmpSignalConversionAtSFunct[4] = CANFDRcvFcn_BCU_B.BCU_Curr2_L;
  rtb_TmpSignalConversionAtSFunct[5] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o4;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o4), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[6] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o5), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[7] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[8] = (uint16_T)((uint16_T)((uint16_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack1_o13 << 2) |
    CANFDRcvFcn_BCU_B.CANFDUnpack1_o14) | (uint16_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack1_o15 << 4));
  rtb_TmpSignalConversionAtSFunct[9] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o6;
  rtb_TmpSignalConversionAtSFunct[10] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o7;
  rtb_TmpSignalConversionAtSFunct[11] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o3;
  rtb_TmpSignalConversionAtSFunct[12] = CFunction_o1;
  rtb_TmpSignalConversionAtSFunct[13] = CFunction_o2;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o12), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[14] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[15] = CFunction1_o1;
  rtb_TmpSignalConversionAtSFunct[16] = CFunction1_o2;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o15), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[17] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack2_o13), 65536.0F);
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
  rtb_TmpSignalConversionAtSFunct[37] = CANFDRcvFcn_BCU_B.CANFDUnpack1_o12;
  rtb_TmpSignalConversionAtSFunct[38] = CANFDRcvFcn_BCU_B.CANFDUnpack1_o10;
  rtb_TmpSignalConversionAtSFunct[39] = CANFDRcvFcn_BCU_B.CANFDUnpack1_o11;
  rtb_TmpSignalConversionAtSFunct[40] = BCU_SOC;
  u = 0.00999999F * (real32_T)CANFDRcvFcn_BCU_B.CANFDUnpack1_o20;
  if (u >= 0.5F) {
    i_1 = (int32_T)floorf(u + 0.5F);
  } else {
    i_1 = 0;
  }

  rtb_TmpSignalConversionAtSFunct[41] = (uint16_T)fmodf((real32_T)i_1, 65536.0F);
  rtb_TmpSignalConversionAtSFunct[42] = (uint16_T)
    ((CANFDRcvFcn_BCU_B.CANFDUnpack2_o1 << 8) +
     CANFDRcvFcn_BCU_B.CANFDUnpack2_o2);
  rtb_TmpSignalConversionAtSFunct[43] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o9;
  tmp = fmod(floor(CANFDRcvFcn_BCU_B.CANFDUnpack2_o23), 65536.0);
  rtb_TmpSignalConversionAtSFunct[44] = (uint16_T)(tmp < 0.0 ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-tmp : (int32_T)(uint16_T)tmp);
  rtb_TmpSignalConversionAtSFunct[45] = (uint16_T)((uint16_T)((uint16_T)
    (CANFDRcvFcn_BCU_B.CANFDUnpack2_o20 - 1) << 4) +
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o21);
  rtb_TmpSignalConversionAtSFunct[46] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o22;
  rtb_TmpSignalConversionAtSFunct[47] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o24;
  rtb_TmpSignalConversionAtSFunct[48] = BCU_TempMaxIdx;
  rtb_TmpSignalConversionAtSFunct[49] = BCU_TempMaxValue;
  rtb_TmpSignalConversionAtSFunct[50] = BCU_TempMinIdx;
  rtb_TmpSignalConversionAtSFunct[51] = BCU_TempMinValue;
  rtb_TmpSignalConversionAtSFunct[52] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o34;
  u = 10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o23;
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
  u = 10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o24;
  v = fabsf(u);
  if (v < 8.388608E+6F) {
    if (v >= 0.5F) {
      u = floorf(u + 0.5F);
    } else {
      u = 0.0F;
    }
  }

  u = fmodf(u, 65536.0F);
  rtb_TmpSignalConversionAtSFunct[54] = (uint16_T)(u < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[55] = rtb_DataTypeConversion29;
  rtb_TmpSignalConversionAtSFunct[56] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o38;
  rtb_TmpSignalConversionAtSFunct[57] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o39;
  rtb_TmpSignalConversionAtSFunct[58] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o37;
  if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o35 > 16) {
    b_0 = 16U;
  } else if (CANFDRcvFcn_BCU_B.CANFDUnpack2_o35 < 1) {
    b_0 = 1U;
  } else {
    b_0 = CANFDRcvFcn_BCU_B.CANFDUnpack2_o35;
  }

  rtb_TmpSignalConversionAtSFunct[59] = (uint16_T)((int32_T)((uint32_T)((b_0 - 1)
    << 7) + ((uint32_T)CANFDRcvFcn_BCU_B.CANFDUnpack2_o36 << 3)) >> 3);
  rtb_TmpSignalConversionAtSFunct[60] = BCU_VoltMaxCellValue;
  rtb_TmpSignalConversionAtSFunct[61] = BCU_VoltMinCellValue;
  rtb_TmpSignalConversionAtSFunct[62] = BCU_VoltMinIdx;
  memcpy(&rtb_TmpSignalConversionAtSFunct[63], &usSingleBatTemp[0], 120U *
         sizeof(uint16_T));
  memcpy(&rtb_TmpSignalConversionAtSFunct[183], &usSingleBatVal[0], 240U *
         sizeof(uint16_T));
  rtb_TmpSignalConversionAtSFunct[423] = BCU_VoltMaxIdx;
  rtb_TmpSignalConversionAtSFunct[424] = ThermCtrl_ACWarmGoal;
  rtb_TmpSignalConversionAtSFunct[425] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack1_o27;
  rtb_TmpSignalConversionAtSFunct[426] = ThermCtrl_Fault;
  u = 0.00999999F * (real32_T)CANFDRcvFcn_BCU_B.CANFDUnpack1_o17;
  if (u >= 0.5F) {
    i_1 = (int32_T)floorf(u + 0.5F);
  } else {
    i_1 = 0;
  }

  rtb_TmpSignalConversionAtSFunct[427] = (uint16_T)fmodf((real32_T)i_1, 65536.0F);
  rtb_TmpSignalConversionAtSFunct[428] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o26;
  rtb_TmpSignalConversionAtSFunct[429] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o27;
  rtb_TmpSignalConversionAtSFunct[430] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack2_o25;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.CANFDUnpack1_o26), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[431] = (uint16_T)(u < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[432] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o1;
  rtb_TmpSignalConversionAtSFunct[433] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o2;
  rtb_TmpSignalConversionAtSFunct[434] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o3;
  rtb_TmpSignalConversionAtSFunct[435] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o4;
  rtb_TmpSignalConversionAtSFunct[436] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o5;
  rtb_TmpSignalConversionAtSFunct[437] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o6;
  rtb_TmpSignalConversionAtSFunct[438] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o7;
  rtb_TmpSignalConversionAtSFunct[439] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o8;
  rtb_TmpSignalConversionAtSFunct[440] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o9;
  rtb_TmpSignalConversionAtSFunct[441] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o10;
  rtb_TmpSignalConversionAtSFunct[442] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o11;
  rtb_TmpSignalConversionAtSFunct[443] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o12;
  rtb_TmpSignalConversionAtSFunct[444] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o13;
  rtb_TmpSignalConversionAtSFunct[445] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o14;
  rtb_TmpSignalConversionAtSFunct[446] = (uint16_T)
    CANFDRcvFcn_BCU_B.CANFDUnpack_o15;
  rtb_TmpSignalConversionAtSFunct[447] = CANFDRcvFcn_BCU_B.BCU_RealtimePower_H;
  rtb_TmpSignalConversionAtSFunct[448] = CANFDRcvFcn_BCU_B.BCU_RealtimePower_L;
  rtb_TmpSignalConversionAtSFunct[449] = CANFDRcvFcn_BCU_B.BCU_V4_L;
  rtb_TmpSignalConversionAtSFunct[450] = CANFDRcvFcn_BCU_B.BCU_V4_H;
  rtb_TmpSignalConversionAtSFunct[451] = CFunction_o1;
  rtb_TmpSignalConversionAtSFunct[452] = CFunction_o2;
  rtb_TmpSignalConversionAtSFunct[453] = CFunction1_o1;
  rtb_TmpSignalConversionAtSFunct[454] = CFunction1_o2;
  rtb_TmpSignalConversionAtSFunct[455] = (uint16_T)port_index;
  rtb_TmpSignalConversionAtSFunct[456] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o5;
  rtb_TmpSignalConversionAtSFunct[457] = CANFDRcvFcn_BCU_B.CANFDUnpack2_o8;
  rtb_TmpSignalConversionAtSFunct[458] = BCU_TemperatureOutBox;
  rtb_TmpSignalConversionAtSFunct[459] = Chiller_Fault;
  rtb_TmpSignalConversionAtSFunct[460] = Chiller_InletPressure;
  rtb_TmpSignalConversionAtSFunct[461] = Chiller_TempInlet;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.OutWaterPressure), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[462] = (uint16_T)(u < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[463] = Chiller_TempOutlet;
  rtb_TmpSignalConversionAtSFunct[464] = Chiller_ModeFb;
  rtb_TmpSignalConversionAtSFunct[465] = CANFDRcvFcn_BCU_B.Sclience_Mode;
  u = fmodf(floorf(10.0F * CANFDRcvFcn_BCU_B.TMS_Power_Req), 65536.0F);
  rtb_TmpSignalConversionAtSFunct[466] = (uint16_T)(u < 0.0F ? (int32_T)
    (uint16_T)-(int16_T)(uint16_T)-u : (int32_T)(uint16_T)u);
  rtb_TmpSignalConversionAtSFunct[467] = CANFDRcvFcn_BCU_B.FanPWM_Rx;
  rtb_TmpSignalConversionAtSFunct[468] = CANFDRcvFcn_BCU_B.HighPress;
  rtb_TmpSignalConversionAtSFunct[469] = CANFDRcvFcn_BCU_B.LowPress;
  rtb_TmpSignalConversionAtSFunct[470] = Chiller_CompressorStatus;
  rtb_TmpSignalConversionAtSFunct[471] = Chiller_PumpStatus;
  rtb_TmpSignalConversionAtSFunct[472] = CANFDRcvFcn_BCU_B.AC_SWVersion;
  rtb_TmpSignalConversionAtSFunct[473] = (uint16_T)((uint32_T)
    (CANFDRcvFcn_BCU_B.ACP_Ver_Major << 8) | CANFDRcvFcn_BCU_B.ACP_Ver_Minor);
  rtb_TmpSignalConversionAtSFunct[474] = (uint16_T)((uint32_T)
    (CANFDRcvFcn_BCU_B.DCDC_Ver_Major << 8) | CANFDRcvFcn_BCU_B.DCDC_Ver_Minor);
  rtb_TmpSignalConversionAtSFunct[475] = BCU_TemperatureOutBox;
  rtb_TmpSignalConversionAtSFunct[476] = BCU_TemperatureInBox;
  rtb_DataTypeConversion40 = 0U;
  port_index = 0U;
  port_len = sizeof(uint32_T);
  if (port_len == 0U) {
    port_len = MAX_uint32_T;

    /* Divide by zero handler */
  } else {
    port_len = sizeof(uint32_T [105]) / port_len;
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
      i = 0U;
      while (i < CANFDRcvFcn_BCU_ConstP.portDimensions_Value[port_index]) {
        q0 = CANFDRcvFcn_BCU_ConstP.index_Value[port_index];
        qY = q0 + i;
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
        i_1 = i + 1;
        if (i + 1 > 65535) {
          i_1 = 65535;
        }

        i = (uint16_T)i_1;
      }
    }

    port_index++;
  }

  u = fmodf(floorf(CANFDRcvFcn_BCU_B.CANFDUnpack1_o16), 4.2949673E+9F);
  BCU_RealtimePower = u < 0.0F ? (uint32_T)-(int32_T)(uint32_T)-u : (uint32_T)u;
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
