/*
 * File: CANRcvFcn_BMU.c
 *
 * Code generated for Simulink model 'CANRcvFcn_BMU'.
 *
 * Model version                  : 5.13
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu Apr  2 15:40:59 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CANRcvFcn_BMU.h"
#include "CANRcvFcn_BMU_types.h"
#include "rtwtypes.h"
#include "CANRcvFcn_BMU_private.h"

/* Exported block signals */
CAN_MESSAGE_BUS CANMsg;                /* '<Root>/CANMsg' */

/* Block signals (default storage) */
B_CANRcvFcn_BMU_T CANRcvFcn_BMU_B;

/* Block states (default storage) */
DW_CANRcvFcn_BMU_T CANRcvFcn_BMU_DW;

/* Real-time model */
static RT_MODEL_CANRcvFcn_BMU_T CANRcvFcn_BMU_M_;
RT_MODEL_CANRcvFcn_BMU_T *const CANRcvFcn_BMU_M = &CANRcvFcn_BMU_M_;

/* Exported data definition */

/* Definition for custom storage class: Default */
uint32_T DAqX_FaultCode1[15];          /* '<S2>/Data Type Conversion2' */
uint32_T DAqX_FaultCode2[15];          /* '<S2>/Data Type Conversion' */
uint16_T DAq_version[15];              /* '<S2>/Data Type Conversion1' */

/* Model step function */
void CANRcvFcn_BMU_step(void)
{
  uint32_T data_index;
  uint32_T port_index;
  uint32_T port_len;
  uint32_T qY;
  uint16_T rtb_TmpSignalConversionAtSFunct[75];
  uint16_T CFunction1;
  uint16_T CFunction10;
  uint16_T CFunction11;
  uint16_T CFunction12;
  uint16_T CFunction13;
  uint16_T CFunction14;
  uint16_T CFunction15;
  uint16_T CFunction2;
  uint16_T CFunction3;
  uint16_T CFunction4;
  uint16_T CFunction5;
  uint16_T CFunction6;
  uint16_T CFunction7;
  uint16_T CFunction8;
  uint16_T CFunction9;

  /* S-Function (scanunpack): '<S2>/0x180110E4' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E4' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985169 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E4_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E4_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function1' */
  CFunction1 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E4_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E4_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E2' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E2' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985170 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E2_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E2_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function2' */
  CFunction2 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E2_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E2_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E1' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E1' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985171 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E1_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E1_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function3' */
  CFunction3 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E1_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E1_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E5' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E5' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985172 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E5_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E5_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function4' */
  CFunction4 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E5_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E5_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E3' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E3' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985173 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E3_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E3_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function5' */
  CFunction5 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E3_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E3_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E7' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E7' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985174 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E7_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E7_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function6' */
  CFunction6 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E7_o2 << 8) |
    CANRcvFcn_BMU_B.ux180110E7_o1);

  /* S-Function (scanunpack): '<S2>/0x180110E6' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E6' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985175 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E6_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E6_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function7' */
  CFunction7 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E6_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E6_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E9' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E9' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985176 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E9_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E9_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function8' */
  CFunction8 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E9_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E9_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E8' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E8' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985177 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E8_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E8_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function9' */
  CFunction9 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E8_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E8_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E11' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E11' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985178 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E11_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E11_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function10' */
  CFunction10 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E11_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E11_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E10' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E10' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985179 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E10_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E10_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function11' */
  CFunction11 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E10_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E10_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E13' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E13' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985180 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E13_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E13_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function12' */
  CFunction12 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E13_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E13_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E12' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E12' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985181 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E12_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E12_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function13' */
  CFunction13 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E12_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E12_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E15' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E15' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985182 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E15_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E15_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function14' */
  CFunction14 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E15_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E15_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E14' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E14' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402985183 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[0]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E14_o1 = result;
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
                  tempValue = tempValue | (uint8_T)(CANMsg.Data[1]);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint8_T) (unpackedValue);
            }

            {
              uint8_T result = (uint8_T) outValue;
              CANRcvFcn_BMU_B.ux180110E14_o2 = result;
            }
          }
        }
      }
    }
  }

  /* CFunction: '<S2>/C Function15' */
  CFunction15 = (uint16_T)((uint32_T)(CANRcvFcn_BMU_B.ux180110E14_o1 << 8) |
    CANRcvFcn_BMU_B.ux180110E14_o2);

  /* S-Function (scanunpack): '<S2>/0x180110E16' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E16' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919633 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E16_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E16_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E17' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E17' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919634 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E17_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E17_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E18' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E18' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919635 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E18_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E18_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E19' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E19' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919636 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E19_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E19_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E20' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E20' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919637 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E20_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E20_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E21' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E21' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919638 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E21_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E21_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E22' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E22' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919639 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E22_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E22_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E23' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E23' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919640 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E23_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E23_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E24' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E24' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919641 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E24_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E24_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E25' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E25' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919642 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E25_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E25_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E26' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E26' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919643 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E26_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E26_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E27' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E27' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919644 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E27_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E27_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E28' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E28' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919645 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E28_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E28_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E29' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E29' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919646 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E29_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E29_o2 = result;
            }
          }
        }
      }
    }
  }

  /* S-Function (scanunpack): '<S2>/0x180110E30' incorporates:
   *  Inport: '<Root>/CANMsg'
   */
  {
    /* S-Function (scanunpack): '<S2>/0x180110E30' */
    if ((8 == CANMsg.Length) && (CANMsg.ID != INVALID_CAN_ID) ) {
      if ((402919647 == CANMsg.ID) && (1U == CANMsg.Extended) ) {
        {
          /* --------------- START Unpacking signal 0 ------------------
           *  startBit                = 24
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[3]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[2]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[1]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[0]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E30_o1 = result;
            }
          }

          /* --------------- START Unpacking signal 1 ------------------
           *  startBit                = 56
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
                  tempValue = tempValue | (uint32_T)(CANMsg.Data[7]);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[6]) <<
                    8);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[5]) <<
                    16);
                  tempValue = tempValue | (uint32_T)((uint32_T)(CANMsg.Data[4]) <<
                    24);
                }

                unpackedValue = tempValue;
              }

              outValue = (uint32_T) (unpackedValue);
            }

            {
              uint32_T result = (uint32_T) outValue;
              CANRcvFcn_BMU_B.ux180110E30_o2 = result;
            }
          }
        }
      }
    }
  }

  /* SignalConversion generated from: '<S1>/ SFunction ' incorporates:
   *  CFunction: '<S2>/C Function16'
   *  CFunction: '<S2>/C Function17'
   *  CFunction: '<S2>/C Function18'
   *  CFunction: '<S2>/C Function19'
   *  CFunction: '<S2>/C Function20'
   *  CFunction: '<S2>/C Function21'
   *  CFunction: '<S2>/C Function22'
   *  CFunction: '<S2>/C Function23'
   *  CFunction: '<S2>/C Function24'
   *  CFunction: '<S2>/C Function25'
   *  CFunction: '<S2>/C Function26'
   *  CFunction: '<S2>/C Function27'
   *  CFunction: '<S2>/C Function28'
   *  CFunction: '<S2>/C Function29'
   *  CFunction: '<S2>/C Function30'
   *  CFunction: '<S2>/C Function31'
   *  CFunction: '<S2>/C Function32'
   *  CFunction: '<S2>/C Function33'
   *  CFunction: '<S2>/C Function34'
   *  CFunction: '<S2>/C Function35'
   *  CFunction: '<S2>/C Function36'
   *  CFunction: '<S2>/C Function37'
   *  CFunction: '<S2>/C Function38'
   *  CFunction: '<S2>/C Function39'
   *  CFunction: '<S2>/C Function40'
   *  CFunction: '<S2>/C Function41'
   *  CFunction: '<S2>/C Function42'
   *  CFunction: '<S2>/C Function43'
   *  CFunction: '<S2>/C Function44'
   *  CFunction: '<S2>/C Function45'
   *  Chart: '<Root>/Chart1'
   */
  rtb_TmpSignalConversionAtSFunct[0] = CFunction1;
  rtb_TmpSignalConversionAtSFunct[1] = CFunction2;
  rtb_TmpSignalConversionAtSFunct[2] = CFunction3;
  rtb_TmpSignalConversionAtSFunct[3] = CFunction4;
  rtb_TmpSignalConversionAtSFunct[4] = CFunction5;
  rtb_TmpSignalConversionAtSFunct[5] = CFunction6;
  rtb_TmpSignalConversionAtSFunct[6] = CFunction7;
  rtb_TmpSignalConversionAtSFunct[7] = CFunction8;
  rtb_TmpSignalConversionAtSFunct[8] = CFunction9;
  rtb_TmpSignalConversionAtSFunct[9] = CFunction10;
  rtb_TmpSignalConversionAtSFunct[10] = CFunction11;
  rtb_TmpSignalConversionAtSFunct[11] = CFunction12;
  rtb_TmpSignalConversionAtSFunct[12] = CFunction13;
  rtb_TmpSignalConversionAtSFunct[13] = CFunction14;
  rtb_TmpSignalConversionAtSFunct[14] = CFunction15;
  rtb_TmpSignalConversionAtSFunct[15] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E16_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[16] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E16_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[17] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E16_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[18] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E16_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[19] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E17_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[20] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E17_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[21] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E17_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[22] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E17_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[23] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E18_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[24] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E18_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[25] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E18_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[26] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E18_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[27] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E19_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[28] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E19_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[29] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E19_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[30] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E19_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[31] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E20_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[32] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E20_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[33] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E20_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[34] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E20_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[35] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E21_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[36] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E21_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[37] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E21_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[38] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E21_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[39] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E22_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[40] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E22_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[41] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E22_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[42] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E22_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[43] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E23_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[44] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E23_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[45] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E23_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[46] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E23_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[47] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E24_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[48] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E24_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[49] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E24_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[50] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E24_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[51] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E25_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[52] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E25_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[53] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E25_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[54] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E25_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[55] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E26_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[56] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E26_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[57] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E26_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[58] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E26_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[59] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E27_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[60] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E27_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[61] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E27_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[62] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E27_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[63] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E28_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[64] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E28_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[65] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E28_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[66] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E28_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[67] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E29_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[68] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E29_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[69] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E29_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[70] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E29_o2 & 65535U);
  rtb_TmpSignalConversionAtSFunct[71] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E30_o1 >> 16);
  rtb_TmpSignalConversionAtSFunct[72] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E30_o1 & 65535U);
  rtb_TmpSignalConversionAtSFunct[73] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E30_o2 >> 16);
  rtb_TmpSignalConversionAtSFunct[74] = (uint16_T)
    (CANRcvFcn_BMU_B.ux180110E30_o2 & 65535U);

  /* Chart: '<Root>/Chart1' incorporates:
   *  Constant: '<Root>/index'
   *  Outport: '<Root>/modbusBuff'
   */
  data_index = 0U;
  port_index = 0U;
  port_len = sizeof(uint32_T);
  if (port_len == 0U) {
    port_len = MAX_uint32_T;

    /* Divide by zero handler */
  } else {
    port_len = sizeof(uint32_T [75]) / port_len;
  }

  while (port_index < port_len) {
    modbusBuff[CANRcvFcn_BMU_ConstP.index_Value[port_index]] =
      rtb_TmpSignalConversionAtSFunct[data_index];
    qY = data_index + /*MW:OvSatOk*/ 1U;
    if (data_index + 1U < data_index) {
      qY = MAX_uint32_T;
    }

    data_index = qY;
    port_index++;
  }

  /* DataTypeConversion: '<S2>/Data Type Conversion' */
  DAqX_FaultCode2[0] = CANRcvFcn_BMU_B.ux180110E16_o2;
  DAqX_FaultCode2[1] = CANRcvFcn_BMU_B.ux180110E17_o2;
  DAqX_FaultCode2[2] = CANRcvFcn_BMU_B.ux180110E18_o2;
  DAqX_FaultCode2[3] = CANRcvFcn_BMU_B.ux180110E19_o2;
  DAqX_FaultCode2[4] = CANRcvFcn_BMU_B.ux180110E20_o2;
  DAqX_FaultCode2[5] = CANRcvFcn_BMU_B.ux180110E21_o2;
  DAqX_FaultCode2[6] = CANRcvFcn_BMU_B.ux180110E22_o2;
  DAqX_FaultCode2[7] = CANRcvFcn_BMU_B.ux180110E23_o2;
  DAqX_FaultCode2[8] = CANRcvFcn_BMU_B.ux180110E24_o2;
  DAqX_FaultCode2[9] = CANRcvFcn_BMU_B.ux180110E25_o2;
  DAqX_FaultCode2[10] = CANRcvFcn_BMU_B.ux180110E26_o2;
  DAqX_FaultCode2[11] = CANRcvFcn_BMU_B.ux180110E27_o2;
  DAqX_FaultCode2[12] = CANRcvFcn_BMU_B.ux180110E28_o2;
  DAqX_FaultCode2[13] = CANRcvFcn_BMU_B.ux180110E29_o2;
  DAqX_FaultCode2[14] = CANRcvFcn_BMU_B.ux180110E30_o2;

  /* DataTypeConversion: '<S2>/Data Type Conversion2' */
  DAqX_FaultCode1[0] = CANRcvFcn_BMU_B.ux180110E16_o1;
  DAqX_FaultCode1[1] = CANRcvFcn_BMU_B.ux180110E17_o1;
  DAqX_FaultCode1[2] = CANRcvFcn_BMU_B.ux180110E18_o1;
  DAqX_FaultCode1[3] = CANRcvFcn_BMU_B.ux180110E19_o1;
  DAqX_FaultCode1[4] = CANRcvFcn_BMU_B.ux180110E20_o1;
  DAqX_FaultCode1[5] = CANRcvFcn_BMU_B.ux180110E21_o1;
  DAqX_FaultCode1[6] = CANRcvFcn_BMU_B.ux180110E22_o1;
  DAqX_FaultCode1[7] = CANRcvFcn_BMU_B.ux180110E23_o1;
  DAqX_FaultCode1[8] = CANRcvFcn_BMU_B.ux180110E24_o1;
  DAqX_FaultCode1[9] = CANRcvFcn_BMU_B.ux180110E25_o1;
  DAqX_FaultCode1[10] = CANRcvFcn_BMU_B.ux180110E26_o1;
  DAqX_FaultCode1[11] = CANRcvFcn_BMU_B.ux180110E27_o1;
  DAqX_FaultCode1[12] = CANRcvFcn_BMU_B.ux180110E28_o1;
  DAqX_FaultCode1[13] = CANRcvFcn_BMU_B.ux180110E29_o1;
  DAqX_FaultCode1[14] = CANRcvFcn_BMU_B.ux180110E30_o1;

  /* DataTypeConversion: '<S2>/Data Type Conversion1' */
  DAq_version[0] = CFunction1;
  DAq_version[1] = CFunction2;
  DAq_version[2] = CFunction3;
  DAq_version[3] = CFunction4;
  DAq_version[4] = CFunction5;
  DAq_version[5] = CFunction6;
  DAq_version[6] = CFunction7;
  DAq_version[7] = CFunction8;
  DAq_version[8] = CFunction9;
  DAq_version[9] = CFunction10;
  DAq_version[10] = CFunction11;
  DAq_version[11] = CFunction12;
  DAq_version[12] = CFunction13;
  DAq_version[13] = CFunction14;
  DAq_version[14] = CFunction15;
}

/* Model initialize function */
void CANRcvFcn_BMU_initialize(void)
{
  /* Start for S-Function (scanunpack): '<S2>/0x180110E4' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E4 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E2' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E2 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E1' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E1 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E5' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E5 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E3' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E3 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E7' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E7 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E6' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E6 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E9' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E9 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E8' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E8 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E11' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E11 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E10' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E10 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E13' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E13 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E12' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E12 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E15' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E15 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E14' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E14 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E16' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E16 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E17' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E17 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E18' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E18 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E19' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E19 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E20' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E20 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E21' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E21 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E22' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E22 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E23' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E23 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E24' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E24 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E25' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E25 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E26' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E26 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E27' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E27 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E28' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E28 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E29' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E29 -----------------*/

  /* Start for S-Function (scanunpack): '<S2>/0x180110E30' incorporates:
   *  Inport: '<Root>/CANMsg'
   */

  /*-----------S-Function Block: <S2>/0x180110E30 -----------------*/
}

/* Model terminate function */
void CANRcvFcn_BMU_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
