/*
 * File: CANFDRcvFcn_BCU.h
 *
 * Code generated for Simulink model 'CANFDRcvFcn_BCU'.
 *
 * Model version                  : 5.276
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu May 28 14:44:39 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef CANFDRcvFcn_BCU_h_
#define CANFDRcvFcn_BCU_h_
#ifndef CANFDRcvFcn_BCU_COMMON_INCLUDES_
#define CANFDRcvFcn_BCU_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "can_fd_message.h"
#endif                                 /* CANFDRcvFcn_BCU_COMMON_INCLUDES_ */

#include "CANFDRcvFcn_BCU_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* user code (top of header file) */
#include "modbustcp_task.h"

/* Block signals (default storage) */
typedef struct {
  real_T CANFDUnpack2_o11;             /* '<S1>/CAN FD Unpack2' */
  real_T CANFDUnpack2_o14;             /* '<S1>/CAN FD Unpack2' */
  real_T CANFDUnpack2_o17;             /* '<S1>/CAN FD Unpack2' */
  real_T CANFDUnpack2_o18;             /* '<S1>/CAN FD Unpack2' */
  real_T CANFDUnpack2_o23;             /* '<S1>/CAN FD Unpack2' */
  real32_T CANFDUnpack1_o5;            /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o6;            /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o7;            /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o8;            /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o19;           /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o24;           /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o25;           /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o26;           /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o27;           /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o28;           /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack1_o29;           /* '<S1>/CAN FD Unpack1' */
  real32_T CANFDUnpack2_o12;           /* '<S1>/CAN FD Unpack2' */
  real32_T CANFDUnpack2_o13;           /* '<S1>/CAN FD Unpack2' */
  real32_T CANFDUnpack2_o15;           /* '<S1>/CAN FD Unpack2' */
  real32_T CANFDUnpack2_o16;           /* '<S1>/CAN FD Unpack2' */
  real32_T InWaterPressure;            /* '<S1>/CAN FD Unpack3' */
  real32_T OutWaterPressure;           /* '<S1>/CAN FD Unpack3' */
  real32_T TMS_Power_Req;              /* '<S1>/CAN FD Unpack3' */
  uint16_T CANFDUnpack1_o13;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o14;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o15;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o16;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o17;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o18;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o20;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o21;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o22;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o23;           /* '<S1>/CAN FD Unpack1' */
  uint16_T CANFDUnpack1_o32;           /* '<S1>/CAN FD Unpack1' */
  uint16_T BCU_Curr2_H;                /* '<S1>/U32_to_Folat_Curr' */
  uint16_T BCU_Curr2_L;                /* '<S1>/U32_to_Folat_Curr' */
  uint16_T CANFDUnpack2_o3;            /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o4;            /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o5;            /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o6;            /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o7;            /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o8;            /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o9;            /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o10;           /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o20;           /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o21;           /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o22;           /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o37;           /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o38;           /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack2_o39;           /* '<S1>/CAN FD Unpack2' */
  uint16_T CANFDUnpack_o24;            /* '<S1>/CAN FD Unpack' */
  uint16_T CANFDUnpack_o25;            /* '<S1>/CAN FD Unpack' */
  uint16_T CANFDUnpack5_o1;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o2;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o3;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o4;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o5;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o6;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o7;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o8;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o9;            /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o10;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o11;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o12;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o13;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o14;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o15;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o16;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o17;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o18;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o19;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o20;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o21;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o22;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o23;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o24;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o25;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o26;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o27;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o28;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o29;           /* '<S1>/CAN FD Unpack5' */
  uint16_T CANFDUnpack5_o30;           /* '<S1>/CAN FD Unpack5' */
  uint16_T BCU_RealtimePower_H;        /* '<S1>/U32_to_F32_Power' */
  uint16_T BCU_RealtimePower_L;        /* '<S1>/U32_to_F32_Power' */
  uint16_T BCU_V4_L;                   /* '<S1>/U32_to_F32_V3' */
  uint16_T BCU_V4_H;                   /* '<S1>/U32_to_F32_V3' */
  uint16_T Sclience_Mode;              /* '<S1>/CAN FD Unpack3' */
  uint16_T FanPWM_Rx;                  /* '<S1>/CAN FD Unpack6' */
  uint16_T HighPress;                  /* '<S1>/CAN FD Unpack6' */
  uint16_T LowPress;                   /* '<S1>/CAN FD Unpack6' */
  uint16_T AC_SWVersion;               /* '<S1>/CAN FD Unpack8' */
  uint16_T ACP_Ver_Major;              /* '<S1>/CAN FD Unpack9' */
  uint16_T ACP_Ver_Minor;              /* '<S1>/CAN FD Unpack9' */
  uint16_T DCDC_Ver_Major;             /* '<S1>/CAN FD Unpack10' */
  uint16_T DCDC_Ver_Minor;             /* '<S1>/CAN FD Unpack10' */
  int16_T CANFDUnpack1_o30;            /* '<S1>/CAN FD Unpack1' */
  int16_T CANFDUnpack1_o31;            /* '<S1>/CAN FD Unpack1' */
  int16_T CANFDUnpack1_o33;            /* '<S1>/CAN FD Unpack1' */
  int16_T CANFDUnpack2_o19;            /* '<S1>/CAN FD Unpack2' */
  int16_T CANFDUnpack2_o24;            /* '<S1>/CAN FD Unpack2' */
  int16_T CANFDUnpack2_o25;            /* '<S1>/CAN FD Unpack2' */
  int16_T CANFDUnpack2_o26;            /* '<S1>/CAN FD Unpack2' */
  int16_T CANFDUnpack2_o27;            /* '<S1>/CAN FD Unpack2' */
  int16_T CANFDUnpack2_o30;            /* '<S1>/CAN FD Unpack2' */
  int16_T CANFDUnpack2_o33;            /* '<S1>/CAN FD Unpack2' */
  int16_T CANFDUnpack2_o34;            /* '<S1>/CAN FD Unpack2' */
  int16_T CANFDUnpack_o1;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o2;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o3;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o4;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o5;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o6;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o7;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o8;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o9;              /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o10;             /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o11;             /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o12;             /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o13;             /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o14;             /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack_o15;             /* '<S1>/CAN FD Unpack' */
  int16_T CANFDUnpack4_o1;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o2;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o3;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o4;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o5;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o6;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o7;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o8;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o9;             /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o10;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o11;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o12;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o13;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o14;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o15;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o16;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o17;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o18;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o19;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o20;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o21;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o22;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o23;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o24;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o25;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o26;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o27;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o28;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o29;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o30;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o31;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o32;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o33;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o34;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o35;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o36;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o37;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o38;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o39;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o40;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o41;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o42;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o43;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o44;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o45;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o46;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o47;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o48;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o49;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o50;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o51;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o52;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o53;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o54;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o55;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o56;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o57;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o58;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o59;            /* '<S1>/CAN FD Unpack4' */
  int16_T CANFDUnpack4_o60;            /* '<S1>/CAN FD Unpack4' */
  int16_T AmbientTemp;                 /* '<S1>/CAN FD Unpack3' */
  int16_T InWaterTemp;                 /* '<S1>/CAN FD Unpack3' */
  int16_T OutWaterTemp;                /* '<S1>/CAN FD Unpack3' */
  uint8_T CANFDUnpack2_o28;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o29;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o31;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o32;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o35;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o36;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o40;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o41;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o43;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack2_o44;            /* '<S1>/CAN FD Unpack2' */
  uint8_T CANFDUnpack4_o61;            /* '<S1>/CAN FD Unpack4' */
  uint8_T CANFDUnpack5_o31;            /* '<S1>/CAN FD Unpack5' */
  uint8_T DTCLevel;                    /* '<S1>/CAN FD Unpack3' */
  uint8_T K1_State;                    /* '<S1>/CAN FD Unpack3' */
  uint8_T K2_State;                    /* '<S1>/CAN FD Unpack3' */
  uint8_T PreHeating_FB;               /* '<S1>/CAN FD Unpack3' */
  uint8_T ACP_RPM;                     /* '<S1>/CAN FD Unpack7' */
} B_CANFDRcvFcn_BCU_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T U32_to_Folat_Curr_float_value;/* '<S1>/U32_to_Folat_Curr' */
  real32_T U32_to_F32_Power_float_value;/* '<S1>/U32_to_F32_Power' */
  real32_T U32_to_F32_V3_float_value;  /* '<S1>/U32_to_F32_V3' */
  uint32_T U32_to_Folat_Curr_float_bits;/* '<S1>/U32_to_Folat_Curr' */
  uint32_T U32_to_F32_Power_float_bits;/* '<S1>/U32_to_F32_Power' */
  uint32_T U32_to_F32_V3_float_bits;   /* '<S1>/U32_to_F32_V3' */
  int_T CANFDUnpack1_ModeSignalID;     /* '<S1>/CAN FD Unpack1' */
  int_T CANFDUnpack1_StatusPortID;     /* '<S1>/CAN FD Unpack1' */
  int_T CANFDUnpack2_ModeSignalID;     /* '<S1>/CAN FD Unpack2' */
  int_T CANFDUnpack2_StatusPortID;     /* '<S1>/CAN FD Unpack2' */
  int_T CANFDUnpack_ModeSignalID;      /* '<S1>/CAN FD Unpack' */
  int_T CANFDUnpack_StatusPortID;      /* '<S1>/CAN FD Unpack' */
  int_T CANFDUnpack4_ModeSignalID;     /* '<S1>/CAN FD Unpack4' */
  int_T CANFDUnpack4_StatusPortID;     /* '<S1>/CAN FD Unpack4' */
  int_T CANFDUnpack5_ModeSignalID;     /* '<S1>/CAN FD Unpack5' */
  int_T CANFDUnpack5_StatusPortID;     /* '<S1>/CAN FD Unpack5' */
  int_T CANFDUnpack3_ModeSignalID;     /* '<S1>/CAN FD Unpack3' */
  int_T CANFDUnpack3_StatusPortID;     /* '<S1>/CAN FD Unpack3' */
  int_T CANFDUnpack6_ModeSignalID;     /* '<S1>/CAN FD Unpack6' */
  int_T CANFDUnpack6_StatusPortID;     /* '<S1>/CAN FD Unpack6' */
  int_T CANFDUnpack7_ModeSignalID;     /* '<S1>/CAN FD Unpack7' */
  int_T CANFDUnpack7_StatusPortID;     /* '<S1>/CAN FD Unpack7' */
  int_T CANFDUnpack8_ModeSignalID;     /* '<S1>/CAN FD Unpack8' */
  int_T CANFDUnpack8_StatusPortID;     /* '<S1>/CAN FD Unpack8' */
  int_T CANFDUnpack9_ModeSignalID;     /* '<S1>/CAN FD Unpack9' */
  int_T CANFDUnpack9_StatusPortID;     /* '<S1>/CAN FD Unpack9' */
  int_T CANFDUnpack10_ModeSignalID;    /* '<S1>/CAN FD Unpack10' */
  int_T CANFDUnpack10_StatusPortID;    /* '<S1>/CAN FD Unpack10' */
  uint16_T tmp[240];                   /* '<S4>/MATLAB Function2' */
  uint16_T tmp_a[120];                 /* '<S3>/MATLAB Function' */
  uint8_T U32_to_Folat_Curr_byteA;     /* '<S1>/U32_to_Folat_Curr' */
  uint8_T U32_to_Folat_Curr_byteB;     /* '<S1>/U32_to_Folat_Curr' */
  uint8_T U32_to_Folat_Curr_byteC;     /* '<S1>/U32_to_Folat_Curr' */
  uint8_T U32_to_Folat_Curr_byteD;     /* '<S1>/U32_to_Folat_Curr' */
  uint8_T U32_to_F32_Power_byteA;      /* '<S1>/U32_to_F32_Power' */
  uint8_T U32_to_F32_Power_byteB;      /* '<S1>/U32_to_F32_Power' */
  uint8_T U32_to_F32_Power_byteC;      /* '<S1>/U32_to_F32_Power' */
  uint8_T U32_to_F32_Power_byteD;      /* '<S1>/U32_to_F32_Power' */
  uint8_T U32_to_F32_V3_byteA;         /* '<S1>/U32_to_F32_V3' */
  uint8_T U32_to_F32_V3_byteB;         /* '<S1>/U32_to_F32_V3' */
  uint8_T U32_to_F32_V3_byteC;         /* '<S1>/U32_to_F32_V3' */
  uint8_T U32_to_F32_V3_byteD;         /* '<S1>/U32_to_F32_V3' */
} DW_CANFDRcvFcn_BCU_T;

/* Invariant block signals (default storage) */
typedef struct {
  const boolean_T DataTypeConversion27;/* '<S5>/Data Type Conversion27' */
  const boolean_T DataTypeConversion28;/* '<S5>/Data Type Conversion28' */
  const boolean_T DataTypeConversion29;/* '<S5>/Data Type Conversion29' */
  const boolean_T DataTypeConversion30;/* '<S5>/Data Type Conversion30' */
  const boolean_T DataTypeConversion38;/* '<S5>/Data Type Conversion38' */
} ConstB_CANFDRcvFcn_BCU_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: index_Value
   * Referenced by: '<Root>/index'
   */
  uint32_T index_Value[106];

  /* Computed Parameter: portDimensions_Value
   * Referenced by: '<Root>/portDimensions'
   */
  uint32_T portDimensions_Value[106];
} ConstP_CANFDRcvFcn_BCU_T;

/* Real-time Model Data Structure */
struct tag_RTM_CANFDRcvFcn_BCU_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_CANFDRcvFcn_BCU_T CANFDRcvFcn_BCU_B;

/* Block states (default storage) */
extern DW_CANFDRcvFcn_BCU_T CANFDRcvFcn_BCU_DW;
extern const ConstB_CANFDRcvFcn_BCU_T CANFDRcvFcn_BCU_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_CANFDRcvFcn_BCU_T CANFDRcvFcn_BCU_ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern CAN_FD_MESSAGE_BUS CANFDRcvMsg; /* '<Root>/CANFDRcvMsg' */

/* Model entry point functions */
extern void CANFDRcvFcn_BCU_initialize(void);
extern void CANFDRcvFcn_BCU_step(void);
extern void CANFDRcvFcn_BCU_terminate(void);

/* Real-time Model object */
extern RT_MODEL_CANFDRcvFcn_BCU_T *const CANFDRcvFcn_BCU_M;

/* Exported data declaration */

/* Declaration for custom storage class: Default */
extern uint16_T BCU_BCUVersion;        /* '<S1>/C Function14' */
extern uint8_T BCU_BCUVersion_H;       /* '<S1>/CAN FD Unpack2' */
extern uint8_T BCU_BCUVersion_L;       /* '<S1>/CAN FD Unpack2' */
extern uint16_T BCU_BatteryStatus;     /* '<S1>/CAN FD Unpack1' */
extern uint16_T BCU_CapacityFlag;      /* '<S1>/CAN FD Unpack1' */
extern uint16_T BCU_CoolingFlag;       /* '<S1>/CAN FD Unpack1' */
extern uint16_T BCU_Curr;              /* '<S1>/Data Type Conversion46' */
extern uint32_T BCU_EngryAccumulateChrg;/* '<S1>/Data Type Conversion18' */
extern uint32_T BCU_EngryAccumulateDisChrg;/* '<S1>/Data Type Conversion20' */
extern uint16_T BCU_FasCOMaxIdx;       /* '<S1>/CAN FD Unpack' */
extern uint16_T BCU_FasCOMaxValue;     /* '<S1>/CAN FD Unpack' */
extern uint16_T BCU_FasH2MaxIdx;       /* '<S1>/CAN FD Unpack' */
extern uint16_T BCU_FasH2MaxValue;     /* '<S1>/CAN FD Unpack' */
extern uint16_T BCU_FasLightMaxIdx;    /* '<S1>/CAN FD Unpack' */
extern uint16_T BCU_FasLightMaxValue;  /* '<S1>/CAN FD Unpack' */
extern uint16_T BCU_FasPressMaxIdx;    /* '<S1>/CAN FD Unpack' */
extern uint16_T BCU_FasPressMaxValue;  /* '<S1>/CAN FD Unpack' */
extern uint32_T BCU_FaultInfoLv1;      /* '<S1>/CAN FD Unpack1' */
extern uint32_T BCU_FaultInfoLv2;      /* '<S1>/CAN FD Unpack1' */
extern uint32_T BCU_FaultInfoLv3;      /* '<S1>/CAN FD Unpack1' */
extern uint32_T BCU_FaultInfoLv4;      /* '<S1>/CAN FD Unpack1' */
extern uint32_T BCU_RealtimePower;     /* '<S1>/Data Type Conversion6' */
extern uint16_T BCU_SOC;               /* '<S1>/Data Type Conversion35' */
extern uint16_T BCU_SystemWorkMode;    /* '<S1>/CAN FD Unpack1' */
extern uint16_T BCU_TempMaxIdx;        /* '<S8>/Data Type Conversion44' */
extern uint16_T BCU_TempMaxValue;      /* '<S1>/Data Type Conversion58' */
extern uint16_T BCU_TempMinIdx;        /* '<S9>/Data Type Conversion45' */
extern uint16_T BCU_TempMinValue;      /* '<S1>/Data Type Conversion59' */
extern uint16_T BCU_TemperatureInBox;  /* '<S1>/Data Type Conversion14' */
extern uint16_T BCU_TemperatureOutBox; /* '<S1>/Data Type Conversion38' */
extern uint8_T BCU_TimeDay;            /* '<S1>/CAN FD Unpack' */
extern uint8_T BCU_TimeHour;           /* '<S1>/CAN FD Unpack' */
extern uint8_T BCU_TimeMinute;         /* '<S1>/CAN FD Unpack' */
extern uint8_T BCU_TimeMonth;          /* '<S1>/CAN FD Unpack' */
extern uint8_T BCU_TimeSencond;        /* '<S1>/CAN FD Unpack' */
extern uint8_T BCU_TimeWeek;           /* '<S1>/CAN FD Unpack' */
extern uint8_T BCU_TimeYear;           /* '<S1>/CAN FD Unpack' */
extern uint16_T BCU_VoltMaxCellValue;  /* '<S1>/CAN FD Unpack2' */
extern uint16_T BCU_VoltMaxIdx;        /* '<S11>/Data Type Conversion41' */
extern uint16_T BCU_VoltMinCellValue;  /* '<S1>/CAN FD Unpack2' */
extern uint16_T BCU_VoltMinIdx;        /* '<S12>/Data Type Conversion42' */
extern uint16_T Chiller_CompressorStatus;/* '<S1>/Data Type Conversion42' */
extern uint16_T Chiller_Fault;         /* '<S1>/CAN FD Unpack3' */
extern uint16_T Chiller_InletPressure; /* '<S1>/Data Type Conversion34' */
extern uint16_T Chiller_ModeFb;        /* '<S1>/CAN FD Unpack3' */
extern uint16_T Chiller_PumpStatus;    /* '<S1>/CAN FD Unpack7' */
extern uint16_T Chiller_TempInlet;     /* '<S1>/Data Type Conversion45' */
extern uint16_T Chiller_TempOutlet;    /* '<S1>/Data Type Conversion60' */
extern uint16_T ThermCtrl_ACWarmGoal;  /* '<S1>/Data Type Conversion11' */
extern uint16_T ThermCtrl_Fault;       /* '<S5>/C Function11' */
extern uint16_T usSingleBatTemp[120];  /* '<S3>/MATLAB Function' */
extern uint16_T usSingleBatVal[240];   /* '<S4>/MATLAB Function2' */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'CANFDRcvFcn_BCU'
 * '<S1>'   : 'CANFDRcvFcn_BCU/BCU_Info'
 * '<S2>'   : 'CANFDRcvFcn_BCU/Chart1'
 * '<S3>'   : 'CANFDRcvFcn_BCU/BCU_Info/AssginTemp'
 * '<S4>'   : 'CANFDRcvFcn_BCU/BCU_Info/AssginVolt'
 * '<S5>'   : 'CANFDRcvFcn_BCU/BCU_Info/GetChillerFault'
 * '<S6>'   : 'CANFDRcvFcn_BCU/BCU_Info/Get_CompressorStatus'
 * '<S7>'   : 'CANFDRcvFcn_BCU/BCU_Info/Get_TempChngbatIdx'
 * '<S8>'   : 'CANFDRcvFcn_BCU/BCU_Info/Get_TempMaxIdx'
 * '<S9>'   : 'CANFDRcvFcn_BCU/BCU_Info/Get_TempMinIdx'
 * '<S10>'  : 'CANFDRcvFcn_BCU/BCU_Info/Get_VoltCellChngeIdx'
 * '<S11>'  : 'CANFDRcvFcn_BCU/BCU_Info/Get_VoltMaxIdx'
 * '<S12>'  : 'CANFDRcvFcn_BCU/BCU_Info/Get_VoltMinIdx'
 * '<S13>'  : 'CANFDRcvFcn_BCU/BCU_Info/SocProcess'
 * '<S14>'  : 'CANFDRcvFcn_BCU/BCU_Info/SocProcess1'
 * '<S15>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement'
 * '<S16>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement1'
 * '<S17>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement10'
 * '<S18>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement11'
 * '<S19>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement12'
 * '<S20>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement13'
 * '<S21>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement14'
 * '<S22>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement15'
 * '<S23>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement16'
 * '<S24>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement17'
 * '<S25>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement18'
 * '<S26>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement19'
 * '<S27>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement2'
 * '<S28>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement20'
 * '<S29>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement21'
 * '<S30>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement22'
 * '<S31>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement23'
 * '<S32>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement24'
 * '<S33>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement25'
 * '<S34>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement26'
 * '<S35>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement27'
 * '<S36>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement28'
 * '<S37>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement29'
 * '<S38>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement3'
 * '<S39>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement30'
 * '<S40>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement31'
 * '<S41>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement32'
 * '<S42>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement33'
 * '<S43>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement34'
 * '<S44>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement35'
 * '<S45>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement36'
 * '<S46>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement37'
 * '<S47>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement38'
 * '<S48>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement39'
 * '<S49>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement4'
 * '<S50>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement40'
 * '<S51>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement41'
 * '<S52>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement42'
 * '<S53>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement43'
 * '<S54>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement44'
 * '<S55>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement45'
 * '<S56>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement46'
 * '<S57>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement47'
 * '<S58>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement48'
 * '<S59>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement49'
 * '<S60>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement5'
 * '<S61>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement50'
 * '<S62>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement51'
 * '<S63>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement6'
 * '<S64>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement7'
 * '<S65>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement8'
 * '<S66>'  : 'CANFDRcvFcn_BCU/BCU_Info/Write Measurement9'
 * '<S67>'  : 'CANFDRcvFcn_BCU/BCU_Info/AssginTemp/MATLAB Function'
 * '<S68>'  : 'CANFDRcvFcn_BCU/BCU_Info/AssginVolt/MATLAB Function2'
 * '<S69>'  : 'CANFDRcvFcn_BCU/BCU_Info/Get_CompressorStatus/Compare To Constant'
 */
#endif                                 /* CANFDRcvFcn_BCU_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
