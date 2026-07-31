/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: CANFDSendFcn_BCU.h
 *
 * Code generated for Simulink model 'CANFDSendFcn_BCU'.
 *
 * Model version                  : 6.2
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Fri Jul 31 16:02:35 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef CANFDSendFcn_BCU_h_
#define CANFDSendFcn_BCU_h_
#ifndef CANFDSendFcn_BCU_COMMON_INCLUDES_
#define CANFDSendFcn_BCU_COMMON_INCLUDES_
#include <math.h>
#include "rtwtypes.h"
#include "can_fd_message.h"
#endif                                 /* CANFDSendFcn_BCU_COMMON_INCLUDES_ */

#include "CANFDSendFcn_BCU_types.h"

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
  real32_T Gain;                       /* '<Root>/Gain' */
} B_CANFDSendFcn_BCU_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  int_T CANFDPack_ModeSignalID;        /* '<Root>/CAN FD Pack' */
} DW_CANFDSendFcn_BCU_T;

/* Real-time Model Data Structure */
struct tag_RTM_CANFDSendFcn_BCU_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_CANFDSendFcn_BCU_T CANFDSendFcn_BCU_B;

/* Block states (default storage) */
extern DW_CANFDSendFcn_BCU_T CANFDSendFcn_BCU_DW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern uint8_T OTA_XCPConnect;         /* '<Root>/OTA_XCPConnect' */
extern uint8_T TCU_ACMuteSet;          /* '<Root>/TCU_ACMuteSet' */
extern uint32_T TCU_BCUEINSet;         /* '<Root>/TCU_BCUEINSet' */
extern uint8_T TCU_ChargerWorkSts;     /* '<Root>/TCU_ChargerWorkSts' */
extern uint8_T TCU_ECOMode;            /* '<Root>/TCU_ECOMode' */
extern uint8_T TCU_FcnStopSet;         /* '<Root>/TCU_FcnStopSet' */
extern uint8_T TCU_HighVoltType;       /* '<Root>/TCU_HighVoltType' */
extern uint16_T TCU_HighVoltValue;     /* '<Root>/TCU_HighVoltValue' */
extern uint8_T TCU_LifeCounter;        /* '<Root>/TCU_LifeCounter' */
extern uint8_T TCU_PowerUpCmd;         /* '<Root>/TCU_PowerUpCmd' */
extern uint8_T TCU_TimeCalFlg;         /* '<Root>/TCU_TimeCalFlg' */
extern uint8_T TCU_TimeDay;            /* '<Root>/TCU_TimeDay' */
extern uint8_T TCU_TimeHour;           /* '<Root>/TCU_TimeHour' */
extern uint8_T TCU_TimeMinute;         /* '<Root>/TCU_TimeMinute' */
extern uint8_T TCU_TimeMonth;          /* '<Root>/TCU_TimeMonth' */
extern uint8_T TCU_TimeSecond;         /* '<Root>/TCU_TimeSecond' */
extern uint8_T TCU_TimeWeek;           /* '<Root>/TCU_TimeWeek' */
extern uint8_T TCU_TimeYear;           /* '<Root>/TCU_TimeYear' */
extern uint8_T TCU_BCUCapacityFlag;    /* '<Root>/TCU_BCUCapacityFlag' */
extern uint8_T TCU_CoolingFlag;        /* '<Root>/TCU_CoolingFlag' */
extern uint8_T TCU_PHYError;           /* '<Root>/TCU_PHYError' */
extern CAN_FD_MESSAGE_BUS CANSendMsg;  /* '<Root>/CANSendMsg' */

/* Model entry point functions */
extern void CANFDSendFcn_BCU_initialize(void);
extern void CANFDSendFcn_BCU_step(void);
extern void CANFDSendFcn_BCU_terminate(void);

/* Real-time Model object */
extern RT_MODEL_CANFDSendFcn_BCU_T *const CANFDSendFcn_BCU_M;

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
 * '<Root>' : 'CANFDSendFcn_BCU'
 */
#endif                                 /* CANFDSendFcn_BCU_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
