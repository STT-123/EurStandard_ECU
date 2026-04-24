/*
 * File: CANFDRcvFcn_BCU_private.h
 *
 * Code generated for Simulink model 'CANFDRcvFcn_BCU'.
 *
 * Model version                  : 5.263
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Fri Apr 24 16:02:27 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: NXP->Cortex-M4
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef CANFDRcvFcn_BCU_private_h_
#define CANFDRcvFcn_BCU_private_h_
#include "rtwtypes.h"
#include "CANFDRcvFcn_BCU_types.h"
#include "CANFDRcvFcn_BCU.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

// #if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
// #error Code was generated for compiler with different sized ulong/long. \
// Consider adjusting Test hardware word size settings on the \
// Hardware Implementation pane to match your compiler word sizes as \
// defined in limits.h of the compiler. Alternatively, you can \
// select the Test hardware is the same as production hardware option and \
// select the Enable portable word sizes option on the Code Generation > \
// Verification pane for ERT based targets, which will disable the \
// preprocessor word size checks.
// #endif

extern void CANFDRcvFcn_BCU_SocProcess(uint16_T rtu_soc, uint16_T
  *rty_soc_process);

/* Exported data declaration */

/* Declaration for custom storage class: ImportFromFile */
// extern uint16_T modbusBuff[16384];     /* '<Root>/modbusBuff' */

#endif                                 /* CANFDRcvFcn_BCU_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
