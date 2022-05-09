/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ltpTrajectory_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 09-May-2022 10:45:13
 */

#ifndef _CODER_LTPTRAJECTORY_API_H
#define _CODER_LTPTRAJECTORY_API_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void ltpTrajectory(real_T DoF, real_T Tsample, real_T v_max[6], real_T
  a_max[6], real_T j_max[6], real_T q_goal[6], real_T q_0[6], real_T v_0[6],
  real_T a_0[6], real_T q_traj[6], real_T v_traj[6], real_T a_traj[6]);
extern void ltpTrajectory_api(const mxArray * const prhs[9], int32_T nlhs, const
  mxArray *plhs[3]);
extern void ltpTrajectory_atexit(void);
extern void ltpTrajectory_initialize(void);
extern void ltpTrajectory_terminate(void);
extern void ltpTrajectory_xil_shutdown(void);
extern void ltpTrajectory_xil_terminate(void);

#endif

/*
 * File trailer for _coder_ltpTrajectory_api.h
 *
 * [EOF]
 */
