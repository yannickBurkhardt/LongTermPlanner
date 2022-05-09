/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ltpTrajectory_api.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 09-May-2022 10:45:13
 */

/* Include Files */
#include "_coder_ltpTrajectory_api.h"
#include "_coder_ltpTrajectory_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131594U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "ltpTrajectory",                     /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *v_max,
  const char_T *identifier))[6];
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *DoF, const
  char_T *identifier);
static const mxArray *emlrt_marshallOut(const real_T u[6]);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *v_max
 *                const char_T *identifier
 * Return Type  : real_T (*)[6]
 */
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *v_max,
  const char_T *identifier))[6]
{
  real_T (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(v_max), &thisId);
  emlrtDestroyArray(&v_max);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[6]
 */
  static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[6]
{
  real_T (*y)[6];
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *DoF
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *DoF, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(DoF), &thisId);
  emlrtDestroyArray(&DoF);
  return y;
}

/*
 * Arguments    : const real_T u[6]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[6])
{
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[2] = { 0, 0 };

  static const int32_T iv1[2] = { 1, 6 };

  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, *(int32_T (*)[2])&iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[6]
 */
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  static const int32_T dims[2] = { 1, 6 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const mxArray * const prhs[9]
 *                int32_T nlhs
 *                const mxArray *plhs[3]
 * Return Type  : void
 */
  void ltpTrajectory_api(const mxArray * const prhs[9], int32_T nlhs, const
  mxArray *plhs[3])
{
  real_T (*q_traj)[6];
  real_T (*v_traj)[6];
  real_T (*a_traj)[6];
  real_T DoF;
  real_T Tsample;
  real_T (*v_max)[6];
  real_T (*a_max)[6];
  real_T (*j_max)[6];
  real_T (*q_goal)[6];
  real_T (*q_0)[6];
  real_T (*v_0)[6];
  real_T (*a_0)[6];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  q_traj = (real_T (*)[6])mxMalloc(sizeof(real_T [6]));
  v_traj = (real_T (*)[6])mxMalloc(sizeof(real_T [6]));
  a_traj = (real_T (*)[6])mxMalloc(sizeof(real_T [6]));

  /* Marshall function inputs */
  DoF = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "DoF");
  Tsample = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "Tsample");
  v_max = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "v_max");
  a_max = c_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "a_max");
  j_max = c_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "j_max");
  q_goal = c_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "q_goal");
  q_0 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "q_0");
  v_0 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[7]), "v_0");
  a_0 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[8]), "a_0");

  /* Invoke the target function */
  ltpTrajectory(DoF, Tsample, *v_max, *a_max, *j_max, *q_goal, *q_0, *v_0, *a_0,
                *q_traj, *v_traj, *a_traj);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*q_traj);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(*v_traj);
  }

  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(*a_traj);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ltpTrajectory_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  ltpTrajectory_xil_terminate();
  ltpTrajectory_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ltpTrajectory_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ltpTrajectory_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_ltpTrajectory_api.c
 *
 * [EOF]
 */
