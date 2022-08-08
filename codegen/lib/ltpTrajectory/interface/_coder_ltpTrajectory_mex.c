/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ltpTrajectory_mex.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 20-Jul-2022 14:48:22
 */

/* Include Files */
#include "_coder_ltpTrajectory_mex.h"
#include "_coder_ltpTrajectory_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void ltpTrajectory_mexFunction(int32_T nlhs, mxArray *plhs[3],
  int32_T nrhs, const mxArray *prhs[9]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[3]
 *                int32_T nrhs
 *                const mxArray *prhs[9]
 * Return Type  : void
 */
void ltpTrajectory_mexFunction(int32_T nlhs, mxArray *plhs[3], int32_T nrhs,
  const mxArray *prhs[9])
{
  const mxArray *outputs[3];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 9) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 9, 4,
                        13, "ltpTrajectory");
  }

  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 13,
                        "ltpTrajectory");
  }

  /* Call the function. */
  ltpTrajectory_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(&ltpTrajectory_atexit);

  /* Module initialization. */
  ltpTrajectory_initialize();

  /* Dispatch the entry-point. */
  ltpTrajectory_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  ltpTrajectory_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/*
 * File trailer for _coder_ltpTrajectory_mex.c
 *
 * [EOF]
 */
