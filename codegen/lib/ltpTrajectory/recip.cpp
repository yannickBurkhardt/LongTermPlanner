//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: recip.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 20-Jul-2022 14:48:22
//

// Include Files
#include "recip.h"
#include "ltpTrajectory.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Arguments    : const creal_T y
// Return Type  : creal_T
//
creal_T recip(const creal_T y)
{
  creal_T z;
  double brm;
  double bim;
  brm = std::abs(y.re);
  bim = std::abs(y.im);
  if (y.im == 0.0) {
    z.re = 1.0 / y.re;
    z.im = 0.0;
  } else if (y.re == 0.0) {
    z.re = 0.0;
    z.im = -1.0 / y.im;
  } else if (brm > bim) {
    double d;
    bim = y.im / y.re;
    d = y.re + bim * y.im;
    z.re = 1.0 / d;
    z.im = -bim / d;
  } else if (brm == bim) {
    double d;
    bim = 0.5;
    if (y.re < 0.0) {
      bim = -0.5;
    }

    d = 0.5;
    if (y.im < 0.0) {
      d = -0.5;
    }

    z.re = bim / brm;
    z.im = -d / brm;
  } else {
    double d;
    bim = y.re / y.im;
    d = y.im + bim * y.re;
    z.re = bim / d;
    z.im = -1.0 / d;
  }

  return z;
}

//
// File trailer for recip.cpp
//
// [EOF]
//
