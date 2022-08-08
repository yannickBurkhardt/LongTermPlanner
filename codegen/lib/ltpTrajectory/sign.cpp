//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sign.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 20-Jul-2022 14:48:22
//

// Include Files
#include "sign.h"
#include "ltpTrajectory.h"
#include "ltpTrajectory_rtwutil.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include <cmath>

// Function Definitions

//
// Arguments    : creal_T *x
// Return Type  : void
//
void b_sign(creal_T *x)
{
  double xr;
  double xi;
  xr = x->re;
  xi = x->im;
  if (xi == 0.0) {
    if (xr < 0.0) {
      xr = -1.0;
    } else if (xr > 0.0) {
      xr = 1.0;
    } else {
      if (xr == 0.0) {
        xr = 0.0;
      }
    }

    x->re = xr;
    x->im = 0.0;
  } else {
    double absx;
    if ((std::abs(xr) > 8.9884656743115785E+307) || (std::abs(xi) >
         8.9884656743115785E+307)) {
      xr /= 2.0;
      xi /= 2.0;
    }

    absx = rt_hypotd_snf(xr, xi);
    if (absx == 0.0) {
      x->re = 0.0;
      x->im = 0.0;
    } else {
      x->re = xr / absx;
      x->im = xi / absx;
    }
  }
}

//
// File trailer for sign.cpp
//
// [EOF]
//
