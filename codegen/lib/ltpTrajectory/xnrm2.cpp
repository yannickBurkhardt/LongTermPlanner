//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xnrm2.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-May-2022 10:45:13
//

// Include Files
#include "xnrm2.h"
#include "ltpTrajectory.h"
#include "ltpTrajectory_rtwutil.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include <cmath>

// Function Definitions

//
// Arguments    : int n
//                const creal_T x_data[]
//                int ix0
// Return Type  : double
//
double xnrm2(int n, const creal_T x_data[], int ix0)
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = rt_hypotd_snf(x_data[ix0 - 1].re, x_data[ix0 - 1].im);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int k = ix0; k <= kend; k++) {
        double absxk;
        double t;
        absxk = std::abs(x_data[k - 1].re);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }

        absxk = std::abs(x_data[k - 1].im);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

//
// File trailer for xnrm2.cpp
//
// [EOF]
//
