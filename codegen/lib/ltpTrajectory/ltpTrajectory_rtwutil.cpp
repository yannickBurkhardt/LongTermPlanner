//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ltpTrajectory_rtwutil.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-May-2022 10:45:13
//

// Include Files
#include "ltpTrajectory_rtwutil.h"
#include "ltpTrajectory.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

//
// File trailer for ltpTrajectory_rtwutil.cpp
//
// [EOF]
//
