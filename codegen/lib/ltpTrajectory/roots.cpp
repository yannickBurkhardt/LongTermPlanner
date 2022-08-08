//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: roots.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 20-Jul-2022 14:48:22
//

// Include Files
#include "roots.h"
#include "eig.h"
#include "ltpTrajectory.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : const double c[7]
//                creal_T r_data[]
//                int r_size[1]
// Return Type  : void
//
void b_roots(const double c[7], creal_T r_data[], int r_size[1])
{
  int k1;
  int k2;
  int nTrailingZeros;
  int a_size[2];
  double ctmp[7];
  creal_T a_data[36];
  creal_T eiga_data[6];
  int eiga_size[1];
  std::memset(&r_data[0], 0, 6U * sizeof(creal_T));
  k1 = 1;
  while ((k1 <= 7) && (!(c[k1 - 1] != 0.0))) {
    k1++;
  }

  k2 = 7;
  while ((k2 >= k1) && (!(c[k2 - 1] != 0.0))) {
    k2--;
  }

  nTrailingZeros = 6 - k2;
  if (k1 < k2) {
    int companDim;
    boolean_T exitg1;
    int j;
    companDim = k2 - k1;
    exitg1 = false;
    while ((!exitg1) && (companDim > 0)) {
      boolean_T exitg2;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j + 1 <= companDim)) {
        ctmp[j] = c[k1 + j] / c[k1 - 1];
        if (rtIsInf(std::abs(ctmp[j]))) {
          exitg2 = true;
        } else {
          j++;
        }
      }

      if (j + 1 > companDim) {
        exitg1 = true;
      } else {
        k1++;
        companDim--;
      }
    }

    if (companDim < 1) {
      if (1 > 7 - k2) {
        r_size[0] = 0;
      } else {
        r_size[0] = 7 - k2;
      }
    } else {
      a_size[0] = companDim;
      a_size[1] = companDim;
      k1 = companDim * companDim;
      if (0 <= k1 - 1) {
        std::memset(&a_data[0], 0, k1 * sizeof(creal_T));
      }

      for (k1 = 0; k1 <= companDim - 2; k1++) {
        j = companDim * k1;
        a_data[j].re = -ctmp[k1];
        a_data[j].im = 0.0;
        j = (k1 + j) + 1;
        a_data[j].re = 1.0;
        a_data[j].im = 0.0;
      }

      j = companDim * (companDim - 1);
      a_data[j].re = -ctmp[companDim - 1];
      a_data[j].im = 0.0;
      if (0 <= nTrailingZeros) {
        std::memset(&r_data[0], 0, (nTrailingZeros + 1) * sizeof(creal_T));
      }

      eig(a_data, a_size, eiga_data, eiga_size);
      for (k1 = 0; k1 < companDim; k1++) {
        r_data[(k1 - k2) + 7] = eiga_data[k1];
      }

      r_size[0] = (companDim - k2) + 7;
    }
  } else if (1 > 7 - k2) {
    r_size[0] = 0;
  } else {
    r_size[0] = 7 - k2;
  }
}

//
// Arguments    : const double c[5]
//                creal_T r_data[]
//                int r_size[1]
// Return Type  : void
//
void roots(const double c[5], creal_T r_data[], int r_size[1])
{
  int k1;
  int k2;
  int nTrailingZeros;
  int a_size[2];
  double ctmp[5];
  creal_T a_data[16];
  creal_T tmp_data[6];
  int tmp_size[1];
  creal_T eiga_data[4];
  std::memset(&r_data[0], 0, 4U * sizeof(creal_T));
  k1 = 1;
  while ((k1 <= 5) && (!(c[k1 - 1] != 0.0))) {
    k1++;
  }

  k2 = 5;
  while ((k2 >= k1) && (!(c[k2 - 1] != 0.0))) {
    k2--;
  }

  nTrailingZeros = 4 - k2;
  if (k1 < k2) {
    int companDim;
    boolean_T exitg1;
    int j;
    companDim = k2 - k1;
    exitg1 = false;
    while ((!exitg1) && (companDim > 0)) {
      boolean_T exitg2;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j + 1 <= companDim)) {
        ctmp[j] = c[k1 + j] / c[k1 - 1];
        if (rtIsInf(std::abs(ctmp[j]))) {
          exitg2 = true;
        } else {
          j++;
        }
      }

      if (j + 1 > companDim) {
        exitg1 = true;
      } else {
        k1++;
        companDim--;
      }
    }

    if (companDim < 1) {
      if (1 > 5 - k2) {
        r_size[0] = 0;
      } else {
        r_size[0] = 5 - k2;
      }
    } else {
      a_size[0] = companDim;
      a_size[1] = companDim;
      k1 = companDim * companDim;
      if (0 <= k1 - 1) {
        std::memset(&a_data[0], 0, k1 * sizeof(creal_T));
      }

      for (k1 = 0; k1 <= companDim - 2; k1++) {
        j = companDim * k1;
        a_data[j].re = -ctmp[k1];
        a_data[j].im = 0.0;
        j = (k1 + j) + 1;
        a_data[j].re = 1.0;
        a_data[j].im = 0.0;
      }

      j = companDim * (companDim - 1);
      a_data[j].re = -ctmp[companDim - 1];
      a_data[j].im = 0.0;
      if (0 <= nTrailingZeros) {
        std::memset(&r_data[0], 0, (nTrailingZeros + 1) * sizeof(creal_T));
      }

      eig(a_data, a_size, tmp_data, tmp_size);
      k1 = tmp_size[0];
      if (0 <= k1 - 1) {
        std::memcpy(&eiga_data[0], &tmp_data[0], k1 * sizeof(creal_T));
      }

      for (k1 = 0; k1 < companDim; k1++) {
        r_data[(k1 - k2) + 5] = eiga_data[k1];
      }

      r_size[0] = (companDim - k2) + 5;
    }
  } else if (1 > 5 - k2) {
    r_size[0] = 0;
  } else {
    r_size[0] = 5 - k2;
  }
}

//
// File trailer for roots.cpp
//
// [EOF]
//
