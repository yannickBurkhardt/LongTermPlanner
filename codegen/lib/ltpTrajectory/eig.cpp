//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eig.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-May-2022 10:45:13
//

// Include Files
#include "eig.h"
#include "ltpTrajectory.h"
#include "rt_nonfinite.h"
#include "xgehrd.h"
#include "xzgeev.h"
#include "xzhseqr.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : const creal_T A_data[]
//                const int A_size[2]
//                creal_T V_data[]
//                int V_size[1]
// Return Type  : void
//
void eig(const creal_T A_data[], const int A_size[2], creal_T V_data[], int
         V_size[1])
{
  int jend;
  boolean_T p;
  int istart;
  int info;
  creal_T beta1_data[6];
  int beta1_size[1];
  int T_size[2];
  creal_T T_data[36];
  jend = A_size[0] * A_size[1];
  p = true;
  for (istart = 0; istart < jend; istart++) {
    if ((!p) || (rtIsInf(A_data[istart].re) || rtIsInf(A_data[istart].im) ||
                 (rtIsNaN(A_data[istart].re) || rtIsNaN(A_data[istart].im)))) {
      p = false;
    }
  }

  if (!p) {
    if ((A_size[0] == 1) && (A_size[1] == 1)) {
      V_size[0] = 1;
      V_data[0].re = rtNaN;
      V_data[0].im = 0.0;
    } else {
      V_size[0] = A_size[0];
      info = A_size[0];
      for (int i = 0; i < info; i++) {
        V_data[i].re = rtNaN;
        V_data[i].im = 0.0;
      }
    }
  } else if ((A_size[0] == 1) && (A_size[1] == 1)) {
    V_size[0] = 1;
    V_data[0] = A_data[0];
  } else {
    int j;
    int i;
    int b_i;
    p = (A_size[0] == A_size[1]);
    if (p) {
      boolean_T exitg2;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j <= A_size[1] - 1)) {
        int exitg1;
        b_i = 0;
        do {
          exitg1 = 0;
          if (b_i <= j) {
            i = b_i + A_size[0] * j;
            info = j + A_size[0] * b_i;
            if ((!(A_data[i].re == A_data[info].re)) || (!(A_data[i].im ==
                  -A_data[info].im))) {
              p = false;
              exitg1 = 1;
            } else {
              b_i++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }

    if (p) {
      p = true;
      for (istart = 0; istart < jend; istart++) {
        if ((!p) || (rtIsInf(A_data[istart].re) || rtIsInf(A_data[istart].im) ||
                     (rtIsNaN(A_data[istart].re) || rtIsNaN(A_data[istart].im))))
        {
          p = false;
        }
      }

      if (!p) {
        signed char unnamed_idx_0;
        signed char unnamed_idx_1;
        unnamed_idx_0 = static_cast<signed char>(A_size[0]);
        unnamed_idx_1 = static_cast<signed char>(A_size[1]);
        T_size[0] = unnamed_idx_0;
        info = unnamed_idx_0 * unnamed_idx_1;
        for (i = 0; i < info; i++) {
          T_data[i].re = rtNaN;
          T_data[i].im = 0.0;
        }

        info = unnamed_idx_0;
        if (1 < unnamed_idx_0) {
          istart = 2;
          if (unnamed_idx_0 - 2 < unnamed_idx_1 - 1) {
            jend = unnamed_idx_0 - 1;
          } else {
            jend = unnamed_idx_1;
          }

          for (j = 0; j < jend; j++) {
            for (b_i = istart; b_i <= info; b_i++) {
              i = (b_i + unnamed_idx_0 * j) - 1;
              T_data[i].re = 0.0;
              T_data[i].im = 0.0;
            }

            istart++;
          }
        }
      } else {
        T_size[0] = A_size[0];
        T_size[1] = A_size[1];
        if (0 <= jend - 1) {
          std::memcpy(&T_data[0], &A_data[0], jend * sizeof(creal_T));
        }

        xgehrd(T_data, T_size);
        eml_zlahqr(T_data, T_size);
        info = T_size[0];
        if (3 < T_size[0]) {
          istart = 4;
          if (T_size[0] - 4 < T_size[1] - 1) {
            jend = T_size[0] - 3;
          } else {
            jend = T_size[1];
          }

          for (j = 0; j < jend; j++) {
            for (b_i = istart; b_i <= info; b_i++) {
              i = (b_i + T_size[0] * j) - 1;
              T_data[i].re = 0.0;
              T_data[i].im = 0.0;
            }

            istart++;
          }
        }
      }

      info = T_size[0];
      V_size[0] = T_size[0];
      for (istart = 0; istart < info; istart++) {
        V_data[istart].re = T_data[istart + T_size[0] * istart].re;
        V_data[istart].im = 0.0;
      }
    } else {
      xzgeev(A_data, A_size, &info, V_data, V_size, beta1_data, beta1_size);
      info = V_size[0];
      for (i = 0; i < info; i++) {
        double re;
        double bim;
        if (beta1_data[i].im == 0.0) {
          if (V_data[i].im == 0.0) {
            re = V_data[i].re / beta1_data[i].re;
            bim = 0.0;
          } else if (V_data[i].re == 0.0) {
            re = 0.0;
            bim = V_data[i].im / beta1_data[i].re;
          } else {
            re = V_data[i].re / beta1_data[i].re;
            bim = V_data[i].im / beta1_data[i].re;
          }
        } else if (beta1_data[i].re == 0.0) {
          if (V_data[i].re == 0.0) {
            re = V_data[i].im / beta1_data[i].im;
            bim = 0.0;
          } else if (V_data[i].im == 0.0) {
            re = 0.0;
            bim = -(V_data[i].re / beta1_data[i].im);
          } else {
            re = V_data[i].im / beta1_data[i].im;
            bim = -(V_data[i].re / beta1_data[i].im);
          }
        } else {
          double brm;
          brm = std::abs(beta1_data[i].re);
          bim = std::abs(beta1_data[i].im);
          if (brm > bim) {
            double d;
            bim = beta1_data[i].im / beta1_data[i].re;
            d = beta1_data[i].re + bim * beta1_data[i].im;
            re = (V_data[i].re + bim * V_data[i].im) / d;
            bim = (V_data[i].im - bim * V_data[i].re) / d;
          } else if (bim == brm) {
            double d;
            if (beta1_data[i].re > 0.0) {
              bim = 0.5;
            } else {
              bim = -0.5;
            }

            if (beta1_data[i].im > 0.0) {
              d = 0.5;
            } else {
              d = -0.5;
            }

            re = (V_data[i].re * bim + V_data[i].im * d) / brm;
            bim = (V_data[i].im * bim - V_data[i].re * d) / brm;
          } else {
            double d;
            bim = beta1_data[i].re / beta1_data[i].im;
            d = beta1_data[i].im + bim * beta1_data[i].re;
            re = (bim * V_data[i].re + V_data[i].im) / d;
            bim = (bim * V_data[i].im - V_data[i].re) / d;
          }
        }

        V_data[i].re = re;
        V_data[i].im = bim;
      }
    }
  }
}

//
// File trailer for eig.cpp
//
// [EOF]
//
