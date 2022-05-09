//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgehrd.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-May-2022 10:45:13
//

// Include Files
#include "xgehrd.h"
#include "ltpTrajectory.h"
#include "recip.h"
#include "rt_nonfinite.h"
#include "xdlapy3.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : creal_T a_data[]
//                const int a_size[2]
// Return Type  : void
//
void xgehrd(creal_T a_data[], const int a_size[2])
{
  int n;
  int i;
  creal_T work_data[6];
  int b_i;
  double alpha1_re;
  double alpha1_im;
  creal_T tau_data[5];
  double beta1;
  int jA;
  creal_T c;
  int ia;
  n = a_size[0];
  i = static_cast<signed char>(a_size[0]);
  if (0 <= i - 1) {
    std::memset(&work_data[0], 0, i * sizeof(creal_T));
  }

  b_i = a_size[0];
  for (int c_i = 0; c_i <= b_i - 2; c_i++) {
    int in;
    int alpha1_re_tmp;
    int n_tmp_tmp;
    int n_tmp;
    double temp_re;
    int iv0_tmp;
    int k;
    int lastv;
    double temp_im;
    int knt;
    int lastc;
    int i1;
    boolean_T exitg2;
    int ix;
    int exitg1;
    int i2;
    in = (c_i + 1) * n;
    alpha1_re_tmp = (c_i + a_size[0] * c_i) + 1;
    alpha1_re = a_data[alpha1_re_tmp].re;
    alpha1_im = a_data[alpha1_re_tmp].im;
    i = c_i + 3;
    if (i >= n) {
      i = n;
    }

    i += c_i * n;
    n_tmp_tmp = n - c_i;
    n_tmp = n_tmp_tmp - 3;
    tau_data[c_i].re = 0.0;
    tau_data[c_i].im = 0.0;
    if (n_tmp + 2 > 0) {
      temp_re = xnrm2(n_tmp + 1, a_data, i);
      if ((temp_re != 0.0) || (a_data[alpha1_re_tmp].im != 0.0)) {
        beta1 = xdlapy3(a_data[alpha1_re_tmp].re, a_data[alpha1_re_tmp].im,
                        temp_re);
        if (a_data[alpha1_re_tmp].re >= 0.0) {
          beta1 = -beta1;
        }

        if (std::abs(beta1) < 1.0020841800044864E-292) {
          knt = -1;
          i1 = i + n_tmp;
          do {
            knt++;
            for (k = i; k <= i1; k++) {
              temp_re = 9.9792015476736E+291 * a_data[k - 1].im + 0.0 * a_data[k
                - 1].re;
              a_data[k - 1].re = 9.9792015476736E+291 * a_data[k - 1].re - 0.0 *
                a_data[k - 1].im;
              a_data[k - 1].im = temp_re;
            }

            beta1 *= 9.9792015476736E+291;
            alpha1_re *= 9.9792015476736E+291;
            alpha1_im *= 9.9792015476736E+291;
          } while (!(std::abs(beta1) >= 1.0020841800044864E-292));

          beta1 = xdlapy3(alpha1_re, alpha1_im, xnrm2(n_tmp + 1, a_data, i));
          if (alpha1_re >= 0.0) {
            beta1 = -beta1;
          }

          temp_im = beta1 - alpha1_re;
          if (0.0 - alpha1_im == 0.0) {
            tau_data[c_i].re = temp_im / beta1;
            tau_data[c_i].im = 0.0;
          } else if (temp_im == 0.0) {
            tau_data[c_i].re = 0.0;
            tau_data[c_i].im = (0.0 - alpha1_im) / beta1;
          } else {
            tau_data[c_i].re = temp_im / beta1;
            tau_data[c_i].im = (0.0 - alpha1_im) / beta1;
          }

          c.re = alpha1_re - beta1;
          c.im = alpha1_im;
          c = recip(c);
          for (k = i; k <= i1; k++) {
            temp_re = c.re * a_data[k - 1].im + c.im * a_data[k - 1].re;
            a_data[k - 1].re = c.re * a_data[k - 1].re - c.im * a_data[k - 1].im;
            a_data[k - 1].im = temp_re;
          }

          for (k = 0; k <= knt; k++) {
            beta1 *= 1.0020841800044864E-292;
          }

          alpha1_re = beta1;
          alpha1_im = 0.0;
        } else {
          temp_im = beta1 - a_data[alpha1_re_tmp].re;
          temp_re = 0.0 - a_data[alpha1_re_tmp].im;
          if (temp_re == 0.0) {
            tau_data[c_i].re = temp_im / beta1;
            tau_data[c_i].im = 0.0;
          } else if (temp_im == 0.0) {
            tau_data[c_i].re = 0.0;
            tau_data[c_i].im = temp_re / beta1;
          } else {
            tau_data[c_i].re = temp_im / beta1;
            tau_data[c_i].im = temp_re / beta1;
          }

          c.re = a_data[alpha1_re_tmp].re - beta1;
          c.im = a_data[alpha1_re_tmp].im;
          c = recip(c);
          i1 = i + n_tmp;
          for (k = i; k <= i1; k++) {
            temp_re = c.re * a_data[k - 1].im + c.im * a_data[k - 1].re;
            a_data[k - 1].re = c.re * a_data[k - 1].re - c.im * a_data[k - 1].im;
            a_data[k - 1].im = temp_re;
          }

          alpha1_re = beta1;
          alpha1_im = 0.0;
        }
      }
    }

    a_data[alpha1_re_tmp].re = 1.0;
    a_data[alpha1_re_tmp].im = 0.0;
    iv0_tmp = (c_i + c_i * n) + 1;
    k = in + 1;
    if ((tau_data[c_i].re != 0.0) || (tau_data[c_i].im != 0.0)) {
      lastv = n_tmp + 1;
      i = iv0_tmp + n_tmp;
      while ((lastv + 1 > 0) && ((a_data[i + 1].re == 0.0) && (a_data[i + 1].im ==
               0.0))) {
        lastv--;
        i--;
      }

      lastc = n;
      exitg2 = false;
      while ((!exitg2) && (lastc > 0)) {
        knt = in + lastc;
        ia = knt;
        do {
          exitg1 = 0;
          if ((n > 0) && (ia <= knt + lastv * n)) {
            if ((a_data[ia - 1].re != 0.0) || (a_data[ia - 1].im != 0.0)) {
              exitg1 = 1;
            } else {
              ia += n;
            }
          } else {
            lastc--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      lastv = -1;
      lastc = 0;
    }

    if (lastv + 1 > 0) {
      if (lastc != 0) {
        if (0 <= lastc - 1) {
          std::memset(&work_data[0], 0, lastc * sizeof(creal_T));
        }

        ix = iv0_tmp;
        i1 = (in + n * lastv) + 1;
        for (i = k; n < 0 ? i >= i1 : i <= i1; i += n) {
          c.re = a_data[ix].re - 0.0 * a_data[ix].im;
          c.im = a_data[ix].im + 0.0 * a_data[ix].re;
          knt = 0;
          i2 = (i + lastc) - 1;
          for (ia = i; ia <= i2; ia++) {
            work_data[knt].re += a_data[ia - 1].re * c.re - a_data[ia - 1].im *
              c.im;
            work_data[knt].im += a_data[ia - 1].re * c.im + a_data[ia - 1].im *
              c.re;
            knt++;
          }

          ix++;
        }
      }

      c.re = -tau_data[c_i].re;
      c.im = -tau_data[c_i].im;
      if ((!(-tau_data[c_i].re == 0.0)) || (!(-tau_data[c_i].im == 0.0))) {
        jA = in;
        knt = iv0_tmp;
        for (i = 0; i <= lastv; i++) {
          if ((a_data[knt].re != 0.0) || (a_data[knt].im != 0.0)) {
            temp_re = a_data[knt].re * c.re + a_data[knt].im * c.im;
            temp_im = a_data[knt].re * c.im - a_data[knt].im * c.re;
            ix = 0;
            i1 = jA + 1;
            i2 = lastc + jA;
            for (k = i1; k <= i2; k++) {
              a_data[k - 1].re += work_data[ix].re * temp_re - work_data[ix].im *
                temp_im;
              a_data[k - 1].im += work_data[ix].re * temp_im + work_data[ix].im *
                temp_re;
              ix++;
            }
          }

          knt++;
          jA += n;
        }
      }
    }

    jA = (c_i + in) + 2;
    if ((tau_data[c_i].re != 0.0) || (-tau_data[c_i].im != 0.0)) {
      lastv = n_tmp + 2;
      i = iv0_tmp + n_tmp;
      while ((lastv > 0) && ((a_data[i + 1].re == 0.0) && (a_data[i + 1].im ==
               0.0))) {
        lastv--;
        i--;
      }

      lastc = n_tmp_tmp - 2;
      exitg2 = false;
      while ((!exitg2) && (lastc + 1 > 0)) {
        i = jA + lastc * n;
        ia = i;
        do {
          exitg1 = 0;
          if (ia <= (i + lastv) - 1) {
            if ((a_data[ia - 1].re != 0.0) || (a_data[ia - 1].im != 0.0)) {
              exitg1 = 1;
            } else {
              ia++;
            }
          } else {
            lastc--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      lastv = 0;
      lastc = -1;
    }

    if (lastv > 0) {
      if (lastc + 1 != 0) {
        if (0 <= lastc) {
          std::memset(&work_data[0], 0, (lastc + 1) * sizeof(creal_T));
        }

        knt = 0;
        i1 = jA + n * lastc;
        for (i = jA; n < 0 ? i >= i1 : i <= i1; i += n) {
          ix = iv0_tmp;
          c.re = 0.0;
          c.im = 0.0;
          i2 = (i + lastv) - 1;
          for (ia = i; ia <= i2; ia++) {
            c.re += a_data[ia - 1].re * a_data[ix].re + a_data[ia - 1].im *
              a_data[ix].im;
            c.im += a_data[ia - 1].re * a_data[ix].im - a_data[ia - 1].im *
              a_data[ix].re;
            ix++;
          }

          work_data[knt].re += c.re - 0.0 * c.im;
          work_data[knt].im += c.im + 0.0 * c.re;
          knt++;
        }
      }

      c.re = -tau_data[c_i].re;
      c.im = tau_data[c_i].im;
      if ((!(-tau_data[c_i].re == 0.0)) || (!(tau_data[c_i].im == 0.0))) {
        knt = 0;
        for (i = 0; i <= lastc; i++) {
          if ((work_data[knt].re != 0.0) || (work_data[knt].im != 0.0)) {
            temp_re = work_data[knt].re * c.re + work_data[knt].im * c.im;
            temp_im = work_data[knt].re * c.im - work_data[knt].im * c.re;
            ix = iv0_tmp;
            i1 = lastv + jA;
            for (k = jA; k < i1; k++) {
              beta1 = a_data[ix].re * temp_im + a_data[ix].im * temp_re;
              a_data[k - 1].re += a_data[ix].re * temp_re - a_data[ix].im *
                temp_im;
              a_data[k - 1].im += beta1;
              ix++;
            }
          }

          knt++;
          jA += n;
        }
      }
    }

    a_data[alpha1_re_tmp].re = alpha1_re;
    a_data[alpha1_re_tmp].im = alpha1_im;
  }
}

//
// File trailer for xgehrd.cpp
//
// [EOF]
//
