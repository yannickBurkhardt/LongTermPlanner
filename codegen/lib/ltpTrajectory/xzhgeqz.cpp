//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzhgeqz.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 20-Jul-2022 14:48:22
//

// Include Files
#include "xzhgeqz.h"
#include "ltpTrajectory.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "xzlartg.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : const creal_T A_data[]
//                const int A_size[2]
//                int ilo
//                int ihi
//                int *info
//                creal_T alpha1_data[]
//                int alpha1_size[1]
//                creal_T beta1_data[]
//                int beta1_size[1]
// Return Type  : void
//
void xzhgeqz(const creal_T A_data[], const int A_size[2], int ilo, int ihi, int *
             info, creal_T alpha1_data[], int alpha1_size[1], creal_T
             beta1_data[], int beta1_size[1])
{
  int A_size_idx_0;
  int jm1;
  creal_T b_A_data[36];
  int n;
  int ctemp_tmp;
  double eshift_re;
  double eshift_im;
  creal_T ctemp;
  double anorm;
  double scale;
  double reAij;
  double sumsq;
  double b_atol;
  boolean_T firstNonZero;
  int j;
  double ascale;
  int i;
  double bscale;
  double imAij;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  double temp2;
  int ilast;
  creal_T shift;
  creal_T b_ascale;
  A_size_idx_0 = A_size[0];
  jm1 = A_size[0] * A_size[1];
  if (0 <= jm1 - 1) {
    std::memcpy(&b_A_data[0], &A_data[0], jm1 * sizeof(creal_T));
  }

  *info = 0;
  if ((A_size[0] == 1) && (A_size[1] == 1)) {
    ihi = 1;
  }

  n = A_size[0];
  alpha1_size[0] = A_size[0];
  jm1 = A_size[0];
  if (0 <= jm1 - 1) {
    std::memset(&alpha1_data[0], 0, jm1 * sizeof(creal_T));
  }

  beta1_size[0] = A_size[0];
  jm1 = A_size[0];
  for (ctemp_tmp = 0; ctemp_tmp < jm1; ctemp_tmp++) {
    beta1_data[ctemp_tmp].re = 1.0;
    beta1_data[ctemp_tmp].im = 0.0;
  }

  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  anorm = 0.0;
  if (ilo <= ihi) {
    scale = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      ctemp_tmp = j + 1;
      if (ihi < j + 1) {
        ctemp_tmp = ihi;
      }

      for (i = ilo; i <= ctemp_tmp; i++) {
        jm1 = (i + A_size[0] * (j - 1)) - 1;
        reAij = A_data[jm1].re;
        imAij = A_data[jm1].im;
        if (reAij != 0.0) {
          anorm = std::abs(reAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = anorm;
            firstNonZero = false;
          } else if (scale < anorm) {
            temp2 = scale / anorm;
            sumsq = sumsq * temp2 * temp2 + 1.0;
            scale = anorm;
          } else {
            temp2 = anorm / scale;
            sumsq += temp2 * temp2;
          }
        }

        if (imAij != 0.0) {
          anorm = std::abs(imAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = anorm;
            firstNonZero = false;
          } else if (scale < anorm) {
            temp2 = scale / anorm;
            sumsq = sumsq * temp2 * temp2 + 1.0;
            scale = anorm;
          } else {
            temp2 = anorm / scale;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    anorm = scale * std::sqrt(sumsq);
  }

  reAij = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (reAij > 2.2250738585072014E-308) {
    b_atol = reAij;
  }

  reAij = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    reAij = anorm;
  }

  ascale = 1.0 / reAij;
  bscale = 1.0 / std::sqrt(static_cast<double>(A_size[0]));
  firstNonZero = true;
  ctemp_tmp = ihi + 1;
  for (j = ctemp_tmp; j <= n; j++) {
    alpha1_data[j - 1] = A_data[(j + A_size[0] * (j - 1)) - 1];
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    int ifirst;
    int istart;
    int ilastm1;
    int ilastm;
    int iiter;
    boolean_T goto60;
    boolean_T goto70;
    boolean_T goto90;
    int jiter;
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    ilastm = ihi;
    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (jiter <= 30 * ((ihi - ilo) + 1) - 1) {
        boolean_T b_guard1 = false;
        boolean_T exitg2;
        b_guard1 = false;
        if (ilast + 1 == ilo) {
          goto60 = true;
          b_guard1 = true;
        } else {
          ctemp_tmp = ilast + A_size_idx_0 * ilastm1;
          if (std::abs(b_A_data[ctemp_tmp].re) + std::abs(b_A_data[ctemp_tmp].im)
              <= b_atol) {
            b_A_data[ctemp_tmp].re = 0.0;
            b_A_data[ctemp_tmp].im = 0.0;
            goto60 = true;
            b_guard1 = true;
          } else {
            boolean_T guard3 = false;
            j = ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (j + 1 >= ilo)) {
              if (j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                ctemp_tmp = j + A_size_idx_0 * (j - 1);
                if (std::abs(b_A_data[ctemp_tmp].re) + std::abs
                    (b_A_data[ctemp_tmp].im) <= b_atol) {
                  b_A_data[ctemp_tmp].re = 0.0;
                  b_A_data[ctemp_tmp].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  j--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              ifirst = j + 1;
              goto70 = true;
            }

            if (goto70) {
              b_guard1 = true;
            } else {
              jm1 = alpha1_size[0];
              for (ctemp_tmp = 0; ctemp_tmp < jm1; ctemp_tmp++) {
                alpha1_data[ctemp_tmp].re = rtNaN;
                alpha1_data[ctemp_tmp].im = 0.0;
              }

              jm1 = beta1_size[0];
              for (ctemp_tmp = 0; ctemp_tmp < jm1; ctemp_tmp++) {
                beta1_data[ctemp_tmp].re = rtNaN;
                beta1_data[ctemp_tmp].im = 0.0;
              }

              *info = 1;
              exitg1 = 1;
            }
          }
        }

        if (b_guard1) {
          if (goto60) {
            goto60 = false;
            alpha1_data[ilast] = b_A_data[ilast + A_size_idx_0 * ilast];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              firstNonZero = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0;
              eshift_im = 0.0;
              ilastm = ilast + 1;
              jiter++;
            }
          } else {
            if (goto70) {
              double ad22_re;
              double ad22_im;
              goto70 = false;
              iiter++;
              if (iiter - iiter / 10 * 10 != 0) {
                double t1_re;
                double t1_im;
                double t1_im_tmp;
                jm1 = ilastm1 + A_size_idx_0 * ilastm1;
                anorm = ascale * b_A_data[jm1].re;
                reAij = ascale * b_A_data[jm1].im;
                if (reAij == 0.0) {
                  shift.re = anorm / bscale;
                  shift.im = 0.0;
                } else if (anorm == 0.0) {
                  shift.re = 0.0;
                  shift.im = reAij / bscale;
                } else {
                  shift.re = anorm / bscale;
                  shift.im = reAij / bscale;
                }

                jm1 = ilast + A_size_idx_0 * ilast;
                anorm = ascale * b_A_data[jm1].re;
                reAij = ascale * b_A_data[jm1].im;
                if (reAij == 0.0) {
                  ad22_re = anorm / bscale;
                  ad22_im = 0.0;
                } else if (anorm == 0.0) {
                  ad22_re = 0.0;
                  ad22_im = reAij / bscale;
                } else {
                  ad22_re = anorm / bscale;
                  ad22_im = reAij / bscale;
                }

                t1_re = 0.5 * (shift.re + ad22_re);
                t1_im = 0.5 * (shift.im + ad22_im);
                t1_im_tmp = t1_re * t1_im;
                jm1 = ilastm1 + A_size_idx_0 * ilast;
                anorm = ascale * b_A_data[jm1].re;
                reAij = ascale * b_A_data[jm1].im;
                if (reAij == 0.0) {
                  imAij = anorm / bscale;
                  temp2 = 0.0;
                } else if (anorm == 0.0) {
                  imAij = 0.0;
                  temp2 = reAij / bscale;
                } else {
                  imAij = anorm / bscale;
                  temp2 = reAij / bscale;
                }

                jm1 = ilast + A_size_idx_0 * ilastm1;
                anorm = ascale * b_A_data[jm1].re;
                reAij = ascale * b_A_data[jm1].im;
                if (reAij == 0.0) {
                  sumsq = anorm / bscale;
                  anorm = 0.0;
                } else if (anorm == 0.0) {
                  sumsq = 0.0;
                  anorm = reAij / bscale;
                } else {
                  sumsq = anorm / bscale;
                  anorm = reAij / bscale;
                }

                reAij = shift.re * ad22_re - shift.im * ad22_im;
                scale = shift.re * ad22_im + shift.im * ad22_re;
                shift.re = ((t1_re * t1_re - t1_im * t1_im) + (imAij * sumsq -
                  temp2 * anorm)) - reAij;
                shift.im = ((t1_im_tmp + t1_im_tmp) + (imAij * anorm + temp2 *
                  sumsq)) - scale;
                b_sqrt(&shift);
                if ((t1_re - ad22_re) * shift.re + (t1_im - ad22_im) * shift.im <=
                    0.0) {
                  shift.re += t1_re;
                  shift.im += t1_im;
                } else {
                  shift.re = t1_re - shift.re;
                  shift.im = t1_im - shift.im;
                }
              } else {
                jm1 = ilast + A_size_idx_0 * ilastm1;
                anorm = ascale * b_A_data[jm1].re;
                reAij = ascale * b_A_data[jm1].im;
                if (reAij == 0.0) {
                  imAij = anorm / bscale;
                  temp2 = 0.0;
                } else if (anorm == 0.0) {
                  imAij = 0.0;
                  temp2 = reAij / bscale;
                } else {
                  imAij = anorm / bscale;
                  temp2 = reAij / bscale;
                }

                eshift_re += imAij;
                eshift_im += temp2;
                shift.re = eshift_re;
                shift.im = eshift_im;
              }

              j = ilastm1;
              n = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > ifirst)) {
                istart = j + 1;
                jm1 = A_size_idx_0 * j;
                ctemp_tmp = j + jm1;
                ctemp.re = ascale * b_A_data[ctemp_tmp].re - shift.re * bscale;
                ctemp.im = ascale * b_A_data[ctemp_tmp].im - shift.im * bscale;
                anorm = std::abs(ctemp.re) + std::abs(ctemp.im);
                jm1 += n;
                temp2 = ascale * (std::abs(b_A_data[jm1].re) + std::abs
                                  (b_A_data[jm1].im));
                reAij = anorm;
                if (temp2 > anorm) {
                  reAij = temp2;
                }

                if ((reAij < 1.0) && (reAij != 0.0)) {
                  anorm /= reAij;
                  temp2 /= reAij;
                }

                ctemp_tmp = j + A_size_idx_0 * (j - 1);
                if ((std::abs(b_A_data[ctemp_tmp].re) + std::abs
                     (b_A_data[ctemp_tmp].im)) * temp2 <= anorm * b_atol) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  n = j;
                  j--;
                }
              }

              if (!goto90) {
                istart = ifirst;
                ctemp_tmp = (ifirst + A_size_idx_0 * (ifirst - 1)) - 1;
                ctemp.re = ascale * b_A_data[ctemp_tmp].re - shift.re * bscale;
                ctemp.im = ascale * b_A_data[ctemp_tmp].im - shift.im * bscale;
              }

              goto90 = false;
              jm1 = istart + A_size_idx_0 * (istart - 1);
              b_ascale.re = ascale * b_A_data[jm1].re;
              b_ascale.im = ascale * b_A_data[jm1].im;
              b_xzlartg(ctemp, b_ascale, &anorm, &shift);
              j = istart;
              jm1 = istart - 2;
              while (j < ilast + 1) {
                if (j > istart) {
                  xzlartg(b_A_data[(j + A_size_idx_0 * jm1) - 1], b_A_data[j +
                          A_size_idx_0 * jm1], &anorm, &shift, &b_A_data[(j +
                           A_size_idx_0 * jm1) - 1]);
                  ctemp_tmp = j + A_size_idx_0 * jm1;
                  b_A_data[ctemp_tmp].re = 0.0;
                  b_A_data[ctemp_tmp].im = 0.0;
                }

                for (n = j; n <= ilastm; n++) {
                  jm1 = j + A_size_idx_0 * (n - 1);
                  ctemp_tmp = jm1 - 1;
                  ad22_re = anorm * b_A_data[ctemp_tmp].re + (shift.re *
                    b_A_data[jm1].re - shift.im * b_A_data[jm1].im);
                  ad22_im = anorm * b_A_data[ctemp_tmp].im + (shift.re *
                    b_A_data[jm1].im + shift.im * b_A_data[jm1].re);
                  reAij = b_A_data[ctemp_tmp].re;
                  b_A_data[jm1].re = anorm * b_A_data[jm1].re - (shift.re *
                    b_A_data[ctemp_tmp].re + shift.im * b_A_data[ctemp_tmp].im);
                  b_A_data[jm1].im = anorm * b_A_data[jm1].im - (shift.re *
                    b_A_data[ctemp_tmp].im - shift.im * reAij);
                  b_A_data[ctemp_tmp].re = ad22_re;
                  b_A_data[ctemp_tmp].im = ad22_im;
                }

                shift.re = -shift.re;
                shift.im = -shift.im;
                n = j;
                if (ilast + 1 < j + 2) {
                  n = ilast - 1;
                }

                for (i = ifirst; i <= n + 2; i++) {
                  jm1 = (i + A_size_idx_0 * (j - 1)) - 1;
                  ctemp_tmp = (i + A_size_idx_0 * j) - 1;
                  ad22_re = anorm * b_A_data[ctemp_tmp].re + (shift.re *
                    b_A_data[jm1].re - shift.im * b_A_data[jm1].im);
                  ad22_im = anorm * b_A_data[ctemp_tmp].im + (shift.re *
                    b_A_data[jm1].im + shift.im * b_A_data[jm1].re);
                  reAij = b_A_data[ctemp_tmp].re;
                  b_A_data[jm1].re = anorm * b_A_data[jm1].re - (shift.re *
                    b_A_data[ctemp_tmp].re + shift.im * b_A_data[ctemp_tmp].im);
                  b_A_data[jm1].im = anorm * b_A_data[jm1].im - (shift.re *
                    b_A_data[ctemp_tmp].im - shift.im * reAij);
                  b_A_data[ctemp_tmp].re = ad22_re;
                  b_A_data[ctemp_tmp].im = ad22_im;
                }

                jm1 = j - 1;
                j++;
              }
            }

            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (firstNonZero) {
      *info = ilast + 1;
      for (jm1 = 0; jm1 <= ilast; jm1++) {
        alpha1_data[jm1].re = rtNaN;
        alpha1_data[jm1].im = 0.0;
        beta1_data[jm1].re = rtNaN;
        beta1_data[jm1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    for (j = 0; j <= ilo - 2; j++) {
      alpha1_data[j] = b_A_data[j + A_size_idx_0 * j];
    }
  }
}

//
// File trailer for xzhgeqz.cpp
//
// [EOF]
//
