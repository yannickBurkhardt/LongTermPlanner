//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzgeev.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 20-Jul-2022 14:48:22
//

// Include Files
#include "xzgeev.h"
#include "ltpTrajectory.h"
#include "ltpTrajectory_rtwutil.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "xzhgeqz.h"
#include "xzlartg.h"
#include <cstring>

// Function Definitions

//
// Arguments    : const creal_T A_data[]
//                const int A_size[2]
//                int *info
//                creal_T alpha1_data[]
//                int alpha1_size[1]
//                creal_T beta1_data[]
//                int beta1_size[1]
// Return Type  : void
//
void xzgeev(const creal_T A_data[], const int A_size[2], int *info, creal_T
            alpha1_data[], int alpha1_size[1], creal_T beta1_data[], int
            beta1_size[1])
{
  int At_size[2];
  int ii;
  creal_T At_data[36];
  double anrm;
  int jcolp1;
  boolean_T exitg1;
  double absxk;
  creal_T atmp;
  At_size[0] = A_size[0];
  At_size[1] = A_size[1];
  ii = A_size[0] * A_size[1];
  if (0 <= ii - 1) {
    std::memcpy(&At_data[0], &A_data[0], ii * sizeof(creal_T));
  }

  *info = 0;
  anrm = 0.0;
  ii = A_size[0] * A_size[1];
  jcolp1 = 0;
  exitg1 = false;
  while ((!exitg1) && (jcolp1 <= ii - 1)) {
    absxk = rt_hypotd_snf(At_data[jcolp1].re, At_data[jcolp1].im);
    if (rtIsNaN(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }

      jcolp1++;
    }
  }

  if (rtIsInf(anrm) || rtIsNaN(anrm)) {
    int i;
    alpha1_size[0] = A_size[0];
    ii = A_size[0];
    for (i = 0; i < ii; i++) {
      alpha1_data[i].re = rtNaN;
      alpha1_data[i].im = 0.0;
    }

    beta1_size[0] = A_size[0];
    ii = A_size[0];
    for (i = 0; i < ii; i++) {
      beta1_data[i].re = rtNaN;
      beta1_data[i].im = 0.0;
    }
  } else {
    boolean_T ilascl;
    double anrmto;
    boolean_T guard1 = false;
    int i;
    int ilo;
    double ctoc;
    int ihi;
    boolean_T notdone;
    double stemp_im;
    int b_i;
    int n;
    double cto1;
    int j;
    int jcol;
    double a;
    int nzcount;
    ilascl = false;
    anrmto = anrm;
    guard1 = false;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      anrmto = 6.7178761075670888E-139;
      ilascl = true;
      guard1 = true;
    } else {
      if (anrm > 1.4885657073574029E+138) {
        anrmto = 1.4885657073574029E+138;
        ilascl = true;
        guard1 = true;
      }
    }

    if (guard1) {
      absxk = anrm;
      ctoc = anrmto;
      notdone = true;
      while (notdone) {
        stemp_im = absxk * 2.0041683600089728E-292;
        cto1 = ctoc / 4.9896007738368E+291;
        if ((stemp_im > ctoc) && (ctoc != 0.0)) {
          a = 2.0041683600089728E-292;
          absxk = stemp_im;
        } else if (cto1 > absxk) {
          a = 4.9896007738368E+291;
          ctoc = cto1;
        } else {
          a = ctoc / absxk;
          notdone = false;
        }

        for (i = 0; i < ii; i++) {
          At_data[i].re *= a;
          At_data[i].im *= a;
        }
      }
    }

    ilo = 1;
    ihi = A_size[0];
    if (A_size[0] <= 1) {
      ihi = 1;
    } else {
      int exitg3;
      boolean_T exitg4;
      do {
        exitg3 = 0;
        b_i = 0;
        j = 0;
        notdone = false;
        ii = ihi;
        exitg1 = false;
        while ((!exitg1) && (ii > 0)) {
          nzcount = 0;
          b_i = ii;
          j = ihi;
          jcol = 0;
          exitg4 = false;
          while ((!exitg4) && (jcol <= ihi - 1)) {
            i = (ii + At_size[0] * jcol) - 1;
            if ((At_data[i].re != 0.0) || (At_data[i].im != 0.0) || (ii == jcol
                 + 1)) {
              if (nzcount == 0) {
                j = jcol + 1;
                nzcount = 1;
                jcol++;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              jcol++;
            }
          }

          if (nzcount < 2) {
            notdone = true;
            exitg1 = true;
          } else {
            ii--;
          }
        }

        if (!notdone) {
          exitg3 = 2;
        } else {
          n = At_size[0];
          if (b_i != ihi) {
            for (jcolp1 = 1; jcolp1 <= n; jcolp1++) {
              ii = At_size[0] * (jcolp1 - 1);
              nzcount = (b_i + ii) - 1;
              atmp = At_data[nzcount];
              jcol = (ihi + ii) - 1;
              At_data[nzcount] = At_data[jcol];
              At_data[jcol] = atmp;
            }
          }

          if (j != ihi) {
            for (jcolp1 = 0; jcolp1 < ihi; jcolp1++) {
              ii = jcolp1 + At_size[0] * (j - 1);
              atmp = At_data[ii];
              jcol = jcolp1 + At_size[0] * (ihi - 1);
              At_data[ii] = At_data[jcol];
              At_data[jcol] = atmp;
            }
          }

          ihi--;
          if (ihi == 1) {
            exitg3 = 1;
          }
        }
      } while (exitg3 == 0);

      if (exitg3 != 1) {
        int exitg2;
        do {
          exitg2 = 0;
          b_i = 0;
          j = 0;
          notdone = false;
          jcol = ilo;
          exitg1 = false;
          while ((!exitg1) && (jcol <= ihi)) {
            nzcount = 0;
            b_i = ihi;
            j = jcol;
            ii = ilo;
            exitg4 = false;
            while ((!exitg4) && (ii <= ihi)) {
              i = (ii + At_size[0] * (jcol - 1)) - 1;
              if ((At_data[i].re != 0.0) || (At_data[i].im != 0.0) || (ii ==
                   jcol)) {
                if (nzcount == 0) {
                  b_i = ii;
                  nzcount = 1;
                  ii++;
                } else {
                  nzcount = 2;
                  exitg4 = true;
                }
              } else {
                ii++;
              }
            }

            if (nzcount < 2) {
              notdone = true;
              exitg1 = true;
            } else {
              jcol++;
            }
          }

          if (!notdone) {
            exitg2 = 1;
          } else {
            n = At_size[0];
            if (b_i != ilo) {
              for (jcolp1 = ilo; jcolp1 <= n; jcolp1++) {
                ii = At_size[0] * (jcolp1 - 1);
                nzcount = (b_i + ii) - 1;
                atmp = At_data[nzcount];
                jcol = (ilo + ii) - 1;
                At_data[nzcount] = At_data[jcol];
                At_data[jcol] = atmp;
              }
            }

            if (j != ilo) {
              for (jcolp1 = 0; jcolp1 < ihi; jcolp1++) {
                ii = jcolp1 + At_size[0] * (j - 1);
                atmp = At_data[ii];
                jcol = jcolp1 + At_size[0] * (ilo - 1);
                At_data[ii] = At_data[jcol];
                At_data[jcol] = atmp;
              }
            }

            ilo++;
            if (ilo == ihi) {
              exitg2 = 1;
            }
          }
        } while (exitg2 == 0);
      }
    }

    n = A_size[0];
    if ((A_size[0] > 1) && (ihi >= ilo + 2)) {
      for (jcol = ilo - 1; jcol + 1 < ihi - 1; jcol++) {
        jcolp1 = jcol + 2;
        for (int jrow = ihi - 1; jrow + 1 > jcol + 2; jrow--) {
          xzlartg(At_data[(jrow + At_size[0] * jcol) - 1], At_data[jrow +
                  At_size[0] * jcol], &absxk, &atmp, &At_data[(jrow + At_size[0]
                   * jcol) - 1]);
          i = jrow + At_size[0] * jcol;
          At_data[i].re = 0.0;
          At_data[i].im = 0.0;
          for (j = jcolp1; j <= n; j++) {
            ii = jrow + At_size[0] * (j - 1);
            nzcount = ii - 1;
            ctoc = absxk * At_data[nzcount].re + (atmp.re * At_data[ii].re -
              atmp.im * At_data[ii].im);
            stemp_im = absxk * At_data[nzcount].im + (atmp.re * At_data[ii].im +
              atmp.im * At_data[ii].re);
            cto1 = At_data[nzcount].re;
            At_data[ii].re = absxk * At_data[ii].re - (atmp.re * At_data[nzcount]
              .re + atmp.im * At_data[nzcount].im);
            At_data[ii].im = absxk * At_data[ii].im - (atmp.re * At_data[nzcount]
              .im - atmp.im * cto1);
            At_data[nzcount].re = ctoc;
            At_data[nzcount].im = stemp_im;
          }

          atmp.re = -atmp.re;
          atmp.im = -atmp.im;
          for (b_i = 1; b_i <= ihi; b_i++) {
            ii = (b_i + At_size[0] * (jrow - 1)) - 1;
            nzcount = (b_i + At_size[0] * jrow) - 1;
            ctoc = absxk * At_data[nzcount].re + (atmp.re * At_data[ii].re -
              atmp.im * At_data[ii].im);
            stemp_im = absxk * At_data[nzcount].im + (atmp.re * At_data[ii].im +
              atmp.im * At_data[ii].re);
            cto1 = At_data[nzcount].re;
            At_data[ii].re = absxk * At_data[ii].re - (atmp.re * At_data[nzcount]
              .re + atmp.im * At_data[nzcount].im);
            At_data[ii].im = absxk * At_data[ii].im - (atmp.re * At_data[nzcount]
              .im - atmp.im * cto1);
            At_data[nzcount].re = ctoc;
            At_data[nzcount].im = stemp_im;
          }
        }
      }
    }

    xzhgeqz(At_data, At_size, ilo, ihi, info, alpha1_data, alpha1_size,
            beta1_data, beta1_size);
    if ((*info == 0) && ilascl) {
      notdone = true;
      while (notdone) {
        stemp_im = anrmto * 2.0041683600089728E-292;
        cto1 = anrm / 4.9896007738368E+291;
        if ((stemp_im > anrm) && (anrm != 0.0)) {
          a = 2.0041683600089728E-292;
          anrmto = stemp_im;
        } else if (cto1 > anrmto) {
          a = 4.9896007738368E+291;
          anrm = cto1;
        } else {
          a = anrm / anrmto;
          notdone = false;
        }

        ii = alpha1_size[0];
        for (i = 0; i < ii; i++) {
          alpha1_data[i].re *= a;
          alpha1_data[i].im *= a;
        }
      }
    }
  }
}

//
// File trailer for xzgeev.cpp
//
// [EOF]
//
