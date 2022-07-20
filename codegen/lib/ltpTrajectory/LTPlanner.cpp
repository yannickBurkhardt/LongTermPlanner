//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: LTPlanner.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 20-Jul-2022 14:48:22
//

// Include Files
#include "LTPlanner.h"
#include "eig.h"
#include "ltpTrajectory.h"
#include "ltpTrajectory_rtwutil.h"
#include "roots.h"
#include "rt_nonfinite.h"
#include "sign.h"
#include "sqrt.h"
#include <cmath>
#include <cstring>
#include <math.h>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions

//
// OPTSWITCHTIMES Calculate time-optimal jerk swtiches
// Arguments    : double varargin_2
//                double varargin_3
//                double varargin_4
//                double varargin_5
//                double varargin_6
//                double varargin_7
//                double t[7]
//                double *dir
//                boolean_T *mod_jerk_profile
// Return Type  : void
//
void LTPlanner::b_optSwitchTimes(double varargin_2, double varargin_3, double
  varargin_4, double varargin_5, double varargin_6, double varargin_7, double t
  [7], double *dir, boolean_T *mod_jerk_profile) const
{
  double v_0;
  double a_0;
  int i;
  double q_diff;
  double t_tmp;
  double dv[5];
  creal_T root_data[4];
  int root_size[1];
  double x_data[4];
  double y_data[4];
  boolean_T tmp_data[4];

  //  Apply transfer parameters
  v_0 = varargin_4;
  a_0 = varargin_5;

  //  v_drive describes the constant velocity in phase 4.
  //  For time-optimal planning, this is v_max.
  //  For time-scaling, v_drive is calculated in timeScaling
  //  Parameters for calculation
  for (i = 0; i < 7; i++) {
    t[i] = 0.0;
  }

  //  Time that is required for one jerk phase
  *mod_jerk_profile = false;

  //  Use the standard jerk profile if not changed during calculations
  //  Check if inputs are in limits
  //  Calculate direction of movement
  this->optBreaking(varargin_4, varargin_5, varargin_6, (&q_diff), (*(double (*)
    [3])&t[0]), dir);
  q_diff = varargin_2 - (varargin_3 + q_diff);
  if (std::abs(q_diff) < 0.004) {
    //  Skip rest if that fulfils scenario
    for (int loop_ub = 0; loop_ub < 6; loop_ub++) {
      t[loop_ub + 1] += t[loop_ub];
    }
  } else {
    int loop_ub;
    boolean_T guard1 = false;
    boolean_T guard2 = false;
    boolean_T guard3 = false;
    double root;
    *dir = q_diff;
    if (q_diff < 0.0) {
      *dir = -1.0;
    } else if (q_diff > 0.0) {
      *dir = 1.0;
    } else {
      if (q_diff == 0.0) {
        *dir = 0.0;
      }
    }

    //  If goal is in negative direction, map to pos. direction
    if (*dir < 0.0) {
      v_0 = -varargin_4;
      a_0 = -varargin_5;
    }

    //             %% Calculate min. time required per joint to reach goal state 
    //  Check if slowing down is necessary to satisfy v_drive
    q_diff = 0.0;
    i = static_cast<int>(varargin_6) - 1;
    guard1 = false;
    guard2 = false;
    guard3 = false;
    if (v_0 + 0.5 * a_0 * std::abs(a_0) / this->j_max[i] > varargin_7) {
      *mod_jerk_profile = true;

      //  Get Joint Position after breaking and required times
      this->optBreaking((v_0 - varargin_7), a_0, varargin_6, (&q_diff),
                        (*(double (*)[3])&t[0]));
      guard3 = true;
    } else {
      //  Constant max/ min jerk (Phase 1, 3)
      t[0] = (this->a_max[i] - a_0) / this->j_max[i];
      t[2] = this->a_max[i] / this->j_max[i];

      //  Constant acceleration (Phase 2)
      t_tmp = varargin_7 - v_0;
      t[1] = (t_tmp - 0.5 * t[0] * a_0) / this->a_max[i] - 0.5 * (t[0] + t[2]);

      //  Check if phase 2 does not exist
      //  (max acceleration cannot be reached)
      if (t[1] < -0.004) {
        //  Check if root is positive
        root = this->j_max[i] * t_tmp + 0.5 * (a_0 * a_0);
        if (root > 0.0) {
          t[2] = std::sqrt(root) / this->j_max[i];
          t[0] = t[2] - a_0 / this->j_max[i];
          t[1] = 0.0;
          guard3 = true;
        } else {
          //  This should hould never occur, only for safety
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        }
      } else {
        guard3 = true;
      }
    }

    if (guard3) {
      //  Constant max/ min jerk (Phase 5, 7)
      t_tmp = this->a_max[i] / this->j_max[i];
      t[4] = t_tmp;
      t[6] = t_tmp;

      //  Constant acceleration (Phase 6)
      t[5] = varargin_7 / this->a_max[i] - 0.5 * (t_tmp + t_tmp);

      //  Check if phase 6 does not exist
      //  (max acceleration cannot be reached)
      if (t[5] < -0.004) {
        //  Check if root is positive
        root = varargin_7 / this->j_max[i];
        if (root > 0.0) {
          t[4] = std::sqrt(root);
          t[6] = t[4];
          t[5] = 0.0;
          guard2 = true;
        } else {
          //  This should hould never occur, only for safety
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        }
      } else {
        guard2 = true;
      }
    }

    if (guard2) {
      double q_part1_tmp;
      double b_t_tmp;

      //  Constant velocity (Phase 4)
      if (*mod_jerk_profile) {
        //  Breaking to satisfy v_drive
        q_diff += varargin_7 * ((t[0] + t[1]) + t[2]);
      } else {
        //  Acceleration to reach v_drive
        q_diff = 0.5 * (t[0] * t[0]);
        root = t[1] + t[2];
        q_part1_tmp = t[2] * t[2];
        q_diff = ((v_0 * ((t[0] + t[1]) + t[2]) + a_0 * ((q_diff + t[0] * root)
                    + 0.5 * q_part1_tmp)) + this->j_max[i] *
                  (((0.16666666666666666 * rt_powd_snf(t[0], 3.0) + q_diff *
                     root) - 0.16666666666666666 * rt_powd_snf(t[2], 3.0)) + 0.5
                   * t[0] * q_part1_tmp)) + this->a_max[i] * (0.5 * (t[1] * t[1])
          + t[1] * t[2]);
      }

      b_t_tmp = varargin_2 - varargin_3;
      t[3] = ((b_t_tmp * *dir - q_diff) - (this->j_max[i] *
               (((0.16666666666666666 * rt_powd_snf(t[6], 3.0) + 0.5 * (t[6] *
        t[6]) * (t[5] + t[4])) - 0.16666666666666666 * rt_powd_snf(t[4], 3.0)) +
                0.5 * t[6] * (t[4] * t[4])) + this->a_max[i] * (0.5 * (t[5] * t
                 [5]) + t[5] * t[4]))) / varargin_7;

      //  Check if phase 4 does not exist
      //  (max velocity cannot be reached)
      if (t[3] < -0.004) {
        if (*mod_jerk_profile) {
          //  This case is not valid
          //  (is handled in timeScaling function)
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        } else {
          double a;
          double b_a;
          double c_a;
          double d_a;
          double e_a;
          double f_a;
          double g_a;
          double root_tmp;
          double b_root_tmp;
          double c_root_tmp;
          double d_root_tmp;
          double e_root_tmp;
          double f_root_tmp;
          double g_root_tmp;
          double h_root_tmp;
          double i_root_tmp;

          //  Calculate times if max velocity is not reached
          a = this->j_max[i];
          b_a = this->j_max[i];
          c_a = this->j_max[i];
          d_a = this->j_max[i];
          e_a = this->j_max[i];
          f_a = this->a_max[i];
          g_a = this->a_max[i];
          root_tmp = 2.0 * this->j_max[i];
          q_diff = rt_powd_snf(t[0], 3.0);
          b_root_tmp = root_tmp * this->a_max[i];
          c_root_tmp = t[2] * t[2];
          d_root_tmp = t[4] * t[4];
          q_part1_tmp = t[0] * t[0];
          root = 2.0 * a_0 * this->a_max[i];
          e_root_tmp = 4.0 * this->a_max[i];
          f_root_tmp = a_0 * a_0;
          g_root_tmp = v_0 * v_0;
          h_root_tmp = 2.0 * f_root_tmp;
          i_root_tmp = root_tmp * v_0;
          root = ((((((((((((((((((((a * a * rt_powd_snf(t[0], 4.0) / 2.0 - b_a *
            b_a * rt_powd_snf(t[2], 4.0) / 4.0) + c_a * c_a * c_root_tmp *
            d_root_tmp / 2.0) - d_a * d_a * rt_powd_snf(t[4], 4.0) / 4.0) + e_a *
            e_a * rt_powd_snf(t[6], 4.0) / 2.0) + root_tmp * a_0 * q_diff) -
                                b_root_tmp * q_diff / 3.0) - b_root_tmp * t[0] *
                               c_root_tmp) + b_root_tmp * rt_powd_snf(t[2], 3.0)
                              / 3.0) + b_root_tmp * rt_powd_snf(t[4], 3.0) / 3.0)
                            - b_root_tmp * d_root_tmp * t[6]) - b_root_tmp *
                           rt_powd_snf(t[6], 3.0) / 3.0) + i_root_tmp *
                          q_part1_tmp) + h_root_tmp * q_part1_tmp) - root *
                        q_part1_tmp) - root * c_root_tmp) + 4.0 * a_0 * v_0 * t
                      [0]) + 2.0 * (f_a * f_a) * c_root_tmp) + 2.0 * (g_a * g_a)
                    * d_root_tmp) - e_root_tmp * v_0 * t[0]) + 4.0 * *dir *
                  b_t_tmp * this->a_max[i]) + 2.0 * g_root_tmp;
          if (root > 0.0) {
            t[5] = -((((e_root_tmp * t[4] - 2.0 * std::sqrt(root)) + this->
                       j_max[i] * c_root_tmp) - this->j_max[i] * d_root_tmp) +
                     root_tmp * (t[6] * t[6])) / e_root_tmp;
            b_t_tmp = 0.5 * this->j_max[i];
            t[1] = (((((((-v_0 - a_0 * t[0]) - b_t_tmp * (t[0] * t[0])) +
                        b_t_tmp * (t[2] * t[2])) + b_t_tmp * (t[6] * t[6])) -
                      b_t_tmp * (t[4] * t[4])) / this->a_max[i] - t[2]) + t[5])
              + t[4];
            t[3] = 0.0;

            //  Check if phase 2 and/ or phase 6 does not exist
            //  (max velocity and max acceleration cannot be reached)
            if ((t[5] < -0.004) || (t[1] < -0.004)) {
              double d;
              double d1;
              double d2;
              int x_size_idx_0;
              int b_i;
              int trueCount;
              a = this->j_max[i];
              b_a = this->j_max[i];
              c_a = this->j_max[i];
              dv[0] = 12.0;
              dv[1] = 0.0;
              dv[2] = -24.0 * f_root_tmp + 48.0 * this->j_max[i] * v_0;
              d = rt_powd_snf(a_0, 3.0);
              dv[3] = ((48.0 * *dir * (a * a) * varargin_3 - 48.0 * *dir * (b_a *
                         b_a) * varargin_2) + 16.0 * d) - 48.0 * a_0 *
                this->j_max[i] * v_0;
              d1 = rt_powd_snf(a_0, 4.0);
              d2 = 12.0 * f_root_tmp * this->j_max[i] * v_0;
              dv[4] = (-3.0 * d1 + d2) - 12.0 * (c_a * c_a) * g_root_tmp;
              roots(dv, root_data, root_size);

              //  Choose non-complex, positive solution
              x_size_idx_0 = root_size[0];
              loop_ub = root_size[0];
              for (b_i = 0; b_i < loop_ub; b_i++) {
                x_data[b_i] = root_data[b_i].im;
              }

              for (loop_ub = 0; loop_ub < x_size_idx_0; loop_ub++) {
                y_data[loop_ub] = std::abs(x_data[loop_ub]);
              }

              for (b_i = 0; b_i < x_size_idx_0; b_i++) {
                tmp_data[b_i] = (y_data[b_i] < 0.004);
              }

              loop_ub = root_size[0] - 1;
              trueCount = 0;
              x_size_idx_0 = 0;
              for (b_i = 0; b_i <= loop_ub; b_i++) {
                if (tmp_data[b_i]) {
                  trueCount++;
                  root_data[x_size_idx_0] = root_data[b_i];
                  x_size_idx_0++;
                }
              }

              loop_ub = trueCount - 1;
              x_size_idx_0 = 0;
              for (b_i = 0; b_i <= loop_ub; b_i++) {
                if (root_data[b_i].re >= 0.0) {
                  root_data[x_size_idx_0] = root_data[b_i];
                  x_size_idx_0++;
                }
              }

              q_diff = root_data[0].re * root_data[0].im;
              b_a = ((2.0 * (root_data[0].re * root_data[0].re - root_data[0].im
                             * root_data[0].im) - 4.0 * a_0 * root_data[0].re) +
                     f_root_tmp) - 2.0 * v_0 * this->j_max[i];
              root = 2.0 * (q_diff + q_diff) - 4.0 * a_0 * root_data[0].im;
              e_root_tmp = 4.0 * this->j_max[i];
              c_root_tmp = e_root_tmp * root_data[0].re;
              d_root_tmp = e_root_tmp * root_data[0].im;
              if (d_root_tmp == 0.0) {
                if (root == 0.0) {
                  q_diff = b_a / c_root_tmp;
                } else if (b_a == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = b_a / c_root_tmp;
                }
              } else if (c_root_tmp == 0.0) {
                if (b_a == 0.0) {
                  q_diff = root / d_root_tmp;
                } else if (root == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = root / d_root_tmp;
                }
              } else {
                q_diff = std::abs(c_root_tmp);
                q_part1_tmp = std::abs(d_root_tmp);
                if (q_diff > q_part1_tmp) {
                  q_diff = d_root_tmp / c_root_tmp;
                  q_diff = (b_a + q_diff * root) / (c_root_tmp + q_diff *
                    d_root_tmp);
                } else if (q_part1_tmp == q_diff) {
                  if (c_root_tmp > 0.0) {
                    c_root_tmp = 0.5;
                  } else {
                    c_root_tmp = -0.5;
                  }

                  if (d_root_tmp > 0.0) {
                    d_root_tmp = 0.5;
                  } else {
                    d_root_tmp = -0.5;
                  }

                  q_diff = (b_a * c_root_tmp + root * d_root_tmp) / q_diff;
                } else {
                  q_diff = c_root_tmp / d_root_tmp;
                  q_diff = (q_diff * b_a + root) / (d_root_tmp + q_diff *
                    c_root_tmp);
                }
              }

              t[0] = q_diff;

              //  Calculate other switch times
              a = this->j_max[i];
              t[6] = std::sqrt(((4.0 * (a * a) * (q_diff * q_diff) + 8.0 * a_0 *
                                 this->j_max[i] * q_diff) + h_root_tmp) +
                               e_root_tmp * v_0) / root_tmp;
              t[4] = (a_0 / this->j_max[i] + q_diff) + t[6];
              t[1] = 0.0;
              t[5] = 0.0;

              //  Check if a_max is exceeded (Phase 2 exists)
              if (a_0 + q_diff * this->j_max[i] > this->a_max[i]) {
                t[0] = (this->a_max[i] - a_0) / this->j_max[i];
                a = this->a_max[i];
                b_a = this->j_max[i];
                c_a = this->j_max[i];
                d_a = this->a_max[i];
                e_a = this->j_max[i];
                f_a = this->j_max[i];
                g_a = this->j_max[i];
                q_diff = this->j_max[i];
                q_part1_tmp = this->j_max[i];
                root = this->j_max[i];
                b_t_tmp = t[0] * t[0];
                c_root_tmp = 72.0 * this->a_max[i] * *dir;
                t[6] = 1.0 / this->j_max[i] * ((this->a_max[i] / 2.0 + std::sqrt
                  (9.0 * (a * a) + 6.0 * std::sqrt(((((((((-12.0 * this->a_max[i]
                  * rt_powd_snf(this->j_max[i], 3.0) * rt_powd_snf(t[0], 3.0) +
                  9.0 * f_root_tmp * (b_a * b_a) * b_t_tmp) - 18.0 * a_0 *
                  this->a_max[i] * (c_a * c_a) * b_t_tmp) + 9.0 * (d_a * d_a) *
                  (e_a * e_a) * b_t_tmp) + 36.0 * a_0 * (f_a * f_a) * t[0] * v_0)
                  - c_root_tmp * (g_a * g_a) * varargin_3) + c_root_tmp *
                  (q_diff * q_diff) * varargin_2) - 36.0 * this->a_max[i] *
                  (q_part1_tmp * q_part1_tmp) * t[0] * v_0) + 3.0 * rt_powd_snf
                  (this->a_max[i], 4.0)) + 36.0 * (root * root) * g_root_tmp)) /
                  6.0) - this->a_max[i]);
                t[4] = t[6] + t_tmp;
                b_t_tmp = 2.0 * this->a_max[i];
                t[1] = -(((((((-this->j_max[i] * (t[4] * t[4]) - root_tmp * t[4]
                               * t[6]) + this->j_max[i] * (t[6] * t[6])) + a_0 *
                             t[0]) + this->a_max[i] * t[0]) + b_t_tmp * t[4]) +
                          b_t_tmp * t[6]) + 2.0 * v_0) / b_t_tmp;
                t[5] = 0.0;
              }

              //  Check if -a_max is exceeded (Phase 6 exists)
              if (t[6] * this->j_max[i] > this->a_max[i]) {
                t[6] = t_tmp;
                a = this->a_max[i];
                b_a = this->j_max[i];
                c_a = this->j_max[i];
                d_a = this->a_max[i];
                e_a = this->a_max[i];
                f_a = this->j_max[i];
                dv[0] = 12.0;
                dv[1] = -24.0 * this->a_max[i];
                dv[2] = (-12.0 * f_root_tmp + 12.0 * (a * a)) + 24.0 *
                  this->j_max[i] * v_0;
                dv[3] = 0.0;
                dv[4] = (((((((24.0 * *dir * (b_a * b_a) * varargin_3 *
                               this->a_max[i] - 24.0 * *dir * (c_a * c_a) *
                               varargin_2 * this->a_max[i]) + 3.0 * d1) + 8.0 *
                             d * this->a_max[i]) + 6.0 * f_root_tmp * (d_a * d_a))
                           - d2) - 24.0 * a_0 * this->j_max[i] * v_0 *
                          this->a_max[i]) - 12.0 * (e_a * e_a) * this->j_max[i] *
                         v_0) + 12.0 * (f_a * f_a) * g_root_tmp;
                roots(dv, root_data, root_size);

                //  Choose non-complex, positive solution
                x_size_idx_0 = root_size[0];
                loop_ub = root_size[0];
                for (b_i = 0; b_i < loop_ub; b_i++) {
                  x_data[b_i] = root_data[b_i].im;
                }

                for (loop_ub = 0; loop_ub < x_size_idx_0; loop_ub++) {
                  y_data[loop_ub] = std::abs(x_data[loop_ub]);
                }

                for (b_i = 0; b_i < x_size_idx_0; b_i++) {
                  tmp_data[b_i] = (y_data[b_i] < 0.004);
                }

                loop_ub = root_size[0] - 1;
                trueCount = 0;
                x_size_idx_0 = 0;
                for (b_i = 0; b_i <= loop_ub; b_i++) {
                  if (tmp_data[b_i]) {
                    trueCount++;
                    root_data[x_size_idx_0] = root_data[b_i];
                    x_size_idx_0++;
                  }
                }

                loop_ub = trueCount - 1;
                trueCount = 0;
                for (b_i = 0; b_i <= loop_ub; b_i++) {
                  if (root_data[b_i].re >= 0.0) {
                    trueCount++;
                  }
                }

                x_size_idx_0 = 0;
                for (b_i = 0; b_i <= loop_ub; b_i++) {
                  if (root_data[b_i].re >= 0.0) {
                    root_data[x_size_idx_0] = root_data[b_i];
                    x_size_idx_0++;
                  }
                }

                root = this->j_max[i];
                q_part1_tmp = this->a_max[i];
                for (b_i = 0; b_i < trueCount; b_i++) {
                  b_a = (root_data[b_i].re - a_0) - q_part1_tmp;
                  if (root_data[b_i].im == 0.0) {
                    q_diff = b_a / root;
                  } else if (b_a == 0.0) {
                    q_diff = 0.0;
                  } else {
                    q_diff = b_a / root;
                  }

                  y_data[b_i] = q_diff;
                }

                b_a = (root_data[0].re - a_0) - this->a_max[i];
                if (root_data[0].im == 0.0) {
                  q_diff = b_a / root;
                } else if (b_a == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = b_a / root;
                }

                t[0] = q_diff;

                //  Calculate other switch times
                t[4] = (a_0 + this->a_max[i]) / this->j_max[i] + y_data[0];
                a = this->j_max[i];
                b_a = this->j_max[i];
                c_a = this->j_max[i];
                d_a = this->a_max[i];
                t_tmp = 2.0 * a_0 * this->j_max[i];
                t[5] = ((((((a * a * (y_data[0] * y_data[0]) + 2.0 * (b_a * b_a)
                             * y_data[0] * t[4]) - c_a * c_a * (t[4] * t[4])) +
                           t_tmp * y_data[0]) + t_tmp * t[4]) - d_a * d_a) +
                        i_root_tmp) / b_root_tmp;
                t[1] = 0.0;
              }

              //  All other times are 0
              t[2] = 0.0;
              t[3] = 0.0;
            }

            guard1 = true;
          } else {
            //  This should hould never occur, only for safety
            for (i = 0; i < 7; i++) {
              t[i] = 0.0;
            }
          }
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      boolean_T y;
      boolean_T exitg1;

      //  Safety checks
      y = false;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub < 7)) {
        if (!(t[loop_ub] < -0.004)) {
          loop_ub++;
        } else {
          y = true;
          exitg1 = true;
        }
      }

      if (y) {
        //  No numeric inaccuracy
        for (i = 0; i < 7; i++) {
          t[i] = 0.0;
        }
      }

      //  Small numeric errors are set to 0
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        if ((0.0 > t[loop_ub]) || rtIsNaN(t[loop_ub])) {
          t[loop_ub] = 0.0;
        }
      }

      //  Calculate absolute times for jerk switches
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        t[loop_ub + 1] += t[loop_ub];
      }
    }
  }
}

//
// OPTSWITCHTIMES Calculate time-optimal jerk swtiches
// Arguments    : double varargin_2
//                double varargin_3
//                double varargin_4
//                double varargin_5
//                double varargin_6
//                const creal_T varargin_7
//                double t[7]
//                double *dir
//                boolean_T *mod_jerk_profile
// Return Type  : void
//
void LTPlanner::c_optSwitchTimes(double varargin_2, double varargin_3, double
  varargin_4, double varargin_5, double varargin_6, const creal_T varargin_7,
  double t[7], double *dir, boolean_T *mod_jerk_profile) const
{
  double v_0;
  double a_0;
  int i;
  double q_diff;
  creal_T q_break;
  creal_T root;
  double t_tmp;
  double dv[5];
  creal_T root_data[4];
  int root_size[1];
  double x_data[4];
  double y_data[4];
  boolean_T tmp_data[4];

  //  Apply transfer parameters
  v_0 = varargin_4;
  a_0 = varargin_5;

  //  v_drive describes the constant velocity in phase 4.
  //  For time-optimal planning, this is v_max.
  //  For time-scaling, v_drive is calculated in timeScaling
  //  Parameters for calculation
  for (i = 0; i < 7; i++) {
    t[i] = 0.0;
  }

  //  Time that is required for one jerk phase
  *mod_jerk_profile = false;

  //  Use the standard jerk profile if not changed during calculations
  //  Check if inputs are in limits
  //  Calculate direction of movement
  this->optBreaking(varargin_4, varargin_5, varargin_6, (&q_diff), (*(double (*)
    [3])&t[0]), dir);
  q_diff = varargin_2 - (varargin_3 + q_diff);
  if (std::abs(q_diff) < 0.004) {
    //  Skip rest if that fulfils scenario
    for (int loop_ub = 0; loop_ub < 6; loop_ub++) {
      t[loop_ub + 1] += t[loop_ub];
    }
  } else {
    int loop_ub;
    boolean_T guard1 = false;
    boolean_T guard2 = false;
    boolean_T guard3 = false;
    double ar_tmp;
    double ar;
    double br;
    *dir = q_diff;
    if (q_diff < 0.0) {
      *dir = -1.0;
    } else if (q_diff > 0.0) {
      *dir = 1.0;
    } else {
      if (q_diff == 0.0) {
        *dir = 0.0;
      }
    }

    //  If goal is in negative direction, map to pos. direction
    if (*dir < 0.0) {
      v_0 = -varargin_4;
      a_0 = -varargin_5;
    }

    //             %% Calculate min. time required per joint to reach goal state 
    //  Check if slowing down is necessary to satisfy v_drive
    q_break.re = 0.0;
    q_break.im = 0.0;
    i = static_cast<int>(varargin_6) - 1;
    guard1 = false;
    guard2 = false;
    guard3 = false;
    if (v_0 + 0.5 * a_0 * std::abs(a_0) / this->j_max[i] > varargin_7.re) {
      *mod_jerk_profile = true;

      //  Get Joint Position after breaking and required times
      root.re = v_0 - varargin_7.re;
      root.im = 0.0 - varargin_7.im;
      this->optBreaking(root, a_0, varargin_6, (&q_break), (*(double (*)[3])&t[0]));
      guard3 = true;
    } else {
      //  Constant max/ min jerk (Phase 1, 3)
      t[0] = (this->a_max[i] - a_0) / this->j_max[i];
      t[2] = this->a_max[i] / this->j_max[i];

      //  Constant acceleration (Phase 2)
      ar_tmp = varargin_7.re - v_0;
      ar = ar_tmp - 0.5 * t[0] * a_0;
      br = this->a_max[i];
      if (varargin_7.im == 0.0) {
        q_diff = ar / br;
      } else if (ar == 0.0) {
        q_diff = 0.0;
      } else {
        q_diff = ar / br;
      }

      t[1] = q_diff - 0.5 * (t[0] + t[2]);

      //  Check if phase 2 does not exist
      //  (max acceleration cannot be reached)
      if (t[1] < -0.004) {
        //  Check if root is positive
        root.re = this->j_max[i] * ar_tmp + 0.5 * (a_0 * a_0);
        root.im = this->j_max[i] * varargin_7.im;
        if (root.re > 0.0) {
          b_sqrt(&root);
          br = this->j_max[i];
          if (root.im == 0.0) {
            q_diff = root.re / br;
          } else if (root.re == 0.0) {
            q_diff = 0.0;
          } else {
            q_diff = root.re / br;
          }

          t[2] = q_diff;
          t[0] = q_diff - a_0 / this->j_max[i];
          t[1] = 0.0;
          guard3 = true;
        } else {
          //  This should hould never occur, only for safety
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        }
      } else {
        guard3 = true;
      }
    }

    if (guard3) {
      //  Constant max/ min jerk (Phase 5, 7)
      t_tmp = this->a_max[i] / this->j_max[i];
      t[4] = t_tmp;
      t[6] = t_tmp;

      //  Constant acceleration (Phase 6)
      br = this->a_max[i];
      if (varargin_7.im == 0.0) {
        q_diff = varargin_7.re / br;
      } else if (varargin_7.re == 0.0) {
        q_diff = 0.0;
      } else {
        q_diff = varargin_7.re / br;
      }

      t[5] = q_diff - 0.5 * (t_tmp + t_tmp);

      //  Check if phase 6 does not exist
      //  (max acceleration cannot be reached)
      if (t[5] < -0.004) {
        //  Check if root is positive
        br = this->j_max[i];
        if (varargin_7.im == 0.0) {
          root.re = varargin_7.re / br;
          root.im = 0.0;
        } else if (varargin_7.re == 0.0) {
          root.re = 0.0;
          root.im = varargin_7.im / br;
        } else {
          root.re = varargin_7.re / br;
          root.im = varargin_7.im / br;
        }

        if (root.re > 0.0) {
          b_sqrt(&root);
          t[4] = root.re;
          t[6] = root.re;
          t[5] = 0.0;
          guard2 = true;
        } else {
          //  This should hould never occur, only for safety
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        }
      } else {
        guard2 = true;
      }
    }

    if (guard2) {
      double q_break_tmp;
      double b_q_break_tmp;

      //  Constant velocity (Phase 4)
      if (*mod_jerk_profile) {
        //  Breaking to satisfy v_drive
        q_diff = (t[0] + t[1]) + t[2];
        q_break.re += varargin_7.re * q_diff;
        q_break.im += varargin_7.im * q_diff;
      } else {
        //  Acceleration to reach v_drive
        q_diff = 0.5 * (t[0] * t[0]);
        q_break_tmp = t[1] + t[2];
        b_q_break_tmp = t[2] * t[2];
        q_break.re = ((v_0 * ((t[0] + t[1]) + t[2]) + a_0 * ((q_diff + t[0] *
          q_break_tmp) + 0.5 * b_q_break_tmp)) + this->j_max[i] *
                      (((0.16666666666666666 * rt_powd_snf(t[0], 3.0) + q_diff *
                         q_break_tmp) - 0.16666666666666666 * rt_powd_snf(t[2],
          3.0)) + 0.5 * t[0] * b_q_break_tmp)) + this->a_max[i] * (0.5 * (t[1] *
          t[1]) + t[1] * t[2]);
        q_break.im = 0.0;
      }

      ar_tmp = varargin_2 - varargin_3;
      ar = (ar_tmp * *dir - q_break.re) - (this->j_max[i] *
        (((0.16666666666666666 * rt_powd_snf(t[6], 3.0) + 0.5 * (t[6] * t[6]) *
           (t[5] + t[4])) - 0.16666666666666666 * rt_powd_snf(t[4], 3.0)) + 0.5 *
         t[6] * (t[4] * t[4])) + this->a_max[i] * (0.5 * (t[5] * t[5]) + t[5] *
        t[4]));
      if (varargin_7.im == 0.0) {
        if (0.0 - q_break.im == 0.0) {
          q_diff = ar / varargin_7.re;
        } else if (ar == 0.0) {
          q_diff = 0.0;
        } else {
          q_diff = ar / varargin_7.re;
        }
      } else if (varargin_7.re == 0.0) {
        if (ar == 0.0) {
          q_diff = (0.0 - q_break.im) / varargin_7.im;
        } else if (0.0 - q_break.im == 0.0) {
          q_diff = 0.0;
        } else {
          q_diff = (0.0 - q_break.im) / varargin_7.im;
        }
      } else {
        q_diff = std::abs(varargin_7.re);
        q_break_tmp = std::abs(varargin_7.im);
        if (q_diff > q_break_tmp) {
          q_diff = varargin_7.im / varargin_7.re;
          q_diff = (ar + q_diff * (0.0 - q_break.im)) / (varargin_7.re + q_diff *
            varargin_7.im);
        } else if (q_break_tmp == q_diff) {
          if (varargin_7.re > 0.0) {
            br = 0.5;
          } else {
            br = -0.5;
          }

          if (varargin_7.im > 0.0) {
            q_break_tmp = 0.5;
          } else {
            q_break_tmp = -0.5;
          }

          q_diff = (ar * br + (0.0 - q_break.im) * q_break_tmp) / q_diff;
        } else {
          q_diff = varargin_7.re / varargin_7.im;
          q_diff = (q_diff * ar + (0.0 - q_break.im)) / (varargin_7.im + q_diff *
            varargin_7.re);
        }
      }

      t[3] = q_diff;

      //  Check if phase 4 does not exist
      //  (max velocity cannot be reached)
      if (q_diff < -0.004) {
        if (*mod_jerk_profile) {
          //  This case is not valid
          //  (is handled in timeScaling function)
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        } else {
          double a;
          double b_a;
          double c_a;
          double d_a;
          double e_a;
          double f_a;
          double root_tmp;
          double b_root_tmp;
          double c_root_tmp;
          double d_root_tmp;
          double e_root_tmp;
          double f_root_tmp;
          double g_root_tmp;
          double h_root_tmp;

          //  Calculate times if max velocity is not reached
          a = this->j_max[i];
          ar = this->j_max[i];
          b_a = this->j_max[i];
          c_a = this->j_max[i];
          d_a = this->j_max[i];
          e_a = this->a_max[i];
          f_a = this->a_max[i];
          root_tmp = 2.0 * this->j_max[i];
          q_diff = rt_powd_snf(t[0], 3.0);
          b_root_tmp = root_tmp * this->a_max[i];
          c_root_tmp = t[2] * t[2];
          br = t[4] * t[4];
          q_break_tmp = t[0] * t[0];
          b_q_break_tmp = 2.0 * a_0 * this->a_max[i];
          d_root_tmp = 4.0 * this->a_max[i];
          e_root_tmp = a_0 * a_0;
          f_root_tmp = v_0 * v_0;
          g_root_tmp = 2.0 * e_root_tmp;
          h_root_tmp = root_tmp * v_0;
          q_diff = ((((((((((((((((((((a * a * rt_powd_snf(t[0], 4.0) / 2.0 - ar
            * ar * rt_powd_snf(t[2], 4.0) / 4.0) + b_a * b_a * c_root_tmp * br /
            2.0) - c_a * c_a * rt_powd_snf(t[4], 4.0) / 4.0) + d_a * d_a *
            rt_powd_snf(t[6], 4.0) / 2.0) + root_tmp * a_0 * q_diff) -
            b_root_tmp * q_diff / 3.0) - b_root_tmp * t[0] * c_root_tmp) +
                                b_root_tmp * rt_powd_snf(t[2], 3.0) / 3.0) +
                               b_root_tmp * rt_powd_snf(t[4], 3.0) / 3.0) -
                              b_root_tmp * br * t[6]) - b_root_tmp * rt_powd_snf
                             (t[6], 3.0) / 3.0) + h_root_tmp * q_break_tmp) +
                           g_root_tmp * q_break_tmp) - b_q_break_tmp *
                          q_break_tmp) - b_q_break_tmp * c_root_tmp) + 4.0 * a_0
                        * v_0 * t[0]) + 2.0 * (e_a * e_a) * c_root_tmp) + 2.0 *
                      (f_a * f_a) * br) - d_root_tmp * v_0 * t[0]) + 4.0 * *dir *
                    ar_tmp * this->a_max[i]) + 2.0 * f_root_tmp;
          if (q_diff > 0.0) {
            t[5] = -((((d_root_tmp * t[4] - 2.0 * std::sqrt(q_diff)) +
                       this->j_max[i] * c_root_tmp) - this->j_max[i] * br) +
                     root_tmp * (t[6] * t[6])) / d_root_tmp;
            br = 0.5 * this->j_max[i];
            t[1] = (((((((-v_0 - a_0 * t[0]) - br * (t[0] * t[0])) + br * (t[2] *
              t[2])) + br * (t[6] * t[6])) - br * (t[4] * t[4])) / this->a_max[i]
                     - t[2]) + t[5]) + t[4];
            t[3] = 0.0;

            //  Check if phase 2 and/ or phase 6 does not exist
            //  (max velocity and max acceleration cannot be reached)
            if ((t[5] < -0.004) || (t[1] < -0.004)) {
              double d;
              double d1;
              int x_size_idx_0;
              int b_i;
              int trueCount;
              a = this->j_max[i];
              ar = this->j_max[i];
              b_a = this->j_max[i];
              dv[0] = 12.0;
              dv[1] = 0.0;
              dv[2] = -24.0 * e_root_tmp + 48.0 * this->j_max[i] * v_0;
              ar_tmp = rt_powd_snf(a_0, 3.0);
              dv[3] = ((48.0 * *dir * (a * a) * varargin_3 - 48.0 * *dir * (ar *
                         ar) * varargin_2) + 16.0 * ar_tmp) - 48.0 * a_0 *
                this->j_max[i] * v_0;
              d = rt_powd_snf(a_0, 4.0);
              d1 = 12.0 * e_root_tmp * this->j_max[i] * v_0;
              dv[4] = (-3.0 * d + d1) - 12.0 * (b_a * b_a) * f_root_tmp;
              roots(dv, root_data, root_size);

              //  Choose non-complex, positive solution
              x_size_idx_0 = root_size[0];
              loop_ub = root_size[0];
              for (b_i = 0; b_i < loop_ub; b_i++) {
                x_data[b_i] = root_data[b_i].im;
              }

              for (loop_ub = 0; loop_ub < x_size_idx_0; loop_ub++) {
                y_data[loop_ub] = std::abs(x_data[loop_ub]);
              }

              for (b_i = 0; b_i < x_size_idx_0; b_i++) {
                tmp_data[b_i] = (y_data[b_i] < 0.004);
              }

              loop_ub = root_size[0] - 1;
              trueCount = 0;
              x_size_idx_0 = 0;
              for (b_i = 0; b_i <= loop_ub; b_i++) {
                if (tmp_data[b_i]) {
                  trueCount++;
                  root_data[x_size_idx_0] = root_data[b_i];
                  x_size_idx_0++;
                }
              }

              loop_ub = trueCount - 1;
              x_size_idx_0 = 0;
              for (b_i = 0; b_i <= loop_ub; b_i++) {
                if (root_data[b_i].re >= 0.0) {
                  root_data[x_size_idx_0] = root_data[b_i];
                  x_size_idx_0++;
                }
              }

              q_diff = root_data[0].re * root_data[0].im;
              ar = ((2.0 * (root_data[0].re * root_data[0].re - root_data[0].im *
                            root_data[0].im) - 4.0 * a_0 * root_data[0].re) +
                    e_root_tmp) - 2.0 * v_0 * this->j_max[i];
              b_q_break_tmp = 2.0 * (q_diff + q_diff) - 4.0 * a_0 * root_data[0]
                .im;
              d_root_tmp = 4.0 * this->j_max[i];
              br = d_root_tmp * root_data[0].re;
              c_root_tmp = d_root_tmp * root_data[0].im;
              if (c_root_tmp == 0.0) {
                if (b_q_break_tmp == 0.0) {
                  q_diff = ar / br;
                } else if (ar == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = ar / br;
                }
              } else if (br == 0.0) {
                if (ar == 0.0) {
                  q_diff = b_q_break_tmp / c_root_tmp;
                } else if (b_q_break_tmp == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = b_q_break_tmp / c_root_tmp;
                }
              } else {
                q_diff = std::abs(br);
                q_break_tmp = std::abs(c_root_tmp);
                if (q_diff > q_break_tmp) {
                  q_diff = c_root_tmp / br;
                  q_diff = (ar + q_diff * b_q_break_tmp) / (br + q_diff *
                    c_root_tmp);
                } else if (q_break_tmp == q_diff) {
                  if (br > 0.0) {
                    br = 0.5;
                  } else {
                    br = -0.5;
                  }

                  if (c_root_tmp > 0.0) {
                    c_root_tmp = 0.5;
                  } else {
                    c_root_tmp = -0.5;
                  }

                  q_diff = (ar * br + b_q_break_tmp * c_root_tmp) / q_diff;
                } else {
                  q_diff = br / c_root_tmp;
                  q_diff = (q_diff * ar + b_q_break_tmp) / (c_root_tmp + q_diff *
                    br);
                }
              }

              t[0] = q_diff;

              //  Calculate other switch times
              a = this->j_max[i];
              t[6] = std::sqrt(((4.0 * (a * a) * (q_diff * q_diff) + 8.0 * a_0 *
                                 this->j_max[i] * q_diff) + g_root_tmp) +
                               d_root_tmp * v_0) / root_tmp;
              t[4] = (a_0 / this->j_max[i] + q_diff) + t[6];
              t[1] = 0.0;
              t[5] = 0.0;

              //  Check if a_max is exceeded (Phase 2 exists)
              if (a_0 + q_diff * this->j_max[i] > this->a_max[i]) {
                t[0] = (this->a_max[i] - a_0) / this->j_max[i];
                a = this->a_max[i];
                ar = this->j_max[i];
                b_a = this->j_max[i];
                c_a = this->a_max[i];
                d_a = this->j_max[i];
                e_a = this->j_max[i];
                f_a = this->j_max[i];
                q_diff = this->j_max[i];
                q_break_tmp = this->j_max[i];
                b_q_break_tmp = this->j_max[i];
                br = t[0] * t[0];
                c_root_tmp = 72.0 * this->a_max[i] * *dir;
                t[6] = 1.0 / this->j_max[i] * ((this->a_max[i] / 2.0 + std::sqrt
                  (9.0 * (a * a) + 6.0 * std::sqrt(((((((((-12.0 * this->a_max[i]
                  * rt_powd_snf(this->j_max[i], 3.0) * rt_powd_snf(t[0], 3.0) +
                  9.0 * e_root_tmp * (ar * ar) * br) - 18.0 * a_0 * this->
                  a_max[i] * (b_a * b_a) * br) + 9.0 * (c_a * c_a) * (d_a * d_a)
                  * br) + 36.0 * a_0 * (e_a * e_a) * t[0] * v_0) - c_root_tmp *
                  (f_a * f_a) * varargin_3) + c_root_tmp * (q_diff * q_diff) *
                  varargin_2) - 36.0 * this->a_max[i] * (q_break_tmp *
                  q_break_tmp) * t[0] * v_0) + 3.0 * rt_powd_snf(this->a_max[i],
                  4.0)) + 36.0 * (b_q_break_tmp * b_q_break_tmp) * f_root_tmp)) /
                  6.0) - this->a_max[i]);
                t[4] = t[6] + t_tmp;
                br = 2.0 * this->a_max[i];
                t[1] = -(((((((-this->j_max[i] * (t[4] * t[4]) - root_tmp * t[4]
                               * t[6]) + this->j_max[i] * (t[6] * t[6])) + a_0 *
                             t[0]) + this->a_max[i] * t[0]) + br * t[4]) + br *
                          t[6]) + 2.0 * v_0) / br;
                t[5] = 0.0;
              }

              //  Check if -a_max is exceeded (Phase 6 exists)
              if (t[6] * this->j_max[i] > this->a_max[i]) {
                t[6] = t_tmp;
                a = this->a_max[i];
                ar = this->j_max[i];
                b_a = this->j_max[i];
                c_a = this->a_max[i];
                d_a = this->a_max[i];
                e_a = this->j_max[i];
                dv[0] = 12.0;
                dv[1] = -24.0 * this->a_max[i];
                dv[2] = (-12.0 * e_root_tmp + 12.0 * (a * a)) + 24.0 *
                  this->j_max[i] * v_0;
                dv[3] = 0.0;
                dv[4] = (((((((24.0 * *dir * (ar * ar) * varargin_3 *
                               this->a_max[i] - 24.0 * *dir * (b_a * b_a) *
                               varargin_2 * this->a_max[i]) + 3.0 * d) + 8.0 *
                             ar_tmp * this->a_max[i]) + 6.0 * e_root_tmp * (c_a *
                  c_a)) - d1) - 24.0 * a_0 * this->j_max[i] * v_0 * this->
                          a_max[i]) - 12.0 * (d_a * d_a) * this->j_max[i] * v_0)
                  + 12.0 * (e_a * e_a) * f_root_tmp;
                roots(dv, root_data, root_size);

                //  Choose non-complex, positive solution
                x_size_idx_0 = root_size[0];
                loop_ub = root_size[0];
                for (b_i = 0; b_i < loop_ub; b_i++) {
                  x_data[b_i] = root_data[b_i].im;
                }

                for (loop_ub = 0; loop_ub < x_size_idx_0; loop_ub++) {
                  y_data[loop_ub] = std::abs(x_data[loop_ub]);
                }

                for (b_i = 0; b_i < x_size_idx_0; b_i++) {
                  tmp_data[b_i] = (y_data[b_i] < 0.004);
                }

                loop_ub = root_size[0] - 1;
                trueCount = 0;
                x_size_idx_0 = 0;
                for (b_i = 0; b_i <= loop_ub; b_i++) {
                  if (tmp_data[b_i]) {
                    trueCount++;
                    root_data[x_size_idx_0] = root_data[b_i];
                    x_size_idx_0++;
                  }
                }

                loop_ub = trueCount - 1;
                trueCount = 0;
                for (b_i = 0; b_i <= loop_ub; b_i++) {
                  if (root_data[b_i].re >= 0.0) {
                    trueCount++;
                  }
                }

                x_size_idx_0 = 0;
                for (b_i = 0; b_i <= loop_ub; b_i++) {
                  if (root_data[b_i].re >= 0.0) {
                    root_data[x_size_idx_0] = root_data[b_i];
                    x_size_idx_0++;
                  }
                }

                q_break_tmp = this->j_max[i];
                q_break.re = this->a_max[i];
                for (b_i = 0; b_i < trueCount; b_i++) {
                  ar = (root_data[b_i].re - a_0) - q_break.re;
                  if (root_data[b_i].im == 0.0) {
                    q_diff = ar / q_break_tmp;
                  } else if (ar == 0.0) {
                    q_diff = 0.0;
                  } else {
                    q_diff = ar / q_break_tmp;
                  }

                  y_data[b_i] = q_diff;
                }

                ar = (root_data[0].re - a_0) - this->a_max[i];
                if (root_data[0].im == 0.0) {
                  q_diff = ar / q_break_tmp;
                } else if (ar == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = ar / q_break_tmp;
                }

                t[0] = q_diff;

                //  Calculate other switch times
                t[4] = (a_0 + this->a_max[i]) / this->j_max[i] + y_data[0];
                a = this->j_max[i];
                ar = this->j_max[i];
                b_a = this->j_max[i];
                c_a = this->a_max[i];
                t_tmp = 2.0 * a_0 * this->j_max[i];
                t[5] = ((((((a * a * (y_data[0] * y_data[0]) + 2.0 * (ar * ar) *
                             y_data[0] * t[4]) - b_a * b_a * (t[4] * t[4])) +
                           t_tmp * y_data[0]) + t_tmp * t[4]) - c_a * c_a) +
                        h_root_tmp) / b_root_tmp;
                t[1] = 0.0;
              }

              //  All other times are 0
              t[2] = 0.0;
              t[3] = 0.0;
            }

            guard1 = true;
          } else {
            //  This should hould never occur, only for safety
            for (i = 0; i < 7; i++) {
              t[i] = 0.0;
            }
          }
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      boolean_T y;
      boolean_T exitg1;

      //  Safety checks
      y = false;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub < 7)) {
        if (!(t[loop_ub] < -0.004)) {
          loop_ub++;
        } else {
          y = true;
          exitg1 = true;
        }
      }

      if (y) {
        //  No numeric inaccuracy
        for (i = 0; i < 7; i++) {
          t[i] = 0.0;
        }
      }

      //  Small numeric errors are set to 0
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        if ((0.0 > t[loop_ub]) || rtIsNaN(t[loop_ub])) {
          t[loop_ub] = 0.0;
        }
      }

      //  Calculate absolute times for jerk switches
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        t[loop_ub + 1] += t[loop_ub];
      }
    }
  }
}

//
// GETTRAJECTORIES % Calculate trajectory based on jerk switch
//  times
// Arguments    : const coder::array<double, 2U> &varargin_2
//                const coder::array<double, 1U> &varargin_3
//                const coder::array<boolean_T, 1U> &varargin_4
//                const double varargin_5[6]
//                const double varargin_6[6]
//                const double varargin_7[6]
//                const double varargin_8[6]
//                double q_traj[6]
//                double v_traj[6]
//                double a_traj[6]
// Return Type  : void
//
void LTPlanner::getTrajectories(const coder::array<double, 2U> &varargin_2,
  const coder::array<double, 1U> &varargin_3, const coder::array<boolean_T, 1U>
  &varargin_4, const double varargin_5[6], const double varargin_6[6], const
  double varargin_7[6], const double varargin_8[6], double q_traj[6], double
  v_traj[6], double a_traj[6]) const
{
  int loop_ub;
  coder::array<boolean_T, 1U> const_v;
  int i;
  int nx;
  coder::array<double, 1U> x;
  int k;
  int n;
  double ex;
  coder::array<double, 2U> sampled_t;
  double r;
  coder::array<double, 2U> j_traj;
  double y;
  int i1;
  double jerk_profile[7];
  static const signed char b[7] = { 1, 0, -1, 0, -1, 0, 1 };

  static const signed char b_b[7] = { -1, 0, 1, 0, -1, 0, 1 };

  double sampled_t_trans[7];

  //  Apply transfer parameters
  loop_ub = static_cast<int>(this->DoF);
  const_v.set_size(loop_ub);
  for (i = 0; i < loop_ub; i++) {
    const_v[i] = false;
  }

  //  Calculate length of trajectory in samples
  nx = varargin_2.size(0);
  x.set_size(varargin_2.size(0));
  for (i = 0; i < nx; i++) {
    x[i] = varargin_2[i + varargin_2.size(0) * 6] / this->Tsample;
  }

  nx = x.size(0);
  for (k = 0; k < nx; k++) {
    x[k] = std::ceil(x[k]);
  }

  n = x.size(0);
  if (x.size(0) <= 2) {
    if (x.size(0) == 1) {
      ex = x[0];
    } else if ((x[0] < x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
      ex = x[1];
    } else {
      ex = x[0];
    }
  } else {
    if (!rtIsNaN(x[0])) {
      nx = 1;
    } else {
      boolean_T exitg1;
      nx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= x.size(0))) {
        if (!rtIsNaN(x[k - 1])) {
          nx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (nx == 0) {
      ex = x[0];
    } else {
      ex = x[nx - 1];
      i = nx + 1;
      for (k = i; k <= n; k++) {
        r = x[k - 1];
        if (ex < r) {
          ex = r;
        }
      }
    }
  }

  //  Jerk switch times in samples
  sampled_t.set_size(loop_ub, 7);
  nx = loop_ub * 7;
  for (i = 0; i < nx; i++) {
    sampled_t[i] = 0.0;
  }

  //             %% Calculate jerk trajectories
  i = static_cast<int>(ex + 1.0);
  j_traj.set_size(loop_ub, i);
  nx = loop_ub * i;
  for (i = 0; i < nx; i++) {
    j_traj[i] = 0.0;
  }

  if (0 <= loop_ub - 1) {
    y = this->Tsample;
  }

  for (k = 0; k < loop_ub; k++) {
    //  Save fractions lost when discretizing switch times
    //  Check which jerk profile to use
    if (varargin_4[k]) {
      //  Only used for specific scenarios in time scaling
      ex = varargin_3[k] * this->j_max[k];
      for (i = 0; i < 7; i++) {
        jerk_profile[i] = ex * static_cast<double>(b_b[i]);
      }
    } else {
      //  Standard case
      ex = varargin_3[k] * this->j_max[k];
      for (i = 0; i < 7; i++) {
        jerk_profile[i] = ex * static_cast<double>(b[i]);
      }
    }

    //  Calculate inaccuraties when sampling times
    for (n = 0; n < 7; n++) {
      ex = varargin_2[k + varargin_2.size(0) * n];
      r = ex;
      if (y == 0.0) {
        if (ex == 0.0) {
          r = y;
        }
      } else if (rtIsNaN(ex) || rtIsNaN(y) || rtIsInf(ex)) {
        r = rtNaN;
      } else if (ex == 0.0) {
        r = 0.0 / y;
      } else if (rtIsInf(y)) {
        if ((y < 0.0) != (ex < 0.0)) {
          r = y;
        }
      } else {
        boolean_T rEQ0;
        r = std::fmod(ex, y);
        rEQ0 = (r == 0.0);
        if ((!rEQ0) && (y > std::floor(y))) {
          double q;
          q = std::abs(ex / y);
          rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 *
                   q);
        }

        if (rEQ0) {
          r = y * 0.0;
        } else {
          if ((ex < 0.0) != (y < 0.0)) {
            r += y;
          }
        }
      }

      sampled_t_trans[n] = r;
    }

    //  Round towards phases with zero jerk
    sampled_t[k] = std::floor(varargin_2[k] / this->Tsample);
    sampled_t[k + sampled_t.size(0)] = std::ceil(varargin_2[k + varargin_2.size
      (0)] / this->Tsample);
    sampled_t[k + sampled_t.size(0) * 2] = std::floor(varargin_2[k +
      varargin_2.size(0) * 2] / this->Tsample);
    sampled_t[k + sampled_t.size(0) * 3] = std::ceil(varargin_2[k +
      varargin_2.size(0) * 3] / this->Tsample);
    sampled_t[k + sampled_t.size(0) * 4] = std::floor(varargin_2[k +
      varargin_2.size(0) * 4] / this->Tsample);
    sampled_t[k + sampled_t.size(0) * 5] = std::ceil(varargin_2[k +
      varargin_2.size(0) * 5] / this->Tsample);
    sampled_t[k + sampled_t.size(0) * 6] = std::floor(varargin_2[k +
      varargin_2.size(0) * 6] / this->Tsample);

    //  Calculate sampled jerk trajectory
    if (sampled_t[k] > 0.0) {
      r = sampled_t[k];
      if (1.0 > r) {
        nx = 0;
      } else {
        nx = static_cast<int>(r);
      }

      for (i = 0; i < nx; i++) {
        j_traj[k + j_traj.size(0) * i] = jerk_profile[0];
      }
    }

    for (n = 0; n < 6; n++) {
      r = sampled_t[k + sampled_t.size(0) * (n + 1)];
      ex = sampled_t[k + sampled_t.size(0) * n];
      if (r - ex > 0.0) {
        if (ex + 1.0 > r) {
          i = 0;
          i1 = 0;
        } else {
          i = static_cast<int>(ex + 1.0) - 1;
          i1 = static_cast<int>(r);
        }

        nx = i1 - i;
        for (i1 = 0; i1 < nx; i1++) {
          j_traj[k + j_traj.size(0) * (i + i1)] = jerk_profile[n + 1];
        }
      }
    }

    //                 %% Add partial jerk of sample fractions to increase accuracy 
    if (sampled_t[k + sampled_t.size(0) * 2] >= sampled_t[k + sampled_t.size(0)])
    {
      //  Phase 2 exists:
      //  Fractions can be added to its beginning and end
      nx = static_cast<int>(sampled_t[k] + 1.0) - 1;
      j_traj[k + j_traj.size(0) * nx] = j_traj[k + j_traj.size(0) * nx] +
        sampled_t_trans[0] / this->Tsample * jerk_profile[0];
      if (sampled_t[k + sampled_t.size(0)] > 0.0) {
        nx = static_cast<int>(sampled_t[k + sampled_t.size(0)]) - 1;
        j_traj[k + j_traj.size(0) * nx] = j_traj[k + j_traj.size(0) * nx] + (1.0
          - sampled_t_trans[1] / this->Tsample) * jerk_profile[2];
      }

      //  Add fraction to end of phase 3
      nx = static_cast<int>(sampled_t[k + sampled_t.size(0) * 2] + 1.0) - 1;
      j_traj[k + j_traj.size(0) * nx] = j_traj[k + j_traj.size(0) * nx] +
        sampled_t_trans[2] / this->Tsample * jerk_profile[2];
    } else {
      //  Phase 2 does not exist:
      //  Calculate transition sample
      if (sampled_t[k + sampled_t.size(0)] > 0.0) {
        nx = static_cast<int>(sampled_t[k + sampled_t.size(0)]) - 1;
        j_traj[k + j_traj.size(0) * nx] = (j_traj[k + j_traj.size(0) * nx] +
          sampled_t_trans[0] / this->Tsample * jerk_profile[0]) +
          (sampled_t_trans[2] - sampled_t_trans[0]) / this->Tsample *
          jerk_profile[2];
      }
    }

    //  Add fraction to end of phase 4
    if (sampled_t[k + sampled_t.size(0) * 3] > 0.0) {
      nx = static_cast<int>(sampled_t[k + sampled_t.size(0) * 3]) - 1;
      j_traj[k + j_traj.size(0) * nx] = j_traj[k + j_traj.size(0) * nx] + (1.0 -
        sampled_t_trans[3] / this->Tsample) * jerk_profile[4];
    }

    if (sampled_t[k + sampled_t.size(0) * 2] - sampled_t[k] > 0.0) {
      //  Phase 2 and/ or 3 exist:
      //  Add fraction to beginning of phase 6
      nx = static_cast<int>(sampled_t[k + sampled_t.size(0) * 4] + 1.0) - 1;
      j_traj[k + j_traj.size(0) * nx] = j_traj[k + j_traj.size(0) * nx] +
        sampled_t_trans[4] / this->Tsample * jerk_profile[4];
    } else {
      //  Phase 2 and 3 do not exist:
      //  Add fractions from previous phases to end of phase 5
      if (sampled_t[k + sampled_t.size(0) * 4] > 0.0) {
        nx = static_cast<int>(sampled_t[k + sampled_t.size(0) * 4]) - 1;
        j_traj[k + j_traj.size(0) * nx] = ((j_traj[k + j_traj.size(0) * nx] +
          sampled_t_trans[4] / this->Tsample * jerk_profile[4]) +
          sampled_t_trans[0] / this->Tsample * jerk_profile[0]) +
          (sampled_t_trans[2] - sampled_t_trans[0]) / this->Tsample *
          jerk_profile[2];
      }
    }

    //  Add fraction to end of phase 4
    if (sampled_t[k + sampled_t.size(0) * 5] > 0.0) {
      nx = static_cast<int>(sampled_t[k + sampled_t.size(0) * 5]) - 1;
      j_traj[k + j_traj.size(0) * nx] = j_traj[k + j_traj.size(0) * nx] + (1.0 -
        sampled_t_trans[5] / this->Tsample) * jerk_profile[6];
    }

    //  Add fraction to end of trajectory (after phase 7)
    nx = static_cast<int>(sampled_t[k + sampled_t.size(0) * 6] + 1.0) - 1;
    j_traj[k + j_traj.size(0) * nx] = j_traj[k + j_traj.size(0) * nx] +
      sampled_t_trans[6] / this->Tsample * jerk_profile[6];

    //  Check if phase 4 exists (constant velocity)
    const_v.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
      const_v[i] = false;
    }

    if (sampled_t[k + sampled_t.size(0) * 3] - sampled_t[k + sampled_t.size(0) *
        2] > 2.0) {
      const_v[k] = true;
    }
  }

  //             %% Calculate acceleration trajectories
  if ((j_traj.size(0) != 0) && (j_traj.size(1) != 0)) {
    i = j_traj.size(1);
    for (k = 0; k <= i - 2; k++) {
      i1 = j_traj.size(0);
      for (nx = 0; nx < i1; nx++) {
        j_traj[nx + j_traj.size(0) * (k + 1)] = j_traj[nx + j_traj.size(0) * k]
          + j_traj[nx + j_traj.size(0) * (k + 1)];
      }
    }
  }

  ex = this->Tsample;

  //             %% Calculate velocitie trajectories
  for (i = 0; i < 6; i++) {
    r = ex * j_traj[i] + varargin_7[i];
    a_traj[i] = r;
    v_traj[i] = r;
  }

  for (k = 0; k < 5; k++) {
    v_traj[k + 1] += v_traj[k];
  }

  ex = this->Tsample;
  for (i = 0; i < 6; i++) {
    v_traj[i] = ex * v_traj[i] + varargin_6[i];
  }

  //  Set constant velocity periods to exactly v_drive
  //  (increases accuracy)
  for (k = 0; k < loop_ub; k++) {
    if (const_v[k]) {
      r = sampled_t[k + sampled_t.size(0) * 2] + 1.0;
      ex = sampled_t[k + sampled_t.size(0) * 3] - 1.0;
      if (r > ex) {
        i = 0;
        i1 = 0;
      } else {
        i = static_cast<int>(r) - 1;
        i1 = static_cast<int>(ex);
      }

      nx = i1 - i;
      for (i1 = 0; i1 < nx; i1++) {
        v_traj[i + i1] = varargin_8[k] * varargin_3[k];
      }
    }
  }

  //             %% Calculate joint angle trajectories
  for (i = 0; i < 6; i++) {
    q_traj[i] = v_traj[i];
  }

  for (k = 0; k < 5; k++) {
    q_traj[k + 1] += q_traj[k];
  }

  ex = this->Tsample;
  for (i = 0; i < 6; i++) {
    q_traj[i] = ex * q_traj[i] + varargin_5[i];
  }
}

//
// OPTBREAKING % Calculate time and joint angles required to
//  bring velocity to zero
// Arguments    : creal_T v_0
//                double a_0
//                double joint
//                creal_T *q
//                double t_rel[3]
// Return Type  : void
//
void LTPlanner::optBreaking(creal_T v_0, double a_0, double joint, creal_T *q,
  double t_rel[3]) const
{
  creal_T dc;
  double dir_re;
  double x;
  double dir_im;
  int t_rel_tmp;
  double br;
  double v_0_re;
  double a;
  double v_0_re_tmp;

  //  This function can be used to:
  //  - bring a joint to a full stop as fast as possible
  //  - calculate in which direction a joint has to be actuated to
  //    reach a goal
  //  - slow a joint down to a desired velocity (v_0 must be set to
  //    v_current - v_desired)
  //  Set direction opposite to maximal velocity to be reached
  if (v_0.re * a_0 > 0.0) {
    //  v and a in same direction
    dc = v_0;
    b_sign(&dc);
    dir_re = -dc.re;
    dir_im = -dc.im;
  } else {
    //  If initial acceleration will cause the robot to
    //  eventually move into opposite direction of velocity, use
    //  this direction
    if (rt_hypotd_snf(v_0.re, v_0.im) > 0.5 * (a_0 * a_0) / this->j_max[
        static_cast<int>(joint) - 1]) {
      dc = v_0;
      b_sign(&dc);
      dir_re = -dc.re;
      dir_im = -dc.im;
    } else {
      x = a_0;
      if (a_0 < 0.0) {
        x = -1.0;
      } else if (a_0 > 0.0) {
        x = 1.0;
      } else {
        if (a_0 == 0.0) {
          x = 0.0;
        }
      }

      dir_re = -x;
      dir_im = 0.0;
    }
  }

  //  If stopping dir. is negative, map scenario to pos. direction
  if (dir_re < 0.0) {
    a_0 = -a_0;
    v_0.re = -v_0.re;
    v_0.im = -v_0.im;
  }

  //  Bring velocity to zero
  t_rel_tmp = static_cast<int>(joint) - 1;
  t_rel[0] = (this->a_max[t_rel_tmp] - a_0) / this->j_max[t_rel_tmp];
  t_rel[2] = this->a_max[t_rel_tmp] / this->j_max[t_rel_tmp];
  x = -v_0.re - 0.5 * t_rel[0] * a_0;
  br = this->a_max[t_rel_tmp];
  if (-v_0.im == 0.0) {
    v_0_re = x / br;
  } else if (x == 0.0) {
    v_0_re = 0.0;
  } else {
    v_0_re = x / br;
  }

  t_rel[1] = v_0_re - 0.5 * (t_rel[0] + t_rel[2]);

  //  Check if phase 2 does not exist
  //  (max acceleration is not reached)
  if (t_rel[1] < -this->Tsample) {
    a = this->j_max[t_rel_tmp];
    br = this->j_max[t_rel_tmp];
    if (v_0.im == 0.0) {
      v_0_re = v_0.re / br;
      x = 0.0;
    } else if (v_0.re == 0.0) {
      v_0_re = 0.0;
      x = v_0.im / br;
    } else {
      v_0_re = v_0.re / br;
      x = v_0.im / br;
    }

    dc.re = a_0 * a_0 / (2.0 * (a * a)) - v_0_re;
    dc.im = 0.0 - x;
    b_sqrt(&dc);
    t_rel[0] = -a_0 / this->j_max[t_rel_tmp] + dc.re;
    t_rel[2] = t_rel[0] + a_0 / this->j_max[t_rel_tmp];
    t_rel[1] = 0.0;
  }

  //  Calculate position after breaking
  //  Correct direction
  x = 0.5 * (t_rel[0] * t_rel[0]);
  br = t_rel[1] + t_rel[2];
  a = t_rel[2] * t_rel[2];
  v_0_re_tmp = (t_rel[0] + t_rel[1]) + t_rel[2];
  v_0_re = ((v_0.re * v_0_re_tmp + a_0 * ((x + t_rel[0] * br) + 0.5 * a)) +
            this->j_max[t_rel_tmp] * (((0.16666666666666666 * rt_powd_snf(t_rel
    [0], 3.0) + x * br) - 0.16666666666666666 * rt_powd_snf(t_rel[2], 3.0)) +
             0.5 * t_rel[0] * a)) + this->a_max[t_rel_tmp] * (0.5 * (t_rel[1] *
    t_rel[1]) + t_rel[1] * t_rel[2]);
  x = v_0.im * v_0_re_tmp;
  q->re = dir_re * v_0_re - dir_im * x;
  q->im = dir_re * x + dir_im * v_0_re;
}

//
// OPTBREAKING % Calculate time and joint angles required to
//  bring velocity to zero
// Arguments    : double v_0
//                double a_0
//                double joint
//                double *q
//                double t_rel[3]
//                double *dir
// Return Type  : void
//
void LTPlanner::optBreaking(double v_0, double a_0, double joint, double *q,
  double t_rel[3], double *dir) const
{
  double x;
  int t_rel_tmp;
  double q_tmp;
  double b_q_tmp;

  //  This function can be used to:
  //  - bring a joint to a full stop as fast as possible
  //  - calculate in which direction a joint has to be actuated to
  //    reach a goal
  //  - slow a joint down to a desired velocity (v_0 must be set to
  //    v_current - v_desired)
  //  Set direction opposite to maximal velocity to be reached
  if (v_0 * a_0 > 0.0) {
    //  v and a in same direction
    x = v_0;
    if (v_0 < 0.0) {
      x = -1.0;
    } else if (v_0 > 0.0) {
      x = 1.0;
    } else {
      if (v_0 == 0.0) {
        x = 0.0;
      }
    }

    *dir = -x;
  } else {
    //  If initial acceleration will cause the robot to
    //  eventually move into opposite direction of velocity, use
    //  this direction
    if (std::abs(v_0) > 0.5 * (a_0 * a_0) / this->j_max[static_cast<int>(joint)
        - 1]) {
      x = v_0;
      if (v_0 < 0.0) {
        x = -1.0;
      } else if (v_0 > 0.0) {
        x = 1.0;
      } else {
        if (v_0 == 0.0) {
          x = 0.0;
        }
      }

      *dir = -x;
    } else {
      x = a_0;
      if (a_0 < 0.0) {
        x = -1.0;
      } else if (a_0 > 0.0) {
        x = 1.0;
      } else {
        if (a_0 == 0.0) {
          x = 0.0;
        }
      }

      *dir = -x;
    }
  }

  //  If stopping dir. is negative, map scenario to pos. direction
  if (*dir < 0.0) {
    a_0 = -a_0;
    v_0 = -v_0;
  }

  //  Bring velocity to zero
  t_rel_tmp = static_cast<int>(joint) - 1;
  t_rel[0] = (this->a_max[t_rel_tmp] - a_0) / this->j_max[t_rel_tmp];
  t_rel[2] = this->a_max[t_rel_tmp] / this->j_max[t_rel_tmp];
  t_rel[1] = (-v_0 - 0.5 * t_rel[0] * a_0) / this->a_max[t_rel_tmp] - 0.5 *
    (t_rel[0] + t_rel[2]);

  //  Check if phase 2 does not exist
  //  (max acceleration is not reached)
  if (t_rel[1] < -this->Tsample) {
    x = this->j_max[t_rel_tmp];
    t_rel[0] = -a_0 / this->j_max[t_rel_tmp] + std::sqrt(a_0 * a_0 / (2.0 * (x *
      x)) - v_0 / this->j_max[t_rel_tmp]);
    t_rel[2] = t_rel[0] + a_0 / this->j_max[t_rel_tmp];
    t_rel[1] = 0.0;
  }

  //  Calculate position after breaking
  //  Correct direction
  x = 0.5 * (t_rel[0] * t_rel[0]);
  q_tmp = t_rel[1] + t_rel[2];
  b_q_tmp = t_rel[2] * t_rel[2];
  *q = *dir * (((v_0 * ((t_rel[0] + t_rel[1]) + t_rel[2]) + a_0 * ((x + t_rel[0]
    * q_tmp) + 0.5 * b_q_tmp)) + this->j_max[t_rel_tmp] * (((0.16666666666666666
    * rt_powd_snf(t_rel[0], 3.0) + x * q_tmp) - 0.16666666666666666 *
    rt_powd_snf(t_rel[2], 3.0)) + 0.5 * t_rel[0] * b_q_tmp)) + this->
               a_max[t_rel_tmp] * (0.5 * (t_rel[1] * t_rel[1]) + t_rel[1] *
    t_rel[2]));
}

//
// OPTBREAKING % Calculate time and joint angles required to
//  bring velocity to zero
// Arguments    : double v_0
//                double a_0
//                double joint
//                double *q
//                double t_rel[3]
// Return Type  : void
//
void LTPlanner::optBreaking(double v_0, double a_0, double joint, double *q,
  double t_rel[3]) const
{
  double x;
  double dir;
  int t_rel_tmp;
  double q_tmp;
  double b_q_tmp;

  //  This function can be used to:
  //  - bring a joint to a full stop as fast as possible
  //  - calculate in which direction a joint has to be actuated to
  //    reach a goal
  //  - slow a joint down to a desired velocity (v_0 must be set to
  //    v_current - v_desired)
  //  Set direction opposite to maximal velocity to be reached
  if (v_0 * a_0 > 0.0) {
    //  v and a in same direction
    x = v_0;
    if (v_0 < 0.0) {
      x = -1.0;
    } else if (v_0 > 0.0) {
      x = 1.0;
    } else {
      if (v_0 == 0.0) {
        x = 0.0;
      }
    }

    dir = -x;
  } else {
    //  If initial acceleration will cause the robot to
    //  eventually move into opposite direction of velocity, use
    //  this direction
    if (std::abs(v_0) > 0.5 * (a_0 * a_0) / this->j_max[static_cast<int>(joint)
        - 1]) {
      x = v_0;
      if (v_0 < 0.0) {
        x = -1.0;
      } else if (v_0 > 0.0) {
        x = 1.0;
      } else {
        if (v_0 == 0.0) {
          x = 0.0;
        }
      }

      dir = -x;
    } else {
      x = a_0;
      if (a_0 < 0.0) {
        x = -1.0;
      } else if (a_0 > 0.0) {
        x = 1.0;
      } else {
        if (a_0 == 0.0) {
          x = 0.0;
        }
      }

      dir = -x;
    }
  }

  //  If stopping dir. is negative, map scenario to pos. direction
  if (dir < 0.0) {
    a_0 = -a_0;
    v_0 = -v_0;
  }

  //  Bring velocity to zero
  t_rel_tmp = static_cast<int>(joint) - 1;
  t_rel[0] = (this->a_max[t_rel_tmp] - a_0) / this->j_max[t_rel_tmp];
  t_rel[2] = this->a_max[t_rel_tmp] / this->j_max[t_rel_tmp];
  t_rel[1] = (-v_0 - 0.5 * t_rel[0] * a_0) / this->a_max[t_rel_tmp] - 0.5 *
    (t_rel[0] + t_rel[2]);

  //  Check if phase 2 does not exist
  //  (max acceleration is not reached)
  if (t_rel[1] < -this->Tsample) {
    x = this->j_max[t_rel_tmp];
    t_rel[0] = -a_0 / this->j_max[t_rel_tmp] + std::sqrt(a_0 * a_0 / (2.0 * (x *
      x)) - v_0 / this->j_max[t_rel_tmp]);
    t_rel[2] = t_rel[0] + a_0 / this->j_max[t_rel_tmp];
    t_rel[1] = 0.0;
  }

  //  Calculate position after breaking
  //  Correct direction
  x = 0.5 * (t_rel[0] * t_rel[0]);
  q_tmp = t_rel[1] + t_rel[2];
  b_q_tmp = t_rel[2] * t_rel[2];
  *q = dir * (((v_0 * ((t_rel[0] + t_rel[1]) + t_rel[2]) + a_0 * ((x + t_rel[0] *
    q_tmp) + 0.5 * b_q_tmp)) + this->j_max[t_rel_tmp] * (((0.16666666666666666 *
    rt_powd_snf(t_rel[0], 3.0) + x * q_tmp) - 0.16666666666666666 * rt_powd_snf
    (t_rel[2], 3.0)) + 0.5 * t_rel[0] * b_q_tmp)) + this->a_max[t_rel_tmp] *
              (0.5 * (t_rel[1] * t_rel[1]) + t_rel[1] * t_rel[2]));
}

//
// OPTSWITCHTIMES Calculate time-optimal jerk swtiches
// Arguments    : double varargin_2
//                double varargin_3
//                double varargin_4
//                double varargin_5
//                double varargin_6
//                double t[7]
//                double *dir
//                boolean_T *mod_jerk_profile
// Return Type  : void
//
void LTPlanner::optSwitchTimes(double varargin_2, double varargin_3, double
  varargin_4, double varargin_5, double varargin_6, double t[7], double *dir,
  boolean_T *mod_jerk_profile) const
{
  double v_0;
  double a_0;
  int v_drive_tmp;
  double v_drive;
  int i;
  double q_diff;
  double t_tmp;
  double dv[5];
  creal_T root_data[4];
  int root_size[1];
  double x_data[4];
  double y_data[4];
  boolean_T tmp_data[4];

  //  Apply transfer parameters
  v_0 = varargin_4;
  a_0 = varargin_5;

  //  v_drive describes the constant velocity in phase 4.
  //  For time-optimal planning, this is v_max.
  //  For time-scaling, v_drive is calculated in timeScaling
  v_drive_tmp = static_cast<int>(varargin_6) - 1;
  v_drive = this->v_max[v_drive_tmp];

  //  Parameters for calculation
  for (i = 0; i < 7; i++) {
    t[i] = 0.0;
  }

  //  Time that is required for one jerk phase
  *mod_jerk_profile = false;

  //  Use the standard jerk profile if not changed during calculations
  //  Check if inputs are in limits
  //  Calculate direction of movement
  this->optBreaking(varargin_4, varargin_5, varargin_6, (&q_diff), (*(double (*)
    [3])&t[0]), dir);
  q_diff = varargin_2 - (varargin_3 + q_diff);
  if (std::abs(q_diff) < 0.004) {
    //  Skip rest if that fulfils scenario
    for (int loop_ub = 0; loop_ub < 6; loop_ub++) {
      t[loop_ub + 1] += t[loop_ub];
    }
  } else {
    int loop_ub;
    boolean_T guard1 = false;
    boolean_T guard2 = false;
    boolean_T guard3 = false;
    double root;
    *dir = q_diff;
    if (q_diff < 0.0) {
      *dir = -1.0;
    } else if (q_diff > 0.0) {
      *dir = 1.0;
    } else {
      if (q_diff == 0.0) {
        *dir = 0.0;
      }
    }

    //  If goal is in negative direction, map to pos. direction
    if (*dir < 0.0) {
      v_0 = -varargin_4;
      a_0 = -varargin_5;
    }

    //             %% Calculate min. time required per joint to reach goal state 
    //  Check if slowing down is necessary to satisfy v_drive
    q_diff = 0.0;
    guard1 = false;
    guard2 = false;
    guard3 = false;
    if (v_0 + 0.5 * a_0 * std::abs(a_0) / this->j_max[v_drive_tmp] > v_drive) {
      *mod_jerk_profile = true;

      //  Get Joint Position after breaking and required times
      this->optBreaking((v_0 - v_drive), a_0, varargin_6, (&q_diff), (*(double (*)
        [3])&t[0]));
      guard3 = true;
    } else {
      //  Constant max/ min jerk (Phase 1, 3)
      t[0] = (this->a_max[v_drive_tmp] - a_0) / this->j_max[v_drive_tmp];
      t[2] = this->a_max[v_drive_tmp] / this->j_max[v_drive_tmp];

      //  Constant acceleration (Phase 2)
      t_tmp = v_drive - v_0;
      t[1] = (t_tmp - 0.5 * t[0] * a_0) / this->a_max[v_drive_tmp] - 0.5 * (t[0]
        + t[2]);

      //  Check if phase 2 does not exist
      //  (max acceleration cannot be reached)
      if (t[1] < -0.004) {
        //  Check if root is positive
        root = this->j_max[v_drive_tmp] * t_tmp + 0.5 * (a_0 * a_0);
        if (root > 0.0) {
          t[2] = std::sqrt(root) / this->j_max[v_drive_tmp];
          t[0] = t[2] - a_0 / this->j_max[v_drive_tmp];
          t[1] = 0.0;
          guard3 = true;
        } else {
          //  This should hould never occur, only for safety
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        }
      } else {
        guard3 = true;
      }
    }

    if (guard3) {
      //  Constant max/ min jerk (Phase 5, 7)
      t_tmp = this->a_max[v_drive_tmp] / this->j_max[v_drive_tmp];
      t[4] = t_tmp;
      t[6] = t_tmp;

      //  Constant acceleration (Phase 6)
      t[5] = v_drive / this->a_max[v_drive_tmp] - 0.5 * (t_tmp + t_tmp);

      //  Check if phase 6 does not exist
      //  (max acceleration cannot be reached)
      if (t[5] < -0.004) {
        //  Check if root is positive
        root = v_drive / this->j_max[v_drive_tmp];
        if (root > 0.0) {
          t[4] = std::sqrt(root);
          t[6] = t[4];
          t[5] = 0.0;
          guard2 = true;
        } else {
          //  This should hould never occur, only for safety
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        }
      } else {
        guard2 = true;
      }
    }

    if (guard2) {
      double q_part1_tmp;
      double b_t_tmp;

      //  Constant velocity (Phase 4)
      if (*mod_jerk_profile) {
        //  Breaking to satisfy v_drive
        q_diff += v_drive * ((t[0] + t[1]) + t[2]);
      } else {
        //  Acceleration to reach v_drive
        q_diff = 0.5 * (t[0] * t[0]);
        root = t[1] + t[2];
        q_part1_tmp = t[2] * t[2];
        q_diff = ((v_0 * ((t[0] + t[1]) + t[2]) + a_0 * ((q_diff + t[0] * root)
                    + 0.5 * q_part1_tmp)) + this->j_max[v_drive_tmp] *
                  (((0.16666666666666666 * rt_powd_snf(t[0], 3.0) + q_diff *
                     root) - 0.16666666666666666 * rt_powd_snf(t[2], 3.0)) + 0.5
                   * t[0] * q_part1_tmp)) + this->a_max[v_drive_tmp] * (0.5 *
          (t[1] * t[1]) + t[1] * t[2]);
      }

      b_t_tmp = varargin_2 - varargin_3;
      t[3] = ((b_t_tmp * *dir - q_diff) - (this->j_max[v_drive_tmp] *
               (((0.16666666666666666 * rt_powd_snf(t[6], 3.0) + 0.5 * (t[6] *
        t[6]) * (t[5] + t[4])) - 0.16666666666666666 * rt_powd_snf(t[4], 3.0)) +
                0.5 * t[6] * (t[4] * t[4])) + this->a_max[v_drive_tmp] * (0.5 *
                (t[5] * t[5]) + t[5] * t[4]))) / v_drive;

      //  Check if phase 4 does not exist
      //  (max velocity cannot be reached)
      if (t[3] < -0.004) {
        if (*mod_jerk_profile) {
          //  This case is not valid
          //  (is handled in timeScaling function)
          for (i = 0; i < 7; i++) {
            t[i] = 0.0;
          }
        } else {
          double a;
          double b_a;
          double c_a;
          double d_a;
          double e_a;
          double f_a;
          double g_a;
          double root_tmp;
          double b_root_tmp;
          double c_root_tmp;
          double d_root_tmp;
          double e_root_tmp;
          double f_root_tmp;
          double g_root_tmp;
          double h_root_tmp;
          double i_root_tmp;

          //  Calculate times if max velocity is not reached
          a = this->j_max[v_drive_tmp];
          b_a = this->j_max[v_drive_tmp];
          c_a = this->j_max[v_drive_tmp];
          d_a = this->j_max[v_drive_tmp];
          e_a = this->j_max[v_drive_tmp];
          f_a = this->a_max[v_drive_tmp];
          g_a = this->a_max[v_drive_tmp];
          root_tmp = 2.0 * this->j_max[v_drive_tmp];
          q_diff = rt_powd_snf(t[0], 3.0);
          b_root_tmp = root_tmp * this->a_max[v_drive_tmp];
          c_root_tmp = t[2] * t[2];
          d_root_tmp = t[4] * t[4];
          q_part1_tmp = t[0] * t[0];
          v_drive = 2.0 * a_0 * this->a_max[v_drive_tmp];
          e_root_tmp = 4.0 * this->a_max[v_drive_tmp];
          f_root_tmp = a_0 * a_0;
          g_root_tmp = v_0 * v_0;
          h_root_tmp = 2.0 * f_root_tmp;
          i_root_tmp = root_tmp * v_0;
          root = ((((((((((((((((((((a * a * rt_powd_snf(t[0], 4.0) / 2.0 - b_a *
            b_a * rt_powd_snf(t[2], 4.0) / 4.0) + c_a * c_a * c_root_tmp *
            d_root_tmp / 2.0) - d_a * d_a * rt_powd_snf(t[4], 4.0) / 4.0) + e_a *
            e_a * rt_powd_snf(t[6], 4.0) / 2.0) + root_tmp * a_0 * q_diff) -
                                b_root_tmp * q_diff / 3.0) - b_root_tmp * t[0] *
                               c_root_tmp) + b_root_tmp * rt_powd_snf(t[2], 3.0)
                              / 3.0) + b_root_tmp * rt_powd_snf(t[4], 3.0) / 3.0)
                            - b_root_tmp * d_root_tmp * t[6]) - b_root_tmp *
                           rt_powd_snf(t[6], 3.0) / 3.0) + i_root_tmp *
                          q_part1_tmp) + h_root_tmp * q_part1_tmp) - v_drive *
                        q_part1_tmp) - v_drive * c_root_tmp) + 4.0 * a_0 * v_0 *
                      t[0]) + 2.0 * (f_a * f_a) * c_root_tmp) + 2.0 * (g_a * g_a)
                    * d_root_tmp) - e_root_tmp * v_0 * t[0]) + 4.0 * *dir *
                  b_t_tmp * this->a_max[v_drive_tmp]) + 2.0 * g_root_tmp;
          if (root > 0.0) {
            t[5] = -((((e_root_tmp * t[4] - 2.0 * std::sqrt(root)) + this->
                       j_max[v_drive_tmp] * c_root_tmp) - this->
                      j_max[v_drive_tmp] * d_root_tmp) + root_tmp * (t[6] * t[6]))
              / e_root_tmp;
            b_t_tmp = 0.5 * this->j_max[v_drive_tmp];
            t[1] = (((((((-v_0 - a_0 * t[0]) - b_t_tmp * (t[0] * t[0])) +
                        b_t_tmp * (t[2] * t[2])) + b_t_tmp * (t[6] * t[6])) -
                      b_t_tmp * (t[4] * t[4])) / this->a_max[v_drive_tmp] - t[2])
                    + t[5]) + t[4];
            t[3] = 0.0;

            //  Check if phase 2 and/ or phase 6 does not exist
            //  (max velocity and max acceleration cannot be reached)
            if ((t[5] < -0.004) || (t[1] < -0.004)) {
              double d;
              double d1;
              double d2;
              int x_size_idx_0;
              int trueCount;
              a = this->j_max[v_drive_tmp];
              b_a = this->j_max[v_drive_tmp];
              c_a = this->j_max[v_drive_tmp];
              dv[0] = 12.0;
              dv[1] = 0.0;
              dv[2] = -24.0 * f_root_tmp + 48.0 * this->j_max[v_drive_tmp] * v_0;
              d = rt_powd_snf(a_0, 3.0);
              dv[3] = ((48.0 * *dir * (a * a) * varargin_3 - 48.0 * *dir * (b_a *
                         b_a) * varargin_2) + 16.0 * d) - 48.0 * a_0 *
                this->j_max[v_drive_tmp] * v_0;
              d1 = rt_powd_snf(a_0, 4.0);
              d2 = 12.0 * f_root_tmp * this->j_max[v_drive_tmp] * v_0;
              dv[4] = (-3.0 * d1 + d2) - 12.0 * (c_a * c_a) * g_root_tmp;
              roots(dv, root_data, root_size);

              //  Choose non-complex, positive solution
              x_size_idx_0 = root_size[0];
              loop_ub = root_size[0];
              for (i = 0; i < loop_ub; i++) {
                x_data[i] = root_data[i].im;
              }

              for (loop_ub = 0; loop_ub < x_size_idx_0; loop_ub++) {
                y_data[loop_ub] = std::abs(x_data[loop_ub]);
              }

              for (i = 0; i < x_size_idx_0; i++) {
                tmp_data[i] = (y_data[i] < 0.004);
              }

              loop_ub = root_size[0] - 1;
              trueCount = 0;
              x_size_idx_0 = 0;
              for (i = 0; i <= loop_ub; i++) {
                if (tmp_data[i]) {
                  trueCount++;
                  root_data[x_size_idx_0] = root_data[i];
                  x_size_idx_0++;
                }
              }

              loop_ub = trueCount - 1;
              x_size_idx_0 = 0;
              for (i = 0; i <= loop_ub; i++) {
                if (root_data[i].re >= 0.0) {
                  root_data[x_size_idx_0] = root_data[i];
                  x_size_idx_0++;
                }
              }

              q_diff = root_data[0].re * root_data[0].im;
              e_root_tmp = ((2.0 * (root_data[0].re * root_data[0].re -
                                    root_data[0].im * root_data[0].im) - 4.0 *
                             a_0 * root_data[0].re) + f_root_tmp) - 2.0 * v_0 *
                this->j_max[v_drive_tmp];
              v_drive = 2.0 * (q_diff + q_diff) - 4.0 * a_0 * root_data[0].im;
              d_root_tmp = 4.0 * this->j_max[v_drive_tmp];
              root = d_root_tmp * root_data[0].re;
              c_root_tmp = d_root_tmp * root_data[0].im;
              if (c_root_tmp == 0.0) {
                if (v_drive == 0.0) {
                  q_diff = e_root_tmp / root;
                } else if (e_root_tmp == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = e_root_tmp / root;
                }
              } else if (root == 0.0) {
                if (e_root_tmp == 0.0) {
                  q_diff = v_drive / c_root_tmp;
                } else if (v_drive == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = v_drive / c_root_tmp;
                }
              } else {
                q_diff = std::abs(root);
                q_part1_tmp = std::abs(c_root_tmp);
                if (q_diff > q_part1_tmp) {
                  q_diff = c_root_tmp / root;
                  q_diff = (e_root_tmp + q_diff * v_drive) / (root + q_diff *
                    c_root_tmp);
                } else if (q_part1_tmp == q_diff) {
                  if (root > 0.0) {
                    root = 0.5;
                  } else {
                    root = -0.5;
                  }

                  if (c_root_tmp > 0.0) {
                    c_root_tmp = 0.5;
                  } else {
                    c_root_tmp = -0.5;
                  }

                  q_diff = (e_root_tmp * root + v_drive * c_root_tmp) / q_diff;
                } else {
                  q_diff = root / c_root_tmp;
                  q_diff = (q_diff * e_root_tmp + v_drive) / (c_root_tmp +
                    q_diff * root);
                }
              }

              t[0] = q_diff;

              //  Calculate other switch times
              a = this->j_max[v_drive_tmp];
              t[6] = std::sqrt(((4.0 * (a * a) * (q_diff * q_diff) + 8.0 * a_0 *
                                 this->j_max[v_drive_tmp] * q_diff) + h_root_tmp)
                               + d_root_tmp * v_0) / root_tmp;
              t[4] = (a_0 / this->j_max[v_drive_tmp] + q_diff) + t[6];
              t[1] = 0.0;
              t[5] = 0.0;

              //  Check if a_max is exceeded (Phase 2 exists)
              if (a_0 + q_diff * this->j_max[v_drive_tmp] > this->
                  a_max[v_drive_tmp]) {
                t[0] = (this->a_max[v_drive_tmp] - a_0) / this->
                  j_max[v_drive_tmp];
                a = this->a_max[v_drive_tmp];
                b_a = this->j_max[v_drive_tmp];
                c_a = this->j_max[v_drive_tmp];
                d_a = this->a_max[v_drive_tmp];
                e_a = this->j_max[v_drive_tmp];
                f_a = this->j_max[v_drive_tmp];
                g_a = this->j_max[v_drive_tmp];
                q_diff = this->j_max[v_drive_tmp];
                q_part1_tmp = this->j_max[v_drive_tmp];
                v_drive = this->j_max[v_drive_tmp];
                b_t_tmp = t[0] * t[0];
                root = 72.0 * this->a_max[v_drive_tmp] * *dir;
                t[6] = 1.0 / this->j_max[v_drive_tmp] * ((this->
                  a_max[v_drive_tmp] / 2.0 + std::sqrt(9.0 * (a * a) + 6.0 * std::
                  sqrt(((((((((-12.0 * this->a_max[v_drive_tmp] * rt_powd_snf
                               (this->j_max[v_drive_tmp], 3.0) * rt_powd_snf(t[0],
                  3.0) + 9.0 * f_root_tmp * (b_a * b_a) * b_t_tmp) - 18.0 * a_0 *
                              this->a_max[v_drive_tmp] * (c_a * c_a) * b_t_tmp)
                             + 9.0 * (d_a * d_a) * (e_a * e_a) * b_t_tmp) + 36.0
                            * a_0 * (f_a * f_a) * t[0] * v_0) - root * (g_a *
                  g_a) * varargin_3) + root * (q_diff * q_diff) * varargin_2) -
                         36.0 * this->a_max[v_drive_tmp] * (q_part1_tmp *
                  q_part1_tmp) * t[0] * v_0) + 3.0 * rt_powd_snf(this->
                  a_max[v_drive_tmp], 4.0)) + 36.0 * (v_drive * v_drive) *
                       g_root_tmp)) / 6.0) - this->a_max[v_drive_tmp]);
                t[4] = t[6] + t_tmp;
                b_t_tmp = 2.0 * this->a_max[v_drive_tmp];
                t[1] = -(((((((-this->j_max[v_drive_tmp] * (t[4] * t[4]) -
                               root_tmp * t[4] * t[6]) + this->j_max[v_drive_tmp]
                              * (t[6] * t[6])) + a_0 * t[0]) + this->
                            a_max[v_drive_tmp] * t[0]) + b_t_tmp * t[4]) +
                          b_t_tmp * t[6]) + 2.0 * v_0) / b_t_tmp;
                t[5] = 0.0;
              }

              //  Check if -a_max is exceeded (Phase 6 exists)
              if (t[6] * this->j_max[v_drive_tmp] > this->a_max[v_drive_tmp]) {
                t[6] = t_tmp;
                a = this->a_max[v_drive_tmp];
                b_a = this->j_max[v_drive_tmp];
                c_a = this->j_max[v_drive_tmp];
                d_a = this->a_max[v_drive_tmp];
                e_a = this->a_max[v_drive_tmp];
                f_a = this->j_max[v_drive_tmp];
                dv[0] = 12.0;
                dv[1] = -24.0 * this->a_max[v_drive_tmp];
                dv[2] = (-12.0 * f_root_tmp + 12.0 * (a * a)) + 24.0 *
                  this->j_max[v_drive_tmp] * v_0;
                dv[3] = 0.0;
                dv[4] = (((((((24.0 * *dir * (b_a * b_a) * varargin_3 *
                               this->a_max[v_drive_tmp] - 24.0 * *dir * (c_a *
                  c_a) * varargin_2 * this->a_max[v_drive_tmp]) + 3.0 * d1) +
                             8.0 * d * this->a_max[v_drive_tmp]) + 6.0 *
                            f_root_tmp * (d_a * d_a)) - d2) - 24.0 * a_0 *
                          this->j_max[v_drive_tmp] * v_0 * this->
                          a_max[v_drive_tmp]) - 12.0 * (e_a * e_a) * this->
                         j_max[v_drive_tmp] * v_0) + 12.0 * (f_a * f_a) *
                  g_root_tmp;
                roots(dv, root_data, root_size);

                //  Choose non-complex, positive solution
                x_size_idx_0 = root_size[0];
                loop_ub = root_size[0];
                for (i = 0; i < loop_ub; i++) {
                  x_data[i] = root_data[i].im;
                }

                for (loop_ub = 0; loop_ub < x_size_idx_0; loop_ub++) {
                  y_data[loop_ub] = std::abs(x_data[loop_ub]);
                }

                for (i = 0; i < x_size_idx_0; i++) {
                  tmp_data[i] = (y_data[i] < 0.004);
                }

                loop_ub = root_size[0] - 1;
                trueCount = 0;
                x_size_idx_0 = 0;
                for (i = 0; i <= loop_ub; i++) {
                  if (tmp_data[i]) {
                    trueCount++;
                    root_data[x_size_idx_0] = root_data[i];
                    x_size_idx_0++;
                  }
                }

                loop_ub = trueCount - 1;
                trueCount = 0;
                for (i = 0; i <= loop_ub; i++) {
                  if (root_data[i].re >= 0.0) {
                    trueCount++;
                  }
                }

                x_size_idx_0 = 0;
                for (i = 0; i <= loop_ub; i++) {
                  if (root_data[i].re >= 0.0) {
                    root_data[x_size_idx_0] = root_data[i];
                    x_size_idx_0++;
                  }
                }

                root = this->j_max[v_drive_tmp];
                q_part1_tmp = this->a_max[v_drive_tmp];
                for (i = 0; i < trueCount; i++) {
                  e_root_tmp = (root_data[i].re - a_0) - q_part1_tmp;
                  if (root_data[i].im == 0.0) {
                    q_diff = e_root_tmp / root;
                  } else if (e_root_tmp == 0.0) {
                    q_diff = 0.0;
                  } else {
                    q_diff = e_root_tmp / root;
                  }

                  y_data[i] = q_diff;
                }

                e_root_tmp = (root_data[0].re - a_0) - this->a_max[v_drive_tmp];
                if (root_data[0].im == 0.0) {
                  q_diff = e_root_tmp / root;
                } else if (e_root_tmp == 0.0) {
                  q_diff = 0.0;
                } else {
                  q_diff = e_root_tmp / root;
                }

                t[0] = q_diff;

                //  Calculate other switch times
                t[4] = (a_0 + this->a_max[v_drive_tmp]) / this->
                  j_max[v_drive_tmp] + y_data[0];
                a = this->j_max[v_drive_tmp];
                b_a = this->j_max[v_drive_tmp];
                c_a = this->j_max[v_drive_tmp];
                d_a = this->a_max[v_drive_tmp];
                t_tmp = 2.0 * a_0 * this->j_max[v_drive_tmp];
                t[5] = ((((((a * a * (y_data[0] * y_data[0]) + 2.0 * (b_a * b_a)
                             * y_data[0] * t[4]) - c_a * c_a * (t[4] * t[4])) +
                           t_tmp * y_data[0]) + t_tmp * t[4]) - d_a * d_a) +
                        i_root_tmp) / b_root_tmp;
                t[1] = 0.0;
              }

              //  All other times are 0
              t[2] = 0.0;
              t[3] = 0.0;
            }

            guard1 = true;
          } else {
            //  This should hould never occur, only for safety
            for (i = 0; i < 7; i++) {
              t[i] = 0.0;
            }
          }
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      boolean_T y;
      boolean_T exitg1;

      //  Safety checks
      y = false;
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub < 7)) {
        if (!(t[loop_ub] < -0.004)) {
          loop_ub++;
        } else {
          y = true;
          exitg1 = true;
        }
      }

      if (y) {
        //  No numeric inaccuracy
        for (i = 0; i < 7; i++) {
          t[i] = 0.0;
        }
      }

      //  Small numeric errors are set to 0
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        if ((0.0 > t[loop_ub]) || rtIsNaN(t[loop_ub])) {
          t[loop_ub] = 0.0;
        }
      }

      //  Calculate absolute times for jerk switches
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        t[loop_ub + 1] += t[loop_ub];
      }
    }
  }
}

//
// TIMESCALING % Calculate switching times to fulfil a given
//  time by adjusting the maximally reached velocity
// Arguments    : double q_goal
//                double q_0
//                double v_0
//                double a_0
//                double dir
//                double joint
//                double t_required
//                double t[7]
//                creal_T *v_drive
//                boolean_T *mod_jerk_profile
// Return Type  : void
//
void LTPlanner::timeScaling(double q_goal, double q_0, double v_0, double a_0,
  double dir, double joint, double t_required, double t[7], creal_T *v_drive,
  boolean_T *mod_jerk_profile) const
{
  int a_tmp;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double potential_v_drive;
  double g_a;
  double h_a;
  double potential_v_drive_tmp;
  double b_potential_v_drive_tmp;
  double c_potential_v_drive_tmp;
  double d_potential_v_drive_tmp;
  double e_potential_v_drive_tmp;
  double f_potential_v_drive_tmp;
  double g_potential_v_drive_tmp;
  double h_potential_v_drive_tmp;
  double i_potential_v_drive_tmp;
  double j_potential_v_drive_tmp;
  double k_potential_v_drive_tmp;
  double l_potential_v_drive_tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  boolean_T guard7 = false;
  boolean_T guard8 = false;
  double unusedU1;
  double d;
  double b_a_tmp;
  double c_a_tmp;
  double dv[5];
  double d1;
  double d2;
  double d3;
  double potential_v_drive_tmp_tmp;
  double d4;
  int j;
  double d5;
  double m_potential_v_drive_tmp;
  double c[6];
  double d6;
  double n_potential_v_drive_tmp;
  double o_potential_v_drive_tmp;
  double p_potential_v_drive_tmp;
  creal_T tmp_data[4];
  int tmp_size[1];
  int k1;
  double dv1[7];
  creal_T root_data[6];
  double c_tmp;
  double b_c_tmp;
  double c_c_tmp;
  double d_c_tmp;
  double e_c_tmp;
  double f_c_tmp;
  double g_c_tmp;
  double h_c_tmp;
  double i_c_tmp;
  double j_c_tmp;
  double k_c_tmp;
  creal_T r_data[5];
  int a_size[2];
  double ctmp[6];
  creal_T a_data[25];
  creal_T eiga_data[5];

  //  Parameters for calculation
  //  If goal is in negative direction, map to pos. direction
  if (dir < 0.0) {
    v_0 = -v_0;
    a_0 = -a_0;
  }

  //             %% Calculate required v_drive to reach goal at given time
  //  Standard jerk profile: Phases 2 and 6 exist
  a_tmp = static_cast<int>(joint) - 1;
  a = this->a_max[a_tmp];
  b_a = this->a_max[a_tmp];
  c_a = this->j_max[a_tmp];
  d_a = this->a_max[a_tmp];
  e_a = this->j_max[a_tmp];
  f_a = this->j_max[a_tmp];
  potential_v_drive = this->j_max[a_tmp];
  g_a = this->a_max[a_tmp];
  h_a = this->j_max[a_tmp];
  potential_v_drive_tmp = a_0 * a_0;
  b_potential_v_drive_tmp = 144.0 * this->a_max[a_tmp] * dir;
  c_potential_v_drive_tmp = 36.0 * potential_v_drive_tmp;
  d_potential_v_drive_tmp = rt_powd_snf(this->a_max[a_tmp], 3.0);
  e_potential_v_drive_tmp = this->a_max[a_tmp] * this->j_max[a_tmp];
  f_potential_v_drive_tmp = rt_powd_snf(a_0, 4.0);
  g_potential_v_drive_tmp = rt_powd_snf(a_0, 3.0);
  h_potential_v_drive_tmp = v_0 * v_0;
  i_potential_v_drive_tmp = 9.0 * f_potential_v_drive_tmp;
  j_potential_v_drive_tmp = 12.0 * g_potential_v_drive_tmp * this->a_max[a_tmp];
  k_potential_v_drive_tmp = rt_powd_snf(this->a_max[a_tmp], 4.0);
  l_potential_v_drive_tmp = t_required * t_required;
  potential_v_drive = (((((e_potential_v_drive_tmp * t_required / 2.0 -
    potential_v_drive_tmp / 4.0) + a_0 * this->a_max[a_tmp] / 2.0) - a * a / 2.0)
                        + v_0 * this->j_max[a_tmp] / 2.0) - std::sqrt
                       ((((((((((((((36.0 * (b_a * b_a) * (c_a * c_a) *
    l_potential_v_drive_tmp - c_potential_v_drive_tmp * this->a_max[a_tmp] *
    this->j_max[a_tmp] * t_required) + 72.0 * a_0 * (d_a * d_a) * this->
    j_max[a_tmp] * t_required) - 72.0 * d_potential_v_drive_tmp * this->
    j_max[a_tmp] * t_required) + b_potential_v_drive_tmp * (e_a * e_a) * q_0) -
    b_potential_v_drive_tmp * (f_a * f_a) * q_goal) + 72.0 * this->a_max[a_tmp] *
    (potential_v_drive * potential_v_drive) * v_0 * t_required) -
    i_potential_v_drive_tmp) + j_potential_v_drive_tmp) +
    c_potential_v_drive_tmp * (g_a * g_a)) + c_potential_v_drive_tmp *
    this->j_max[a_tmp] * v_0) - 72.0 * a_0 * d_potential_v_drive_tmp) - 72.0 *
    a_0 * this->a_max[a_tmp] * this->j_max[a_tmp] * v_0) + 36.0 *
    k_potential_v_drive_tmp) - 36.0 * (h_a * h_a) * h_potential_v_drive_tmp) /
                       12.0) / this->j_max[a_tmp];

  //  Check if v_drive is real and positive
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  guard5 = false;
  guard6 = false;
  guard7 = false;
  guard8 = false;
  if (potential_v_drive > 0.0) {
    this->b_optSwitchTimes(q_goal, q_0, (dir * v_0), (dir * a_0), joint,
      potential_v_drive, t, (&unusedU1), mod_jerk_profile);

    //  Check time constraint was fulfilled
    d = t_required - t[6];
    if ((d < 0.1) && (d > -0.01)) {
      v_drive->re = potential_v_drive;
      v_drive->im = 0.0;
    } else {
      guard8 = true;
    }
  } else {
    guard8 = true;
  }

  if (guard8) {
    //  Modified jerk profile: Phases 2 and 6 exist
    a = this->a_max[a_tmp];
    unusedU1 = a_0 + this->a_max[a_tmp];
    b_a = this->j_max[a_tmp];
    c_a = this->j_max[a_tmp];
    d_a = this->a_max[a_tmp];
    e_a = this->j_max[a_tmp];
    b_a_tmp = 2.0 * this->j_max[a_tmp];
    potential_v_drive = a_0 - this->a_max[a_tmp];
    c_a_tmp = (v_0 + a_0 * potential_v_drive / b_a_tmp) / this->a_max[a_tmp];
    g_a = this->a_max[a_tmp] / b_a_tmp;
    potential_v_drive /= b_a_tmp;
    h_a = (c_a_tmp - g_a) + potential_v_drive;
    f_a = this->a_max[a_tmp];
    potential_v_drive_tmp_tmp = rt_powd_snf(this->j_max[a_tmp], 3.0);
    b_potential_v_drive_tmp = 6.0 * potential_v_drive_tmp_tmp;
    m_potential_v_drive_tmp = (c_a_tmp + g_a) + potential_v_drive;
    n_potential_v_drive_tmp = 2.0 * this->a_max[a_tmp] * this->j_max[a_tmp];
    o_potential_v_drive_tmp = b_a_tmp * v_0;
    p_potential_v_drive_tmp = unusedU1 * unusedU1;
    potential_v_drive = -((((dir * (q_0 - q_goal) - this->j_max[a_tmp] *
      (((rt_powd_snf(unusedU1, 3.0) / b_potential_v_drive_tmp -
         d_potential_v_drive_tmp / b_potential_v_drive_tmp) + a * a * unusedU1 /
        (2.0 * potential_v_drive_tmp_tmp)) + p_potential_v_drive_tmp *
       m_potential_v_drive_tmp / (2.0 * (b_a * b_a)))) + a_0 *
      ((p_potential_v_drive_tmp / (2.0 * (c_a * c_a)) + d_a * d_a / (2.0 * (e_a *
      e_a))) + unusedU1 * m_potential_v_drive_tmp / this->j_max[a_tmp])) -
      this->a_max[a_tmp] * (h_a * h_a / 2.0 + this->a_max[a_tmp] * h_a /
      this->j_max[a_tmp])) + v_0 * (((c_a_tmp + unusedU1 / this->j_max[a_tmp]) +
      g_a) + potential_v_drive)) / (((((g_a - v_0 / this->a_max[a_tmp]) +
      this->a_max[a_tmp] * (h_a / this->a_max[a_tmp] + 1.0 / this->j_max[a_tmp]))
      - ((((potential_v_drive_tmp + 2.0 * a_0 * this->a_max[a_tmp]) + 4.0 * (f_a
      * f_a)) - b_a_tmp * t_required * this->a_max[a_tmp]) +
         o_potential_v_drive_tmp) / n_potential_v_drive_tmp) +
      p_potential_v_drive_tmp / n_potential_v_drive_tmp) - a_0 * unusedU1 /
      e_potential_v_drive_tmp);

    //  Check if potential_v_drive is real and positive
    if (potential_v_drive > 0.0) {
      this->b_optSwitchTimes(q_goal, q_0, (dir * v_0), (dir * a_0), joint,
        potential_v_drive, t, (&unusedU1), mod_jerk_profile);

      //  Check time constraint was fulfilled
      d = t_required - t[6];
      if ((d < 0.1) && (d > -0.01)) {
        v_drive->re = potential_v_drive;
        v_drive->im = 0.0;
      } else {
        guard7 = true;
      }
    } else {
      guard7 = true;
    }
  }

  if (guard7) {
    //  Standard jerk profile: Phase 2 does not exist
    a = this->a_max[a_tmp];
    b_a = this->j_max[a_tmp];
    c_a = this->j_max[a_tmp];
    d_a = this->j_max[a_tmp];
    e_a = this->a_max[a_tmp];
    f_a = this->a_max[a_tmp];
    potential_v_drive = this->j_max[a_tmp];
    dv[0] = 3.0;
    d = 12.0 * this->a_max[a_tmp];
    dv[1] = d;
    d2 = -24.0 * this->a_max[a_tmp];
    d3 = d2 * this->j_max[a_tmp] * t_required;
    d4 = 24.0 * this->j_max[a_tmp] * v_0;
    d5 = 12.0 * potential_v_drive_tmp;
    d6 = 24.0 * a_0 * this->a_max[a_tmp];
    dv[2] = (((d3 - d5) - d6) + 12.0 * (a * a)) + d4;
    dv[3] = 0.0;
    d1 = 48.0 * potential_v_drive_tmp;
    c_a_tmp = 24.0 * potential_v_drive_tmp;
    dv[4] = ((((((((d1 * this->a_max[a_tmp] * this->j_max[a_tmp] * t_required -
                    96.0 * dir * (b_a * b_a) * this->a_max[a_tmp] * q_0) + 96.0 *
                   dir * (c_a * c_a) * this->a_max[a_tmp] * q_goal) - 96.0 *
                  this->a_max[a_tmp] * (d_a * d_a) * v_0 * t_required) + 12.0 *
                 f_potential_v_drive_tmp) + 16.0 * g_potential_v_drive_tmp *
                this->a_max[a_tmp]) - c_a_tmp * (e_a * e_a)) - d1 * this->
              j_max[a_tmp] * v_0) + 48.0 * (f_a * f_a) * this->j_max[a_tmp] *
             v_0) + 48.0 * (potential_v_drive * potential_v_drive) *
      h_potential_v_drive_tmp;
    roots(dv, tmp_data, tmp_size);
    k1 = tmp_size[0];
    if (0 <= k1 - 1) {
      std::memcpy(&root_data[0], &tmp_data[0], k1 * sizeof(creal_T));
    }

    potential_v_drive = root_data[2].re * root_data[2].im;
    g_a = potential_v_drive + potential_v_drive;
    unusedU1 = 4.0 * this->j_max[a_tmp];
    potential_v_drive = (-2.0 * potential_v_drive_tmp + unusedU1 * v_0) +
      (root_data[2].re * root_data[2].re - root_data[2].im * root_data[2].im);
    if (g_a == 0.0) {
      v_drive->re = potential_v_drive / unusedU1;
      v_drive->im = 0.0;
    } else if (potential_v_drive == 0.0) {
      v_drive->re = 0.0;
      v_drive->im = g_a / unusedU1;
    } else {
      v_drive->re = potential_v_drive / unusedU1;
      v_drive->im = g_a / unusedU1;
    }

    //  Check if potential_v_drive is real and positive
    if ((!(v_drive->im != 0.0)) && (v_drive->re > 0.0)) {
      this->c_optSwitchTimes(q_goal, q_0, (dir * v_0), (dir * a_0), joint,
        (*v_drive), t, (&unusedU1), mod_jerk_profile);

      //  Check time constraint was fulfilled
      d1 = t_required - t[6];
      if ((!(d1 < 0.1)) || (!(d1 > -0.01))) {
        guard6 = true;
      }
    } else {
      guard6 = true;
    }
  }

  if (guard6) {
    //  Standard jerk profile: Phase 6 does not exist
    a = this->a_max[a_tmp];
    b_a = this->j_max[a_tmp];
    c_a = this->j_max[a_tmp];
    d_a = this->a_max[a_tmp];
    e_a = this->j_max[a_tmp];
    f_a = this->a_max[a_tmp];
    potential_v_drive = this->a_max[a_tmp];
    dv[0] = 12.0;
    d1 = 24.0 * this->a_max[a_tmp];
    dv[1] = d1;
    unusedU1 = 48.0 * a_0 * this->a_max[a_tmp];
    dv[2] = (((((d3 + c_a_tmp) - unusedU1) + 24.0 * (a * a)) - d4) + 12.0 * a_0)
      - d;
    dv[3] = 0.0;
    d *= this->j_max[a_tmp];
    d3 = 6.0 * potential_v_drive_tmp;
    dv[4] = ((((((((((((((-24.0 * dir * (b_a * b_a) * this->a_max[a_tmp] * q_0 +
                          24.0 * dir * (c_a * c_a) * this->a_max[a_tmp] * q_goal)
                         + i_potential_v_drive_tmp) - j_potential_v_drive_tmp) -
                       c_a_tmp * this->j_max[a_tmp] * v_0) + unusedU1 *
                      this->j_max[a_tmp] * v_0) + 4.0 * k_potential_v_drive_tmp)
                    - 24.0 * (d_a * d_a) * this->j_max[a_tmp] * v_0) + 12.0 *
                   (e_a * e_a) * h_potential_v_drive_tmp) + 6.0 *
                  g_potential_v_drive_tmp) + d3 * this->a_max[a_tmp]) - 12.0 *
                a_0 * (f_a * f_a)) - 12.0 * a_0 * this->j_max[a_tmp] * v_0) + d *
              v_0) + 4.0 * a_0 * this->a_max[a_tmp]) - 4.0 * (potential_v_drive *
      potential_v_drive);
    roots(dv, tmp_data, tmp_size);
    k1 = tmp_size[0];
    if (0 <= k1 - 1) {
      std::memcpy(&root_data[0], &tmp_data[0], k1 * sizeof(creal_T));
    }

    unusedU1 = root_data[2].re * root_data[2].re - root_data[2].im * root_data[2]
      .im;
    potential_v_drive = root_data[2].re * root_data[2].im;
    g_a = potential_v_drive + potential_v_drive;
    c_a_tmp = this->j_max[a_tmp];
    if (g_a == 0.0) {
      v_drive->re = unusedU1 / c_a_tmp;
      v_drive->im = 0.0;
    } else if (unusedU1 == 0.0) {
      v_drive->re = 0.0;
      v_drive->im = g_a / c_a_tmp;
    } else {
      v_drive->re = unusedU1 / c_a_tmp;
      v_drive->im = g_a / c_a_tmp;
    }

    //  Check if potential_v_drive is real and positive
    if ((!(v_drive->im != 0.0)) && (v_drive->re > 0.0)) {
      this->c_optSwitchTimes(q_goal, q_0, (dir * v_0), (dir * a_0), joint,
        (*v_drive), t, (&unusedU1), mod_jerk_profile);

      //  Check time constraint was fulfilled
      c_a_tmp = t_required - t[6];
      if ((!(c_a_tmp < 0.1)) || (!(c_a_tmp > -0.01))) {
        guard5 = true;
      }
    } else {
      guard5 = true;
    }
  }

  if (guard5) {
    int k2;
    int nTrailingZeros;

    //  Standard jerk profile: Phases 2 and 6 do not exist
    a = this->j_max[a_tmp];
    b_a = this->j_max[a_tmp];
    c_a = this->j_max[a_tmp];
    d_a = this->j_max[a_tmp];
    e_a = this->j_max[a_tmp];
    f_a = this->j_max[a_tmp];
    potential_v_drive = this->j_max[a_tmp];
    g_a = this->j_max[a_tmp];
    h_a = this->j_max[a_tmp];
    c_a_tmp = this->j_max[a_tmp];
    c[0] = 144.0 * this->j_max[a_tmp] * t_required + 144.0 * a_0;
    n_potential_v_drive_tmp = 144.0 * a_0 * this->j_max[a_tmp];
    p_potential_v_drive_tmp = n_potential_v_drive_tmp * t_required;
    b_potential_v_drive_tmp = 216.0 * this->j_max[a_tmp] * v_0;
    c[1] = ((-72.0 * (a * a) * l_potential_v_drive_tmp - p_potential_v_drive_tmp)
            + c_potential_v_drive_tmp) - b_potential_v_drive_tmp;
    d_potential_v_drive_tmp = 48.0 * g_potential_v_drive_tmp;
    n_potential_v_drive_tmp *= v_0;
    c[2] = ((144.0 * dir * (b_a * b_a) * q_0 - 144.0 * dir * (c_a * c_a) *
             q_goal) + d_potential_v_drive_tmp) - n_potential_v_drive_tmp;
    e_potential_v_drive_tmp = 144.0 * a_0 * dir;
    i_potential_v_drive_tmp = 6.0 * f_potential_v_drive_tmp;
    j_potential_v_drive_tmp = 144.0 * dir * potential_v_drive_tmp_tmp;
    k_potential_v_drive_tmp = j_potential_v_drive_tmp * q_goal * t_required;
    c_tmp = d_potential_v_drive_tmp * this->j_max[a_tmp] * t_required;
    b_c_tmp = 72.0 * potential_v_drive_tmp * this->j_max[a_tmp] * v_0;
    c[3] = (((((((-144.0 * dir * potential_v_drive_tmp_tmp * q_0 * t_required +
                  k_potential_v_drive_tmp) - c_tmp) - e_potential_v_drive_tmp *
                (d_a * d_a) * q_0) + e_potential_v_drive_tmp * (e_a * e_a) *
               q_goal) + 144.0 * a_0 * (f_a * f_a) * v_0 * t_required) +
             i_potential_v_drive_tmp) - b_c_tmp) + 216.0 * (potential_v_drive *
      potential_v_drive) * h_potential_v_drive_tmp;
    c[4] = 0.0;
    unusedU1 = dir * dir;
    potential_v_drive = rt_powd_snf(this->j_max[a_tmp], 4.0);
    g_c_tmp = d_potential_v_drive_tmp * dir;
    i_c_tmp = e_potential_v_drive_tmp * potential_v_drive_tmp_tmp;
    c_c_tmp = 72.0 * unusedU1 * potential_v_drive;
    d_c_tmp = q_0 * q_0;
    e_c_tmp = 144.0 * unusedU1 * potential_v_drive * q_0 * q_goal;
    f_c_tmp = c_c_tmp * (q_goal * q_goal);
    h_c_tmp = i_c_tmp * q_0 * v_0;
    i_c_tmp = i_c_tmp * q_goal * v_0;
    j_c_tmp = rt_powd_snf(a_0, 6.0);
    k_c_tmp = i_potential_v_drive_tmp * this->j_max[a_tmp] * v_0;
    m_potential_v_drive_tmp = 72.0 * potential_v_drive_tmp_tmp * rt_powd_snf(v_0,
      3.0);
    c[5] = (((((((((-72.0 * unusedU1 * potential_v_drive * d_c_tmp + e_c_tmp) -
                   f_c_tmp) - g_c_tmp * (g_a * g_a) * q_0) + g_c_tmp * (h_a *
      h_a) * q_goal) + h_c_tmp) - i_c_tmp) + j_c_tmp) - k_c_tmp) +
            c_potential_v_drive_tmp * (c_a_tmp * c_a_tmp) *
            h_potential_v_drive_tmp) - m_potential_v_drive_tmp;
    std::memset(&r_data[0], 0, 5U * sizeof(creal_T));
    k1 = 1;
    while ((k1 <= 6) && (!(c[k1 - 1] != 0.0))) {
      k1++;
    }

    k2 = 6;
    while ((k2 >= k1) && (!(c[k2 - 1] != 0.0))) {
      k2--;
    }

    nTrailingZeros = 5 - k2;
    if (k1 < k2) {
      int companDim;
      boolean_T exitg1;
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

      if (companDim >= 1) {
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

        eig(a_data, a_size, root_data, tmp_size);
        k1 = tmp_size[0];
        if (0 <= k1 - 1) {
          std::memcpy(&eiga_data[0], &root_data[0], k1 * sizeof(creal_T));
        }

        for (k1 = 0; k1 < companDim; k1++) {
          r_data[(k1 - k2) + 6] = eiga_data[k1];
        }
      }
    }

    potential_v_drive = r_data[1].re * r_data[1].re - r_data[1].im * r_data[1].
      im;
    unusedU1 = r_data[1].re * r_data[1].im;
    unusedU1 += unusedU1;
    c_a_tmp = this->j_max[a_tmp];
    if (unusedU1 == 0.0) {
      v_drive->re = potential_v_drive / c_a_tmp;
      v_drive->im = 0.0;
    } else if (potential_v_drive == 0.0) {
      v_drive->re = 0.0;
      v_drive->im = unusedU1 / c_a_tmp;
    } else {
      v_drive->re = potential_v_drive / c_a_tmp;
      v_drive->im = unusedU1 / c_a_tmp;
    }

    //  Check if potential_v_drive is real and positive
    if ((!(v_drive->im != 0.0)) && (v_drive->re > 0.0)) {
      this->c_optSwitchTimes(q_goal, q_0, (dir * v_0), (dir * a_0), joint,
        (*v_drive), t, (&unusedU1), mod_jerk_profile);

      //  Check time constraint was fulfilled
      c_a_tmp = t_required - t[6];
      if ((!(c_a_tmp < 0.1)) || (!(c_a_tmp > -0.01))) {
        guard4 = true;
      }
    } else {
      guard4 = true;
    }
  }

  if (guard4) {
    //  Modified profile: Phase 2 does not exist
    a = this->a_max[a_tmp];
    b_a = this->j_max[a_tmp];
    c_a = this->j_max[a_tmp];
    d_a = this->j_max[a_tmp];
    e_a = this->a_max[a_tmp];
    f_a = this->a_max[a_tmp];
    potential_v_drive = this->j_max[a_tmp];
    dv[0] = 3.0;
    dv[1] = -8.4852813742385713 * this->a_max[a_tmp];
    dv[2] = (((d * t_required - d3) - 12.0 * a_0 * this->a_max[a_tmp]) - 6.0 *
             (a * a)) - 12.0 * this->j_max[a_tmp] * v_0;
    dv[3] = 0.0;
    d = 3.0 * f_potential_v_drive_tmp;
    c_a_tmp = d5 * this->j_max[a_tmp] * v_0;
    dv[4] = ((((((((-12.0 * potential_v_drive_tmp * this->a_max[a_tmp] *
                    this->j_max[a_tmp] * t_required - 24.0 * dir * (b_a * b_a) *
                    this->a_max[a_tmp] * q_0) + 24.0 * dir * (c_a * c_a) *
                   this->a_max[a_tmp] * q_goal) - d1 * (d_a * d_a) * v_0 *
                  t_required) + d) + 4.0 * g_potential_v_drive_tmp * this->
                a_max[a_tmp]) + d3 * (e_a * e_a)) + c_a_tmp) + 12.0 * (f_a * f_a)
             * this->j_max[a_tmp] * v_0) + 12.0 * (potential_v_drive *
      potential_v_drive) * h_potential_v_drive_tmp;
    roots(dv, tmp_data, tmp_size);
    k1 = tmp_size[0];
    if (0 <= k1 - 1) {
      std::memcpy(&root_data[0], &tmp_data[0], k1 * sizeof(creal_T));
    }

    potential_v_drive = root_data[2].re * root_data[2].im;
    g_a = potential_v_drive + potential_v_drive;
    potential_v_drive = -(((root_data[2].re * root_data[2].re - root_data[2].im *
      root_data[2].im) - potential_v_drive_tmp) - o_potential_v_drive_tmp);
    if (-g_a == 0.0) {
      v_drive->re = potential_v_drive / b_a_tmp;
      v_drive->im = 0.0;
    } else if (potential_v_drive == 0.0) {
      v_drive->re = 0.0;
      v_drive->im = -g_a / b_a_tmp;
    } else {
      v_drive->re = potential_v_drive / b_a_tmp;
      v_drive->im = -g_a / b_a_tmp;
    }

    //  Check if potential_v_drive is real and positive
    if ((!(v_drive->im != 0.0)) && (v_drive->re > 0.0)) {
      this->c_optSwitchTimes(q_goal, q_0, (dir * v_0), (dir * a_0), joint,
        (*v_drive), t, (&unusedU1), mod_jerk_profile);

      //  Check time constraint was fulfilled
      unusedU1 = t_required - t[6];
      if ((!(unusedU1 < 0.1)) || (!(unusedU1 > -0.01))) {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  }

  if (guard3) {
    //  Modified profile: Phase 6 does not exist
    a = this->a_max[a_tmp];
    b_a = this->j_max[a_tmp];
    c_a = this->j_max[a_tmp];
    d_a = this->a_max[a_tmp];
    e_a = this->a_max[a_tmp];
    f_a = this->j_max[a_tmp];
    dv[0] = 12.0;
    dv[1] = d2;
    dv[2] = (((d1 * this->j_max[a_tmp] * t_required - d5) - d6) - 12.0 * (a * a))
      - d4;
    dv[3] = 0.0;
    dv[4] = (((((((24.0 * dir * (b_a * b_a) * this->a_max[a_tmp] * q_0 - 24.0 *
                   dir * (c_a * c_a) * this->a_max[a_tmp] * q_goal) + d) + 8.0 *
                 g_potential_v_drive_tmp * this->a_max[a_tmp]) + d3 * (d_a * d_a))
               + c_a_tmp) + d6 * this->j_max[a_tmp] * v_0) + 12.0 * (e_a * e_a) *
             this->j_max[a_tmp] * v_0) + 12.0 * (f_a * f_a) *
      h_potential_v_drive_tmp;
    roots(dv, tmp_data, tmp_size);
    k1 = tmp_size[0];
    if (0 <= k1 - 1) {
      std::memcpy(&root_data[0], &tmp_data[0], k1 * sizeof(creal_T));
    }

    unusedU1 = root_data[2].re * root_data[2].re - root_data[2].im * root_data[2]
      .im;
    potential_v_drive = root_data[2].re * root_data[2].im;
    g_a = potential_v_drive + potential_v_drive;
    c_a_tmp = this->j_max[a_tmp];
    if (g_a == 0.0) {
      v_drive->re = unusedU1 / c_a_tmp;
      v_drive->im = 0.0;
    } else if (unusedU1 == 0.0) {
      v_drive->re = 0.0;
      v_drive->im = g_a / c_a_tmp;
    } else {
      v_drive->re = unusedU1 / c_a_tmp;
      v_drive->im = g_a / c_a_tmp;
    }

    //  Check if potential_v_drive is real and positive
    if ((!(v_drive->im != 0.0)) && (v_drive->re > 0.0)) {
      this->c_optSwitchTimes(q_goal, q_0, (dir * v_0), (dir * a_0), joint,
        (*v_drive), t, (&unusedU1), mod_jerk_profile);

      //  Check time constraint was fulfilled
      d = t_required - t[6];
      if ((!(d < 0.1)) || (!(d > -0.01))) {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    //  Modified profile: Phases 2 and 6 do not exist
    a = this->j_max[a_tmp];
    b_a = this->j_max[a_tmp];
    c_a = this->j_max[a_tmp];
    d_a = this->j_max[a_tmp];
    e_a = this->j_max[a_tmp];
    f_a = this->j_max[a_tmp];
    potential_v_drive = this->j_max[a_tmp];
    g_a = this->j_max[a_tmp];
    h_a = this->j_max[a_tmp];
    c_a_tmp = this->j_max[a_tmp];
    dv1[0] = 144.0;
    dv1[1] = -144.0 * this->j_max[a_tmp] * t_required + 144.0 * a_0;
    dv1[2] = ((72.0 * (a * a) * l_potential_v_drive_tmp -
               p_potential_v_drive_tmp) - c_potential_v_drive_tmp) -
      b_potential_v_drive_tmp;
    dv1[3] = ((-144.0 * dir * (b_a * b_a) * q_0 + 144.0 * dir * (c_a * c_a) *
               q_goal) - d_potential_v_drive_tmp) - n_potential_v_drive_tmp;
    dv1[4] = (((((((j_potential_v_drive_tmp * q_0 * t_required -
                    k_potential_v_drive_tmp) + c_tmp) - e_potential_v_drive_tmp *
                  (d_a * d_a) * q_0) + e_potential_v_drive_tmp * (e_a * e_a) *
                 q_goal) + 144.0 * a_0 * (f_a * f_a) * v_0 * t_required) +
               i_potential_v_drive_tmp) + b_c_tmp) + 216.0 * (potential_v_drive *
      potential_v_drive) * h_potential_v_drive_tmp;
    dv1[5] = 0.0;
    dv1[6] = (((((((((c_c_tmp * d_c_tmp - e_c_tmp) + f_c_tmp) + g_c_tmp * (g_a *
      g_a) * q_0) - g_c_tmp * (h_a * h_a) * q_goal) + h_c_tmp) - i_c_tmp) -
                j_c_tmp) - k_c_tmp) - c_potential_v_drive_tmp * (c_a_tmp *
               c_a_tmp) * h_potential_v_drive_tmp) - m_potential_v_drive_tmp;
    b_roots(dv1, root_data, tmp_size);
    unusedU1 = root_data[3].re * root_data[3].re - root_data[3].im * root_data[3]
      .im;
    potential_v_drive = root_data[3].re * root_data[3].im;
    g_a = potential_v_drive + potential_v_drive;
    c_a_tmp = this->j_max[a_tmp];
    if (g_a == 0.0) {
      v_drive->re = unusedU1 / c_a_tmp;
      v_drive->im = 0.0;
    } else if (unusedU1 == 0.0) {
      v_drive->re = 0.0;
      v_drive->im = g_a / c_a_tmp;
    } else {
      v_drive->re = unusedU1 / c_a_tmp;
      v_drive->im = g_a / c_a_tmp;
    }

    //  Check if potential_v_drive is real and positive
    if ((!(v_drive->im != 0.0)) && (v_drive->re > 0.0)) {
      this->c_optSwitchTimes(q_goal, q_0, (dir * v_0), (dir * a_0), joint,
        (*v_drive), t, (&unusedU1), mod_jerk_profile);

      //  Check time constraint was fulfilled
      d = t_required - t[6];
      if ((!(d < 0.1)) || (!(d > -0.01))) {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    //  No valid solution found, reset return parameters
    *mod_jerk_profile = false;
    for (j = 0; j < 7; j++) {
      t[j] = 0.0;
    }

    v_drive->re = this->v_max[a_tmp];
    v_drive->im = 0.0;
  }
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : double varargin_1
//                double varargin_2
//                const double varargin_3[6]
//                const double varargin_4[6]
//                const double varargin_5[6]
// Return Type  : LTPlanner *
//
LTPlanner *LTPlanner::init(double varargin_1, double varargin_2, const double
  varargin_3[6], const double varargin_4[6], const double varargin_5[6])
{
  LTPlanner *obj;
  int i;
  obj = this;

  // LTPLANNER Summary of this class goes here
  //    Detailed explanation goes here
  //  Degrees of freedom of the robot
  //  Sample time
  // Output length in samples
  //  Maximal jerk, acceleration and velocity for every joint
  // LTPLANNER Construct an instance of this class
  for (i = 0; i < 6; i++) {
    obj->v_max[i] = varargin_3[i];
  }

  for (i = 0; i < 6; i++) {
    obj->a_max[i] = varargin_4[i];
  }

  for (i = 0; i < 6; i++) {
    obj->j_max[i] = varargin_5[i];
  }

  obj->DoF = varargin_1;
  obj->Tsample = varargin_2;
  return obj;
}

//
// Arguments    : const double q_goal[6]
//                const double q_0[6]
//                const double v_0[6]
//                const double a_0[6]
//                double q_traj[6]
//                double v_traj[6]
//                double a_traj[6]
// Return Type  : void
//
void LTPlanner::trajectory(const double q_goal[6], const double q_0[6], const
  double v_0[6], const double a_0[6], double q_traj[6], double v_traj[6], double
  a_traj[6]) const
{
  int loop_ub;
  coder::array<double, 2U> t_opt;
  int loop_ub_tmp;
  coder::array<double, 2U> t_scaled;
  int i;
  coder::array<double, 1U> dir;
  coder::array<boolean_T, 1U> mod_jerk_profile;
  double dv[7];
  boolean_T unusedU0;
  int idx;
  double ex;
  int k;
  boolean_T exitg1;
  double v_drive[6];
  creal_T potential_v_drive;
  loop_ub = static_cast<int>(this->DoF);
  t_opt.set_size(loop_ub, 7);
  loop_ub_tmp = loop_ub * 7;

  //  Optimal jerk switching times
  t_scaled.set_size(loop_ub, 7);
  for (i = 0; i < loop_ub_tmp; i++) {
    t_opt[i] = 0.0;
    t_scaled[i] = 0.0;
  }

  //  Scaled jerk switching times
  dir.set_size(loop_ub);

  //  Direction of movement
  mod_jerk_profile.set_size(loop_ub);
  for (i = 0; i < loop_ub; i++) {
    dir[i] = 0.0;
    mod_jerk_profile[i] = false;
  }

  //  Boolean to choose jerk profile
  //              % Bring inputs into correct format
  //              if size(q_goal, 2) > 1
  //                  q_goal = q_goal';
  //              end
  //              if size(q_0, 2) > 1
  //                  q_0 = q_0';
  //              end
  //              if size(v_0, 2) > 1
  //                  v_0 = v_0';
  //              end
  //              if size(a_0, 2) > 1
  //                  a_0 = a_0';
  //              end
  //             %% Find slowest joint
  //  Calculate time-optimal profiles
  for (loop_ub_tmp = 0; loop_ub_tmp < loop_ub; loop_ub_tmp++) {
    this->optSwitchTimes(q_goal[loop_ub_tmp], q_0[loop_ub_tmp], v_0[loop_ub_tmp],
                         a_0[loop_ub_tmp], (static_cast<double>(loop_ub_tmp) +
      1.0), dv, (&dir[loop_ub_tmp]), (&unusedU0));
    for (i = 0; i < 7; i++) {
      t_opt[loop_ub_tmp + t_opt.size(0) * i] = dv[i];
    }
  }

  //  Save minimum time and joint
  i = t_opt.size(0);
  if (t_opt.size(0) <= 2) {
    if (t_opt.size(0) == 1) {
      ex = t_opt[t_opt.size(0) * 6];
      idx = 1;
    } else if ((t_opt[t_opt.size(0) * 6] < t_opt[t_opt.size(0) * 6 + 1]) ||
               (rtIsNaN(t_opt[t_opt.size(0) * 6]) && (!rtIsNaN(t_opt[t_opt.size
                  (0) * 6 + 1])))) {
      ex = t_opt[t_opt.size(0) * 6 + 1];
      idx = 2;
    } else {
      ex = t_opt[t_opt.size(0) * 6];
      idx = 1;
    }
  } else {
    if (!rtIsNaN(t_opt[t_opt.size(0) * 6])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= t_opt.size(0))) {
        if (!rtIsNaN(t_opt[(k + t_opt.size(0) * 6) - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      ex = t_opt[t_opt.size(0) * 6];
      idx = 1;
    } else {
      ex = t_opt[(idx + t_opt.size(0) * 6) - 1];
      loop_ub_tmp = idx + 1;
      for (k = loop_ub_tmp; k <= i; k++) {
        double d;
        d = t_opt[(k + t_opt.size(0) * 6) - 1];
        if (ex < d) {
          ex = d;
          idx = k;
        }
      }
    }
  }

  //             %% Scale other joints to require same time
  for (i = 0; i < 6; i++) {
    v_drive[i] = this->v_max[i];
  }

  for (loop_ub_tmp = 0; loop_ub_tmp < loop_ub; loop_ub_tmp++) {
    if (!(static_cast<double>(loop_ub_tmp) + 1.0 == idx)) {
      this->timeScaling(q_goal[loop_ub_tmp], q_0[loop_ub_tmp], v_0[loop_ub_tmp],
                        a_0[loop_ub_tmp], dir[loop_ub_tmp], (static_cast<double>
        (loop_ub_tmp) + 1.0), ex, dv, (&potential_v_drive),
                        (&mod_jerk_profile[loop_ub_tmp]));
      for (i = 0; i < 7; i++) {
        t_scaled[loop_ub_tmp + t_scaled.size(0) * i] = dv[i];
      }

      v_drive[loop_ub_tmp] = potential_v_drive.re;
    }
  }

  //  If no solution was found, use optimal time
  for (loop_ub_tmp = 0; loop_ub_tmp < loop_ub; loop_ub_tmp++) {
    unusedU0 = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 7)) {
      if ((t_scaled[loop_ub_tmp + t_scaled.size(0) * k] == 0.0) || rtIsNaN
          (t_scaled[loop_ub_tmp + t_scaled.size(0) * k])) {
        k++;
      } else {
        unusedU0 = true;
        exitg1 = true;
      }
    }

    if (!unusedU0) {
      for (i = 0; i < 7; i++) {
        t_scaled[loop_ub_tmp + t_scaled.size(0) * i] = t_opt[loop_ub_tmp +
          t_opt.size(0) * i];
      }
    }
  }

  //             %% Calculate sampled trajectories
  this->getTrajectories(t_scaled, dir, mod_jerk_profile, q_0, v_0, a_0, v_drive,
                        q_traj, v_traj, a_traj);
}

//
// File trailer for LTPlanner.cpp
//
// [EOF]
//
