//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ltpTrajectory_types.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 20-Jul-2022 14:48:22
//
#ifndef LTPTRAJECTORY_TYPES_H
#define LTPTRAJECTORY_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable : 4251)

#endif

// Type Definitions
class LTPlanner
{
 public:
  LTPlanner *init(double varargin_1, double varargin_2, const double varargin_3
                  [6], const double varargin_4[6], const double varargin_5[6]);
  void trajectory(const double q_goal[6], const double q_0[6], const double v_0
                  [6], const double a_0[6], double q_traj[6], double v_traj[6],
                  double a_traj[6]) const;
  void optSwitchTimes(double varargin_2, double varargin_3, double varargin_4,
                      double varargin_5, double varargin_6, double t[7], double *
                      dir, boolean_T *mod_jerk_profile) const;
  void optBreaking(double v_0, double a_0, double joint, double *q, double
                   t_rel[3], double *dir) const;
  void optBreaking(double v_0, double a_0, double joint, double *q, double
                   t_rel[3]) const;
  void timeScaling(double q_goal, double q_0, double v_0, double a_0, double dir,
                   double joint, double t_required, double t[7], creal_T
                   *v_drive, boolean_T *mod_jerk_profile) const;
  void b_optSwitchTimes(double varargin_2, double varargin_3, double varargin_4,
                        double varargin_5, double varargin_6, double varargin_7,
                        double t[7], double *dir, boolean_T *mod_jerk_profile)
    const;
  void c_optSwitchTimes(double varargin_2, double varargin_3, double varargin_4,
                        double varargin_5, double varargin_6, const creal_T
                        varargin_7, double t[7], double *dir, boolean_T
                        *mod_jerk_profile) const;
  void optBreaking(creal_T v_0, double a_0, double joint, creal_T *q, double
                   t_rel[3]) const;
  void getTrajectories(const coder::array<double, 2U> &varargin_2, const coder::
                       array<double, 1U> &varargin_3, const coder::array<
                       boolean_T, 1U> &varargin_4, const double varargin_5[6],
                       const double varargin_6[6], const double varargin_7[6],
                       const double varargin_8[6], double q_traj[6], double
                       v_traj[6], double a_traj[6]) const;
  double DoF;
  double Tsample;
  double j_max[6];
  double a_max[6];
  double v_max[6];
};

#ifdef _MSC_VER

#pragma warning(pop)

#endif
#endif

//
// File trailer for ltpTrajectory_types.h
//
// [EOF]
//
