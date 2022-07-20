//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ltpTrajectory.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 20-Jul-2022 14:48:22
//
#ifndef LTPTRAJECTORY_H
#define LTPTRAJECTORY_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "ltpTrajectory_types.h"

// Function Declarations
extern void ltpTrajectory(double DoF, double Tsample, const double v_max[6],
  const double a_max[6], const double j_max[6], const double q_goal[6], const
  double q_0[6], const double v_0[6], const double a_0[6], double q_traj[6],
  double v_traj[6], double a_traj[6]);

#endif

//
// File trailer for ltpTrajectory.h
//
// [EOF]
//
