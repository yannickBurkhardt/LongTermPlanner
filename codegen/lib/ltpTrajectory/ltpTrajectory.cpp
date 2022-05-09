//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ltpTrajectory.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-May-2022 10:45:13
//

// Include Files
#include "ltpTrajectory.h"
#include "LTPlanner.h"
#include "ltpTrajectory_data.h"
#include "ltpTrajectory_initialize.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// LTPTRAJECTORY Function to generate sampled trajectory using LTPlanner
//  Used to generate C++ Function
// Arguments    : double DoF
//                double Tsample
//                const double v_max[6]
//                const double a_max[6]
//                const double j_max[6]
//                const double q_goal[6]
//                const double q_0[6]
//                const double v_0[6]
//                const double a_0[6]
//                double q_traj[6]
//                double v_traj[6]
//                double a_traj[6]
// Return Type  : void
//
void ltpTrajectory(double DoF, double Tsample, const double v_max[6], const
                   double a_max[6], const double j_max[6], const double q_goal[6],
                   const double q_0[6], const double v_0[6], const double a_0[6],
                   double q_traj[6], double v_traj[6], double a_traj[6])
{
  LTPlanner ltp;
  if (!isInitialized_ltpTrajectory) {
    ltpTrajectory_initialize();
  }

  ltp.init(DoF, Tsample, v_max, a_max, j_max);
  ltp.trajectory(q_goal, q_0, v_0, a_0, q_traj, v_traj, a_traj);
}

//
// File trailer for ltpTrajectory.cpp
//
// [EOF]
//
