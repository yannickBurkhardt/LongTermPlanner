//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ltpTrajectory_initialize.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-May-2022 10:45:13
//

// Include Files
#include "ltpTrajectory_initialize.h"
#include "ltpTrajectory.h"
#include "ltpTrajectory_data.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void ltpTrajectory_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_ltpTrajectory = true;
}

//
// File trailer for ltpTrajectory_initialize.cpp
//
// [EOF]
//
