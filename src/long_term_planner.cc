#include "long_term_planner/long_term_planner.h"

namespace long_term_planner {
// ===========================================================
// ================== Plan Trajectory ========================
// ===========================================================
bool LongTermPlanner::planTrajectory(
    const std::vector<double>& q_goal,
    const std::vector<double>& q_0,
    const std::vector<double>& v_0,
    const std::vector<double>& a_0,
    Trajectory& traj) {
  return true;
}

// ===========================================================
// ================== Check Inputs ===========================
// ===========================================================
bool LongTermPlanner::checkInputs(
    const std::vector<double>& q_0,
    const std::vector<double>& v_0,
    const std::vector<double>& a_0) {
  return true;
}

// ===========================================================
// ================== Opt Switch Times =======================
// ===========================================================
bool LongTermPlanner::optSwitchTimes(int joint, 
    double q_goal, 
    double q_0, 
    double v_0, 
    double a_0, 
    std::vector<double>& t, 
    double& dir,
    bool& mod_jerk_profile) {
  return true;
}

// ===========================================================
// ================== Time Scaling ===========================
// ===========================================================
bool LongTermPlanner::timeScaling(
    int joint, 
    double q_goal, 
    double q_0, 
    double v_0, 
    double a_0, 
    double dir,
    double t_required,
    std::vector<double>& scaled_t,
    double& v_drive,
    bool& scaled_mod_jerk_profile) {
  return true;
}

// ===========================================================
// ================== Opt Braking ============================
// ===========================================================
bool LongTermPlanner::optBraking(
    int joint, 
    double v_0, 
    double a_0, 
    double& q,
    std::vector<double>& t_rel,
    double& dir) {
  return true;
}

// ===========================================================
// ================== Get Trajectory =========================
// ===========================================================
Trajectory LongTermPlanner::getTrajectory(
    const std::vector<double>& t,
    double dir,
    bool mod_jerk_profile,
    const std::vector<double>& q_0,
    const std::vector<double>& v_0,
    const std::vector<double>& a_0) {
  Trajectory traj;
  return traj;
}
} // namespace long_term_planner