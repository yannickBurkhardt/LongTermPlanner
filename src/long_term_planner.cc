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
  //// Check inputs
  bool success = checkInputs(q_0, v_0, a_0);
  if (!success) return false;
  //// Initialize values
  // Optimal jerk switching times
  std::vector<std::array<double, 7>> t_opt(dof_);
  // Scaled jerk switching times
  std::vector<std::array<double, 7>> t_scaled(dof_);
  // Direction of movement
  std::vector<double> dir(dof_);
  // Boolean to choose jerk profile
  std::vector<char> mod_jerk_profile(dof_);

  //// Find slowest joint
  for (int i=0; i<dof_; i++) {
    bool success = optSwitchTimes(i, q_goal[i], q_0[i], v_0[i], a_0[i], t_opt[i], dir[i], mod_jerk_profile[i]);
    if (!success) return false;
  }
  double t_required=-1;
  int slowest_joint=-1;
  for (int i=0; i<dof_; i++) {
    if (t_opt[i][6] > t_required) {
        t_required = t_opt[i][6];
        slowest_joint = i;
    }
  }
  if (slowest_joint == -1) return false; 

  //// Scale other joints to require same time
  std::vector<double> v_drive = v_max_;
  for (int i=0; i<dof_; i++) {
    if (i==slowest_joint) {
      continue;
    }
    timeScaling(i, q_goal[i], q_0[i], v_0[i], a_0[i], dir[i], t_required, t_scaled[i], v_drive[i], mod_jerk_profile[i]);
  }
  // If no solution was found, use optimal time
  for (int i=0; i<dof_; i++) {
    if (*std::max_element(std::begin(t_scaled[i]), std::end(t_scaled[i])) <= 0.0) {
      // t_scaled[i] = t_opt[i]
      std::copy(std::begin(t_opt[i]), std::end(t_opt[i]), std::begin(t_scaled[i]));
    }
  }

  //// Calculate sampled trajectories
  traj = getTrajectory(t_scaled, dir, mod_jerk_profile, q_0, v_0, a_0);
  return true;
}

// ===========================================================
// ================== Check Inputs ===========================
// ===========================================================
bool LongTermPlanner::checkInputs(
    const std::vector<double>& q_0,
    const std::vector<double>& v_0,
    const std::vector<double>& a_0) {
  for (int i=0; i<dof_; i++) {
    if (q_0[i] < q_min_[i] || q_0[i] > q_max_[i] || abs(v_0[i]) > v_max_[i] || abs(a_0[i]) > a_max_[i]) return false;
    if (abs(v_0[i] + 0.5*a_0[i]*abs(a_0[i])/j_max_[i]) > v_max_[i]) return false;
  }
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
    std::array<double, 7>& t,
    double& dir,
    char& mod_jerk_profile) {
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
    std::array<double, 7>& scaled_t,
    double& v_drive,
    char& scaled_mod_jerk_profile) {
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
    std::array<double, 7>& t_rel,
    double& dir) {
  return true;
}

// ===========================================================
// ================== Get Trajectory =========================
// ===========================================================
Trajectory LongTermPlanner::getTrajectory(
    const std::vector<std::array<double, 7>>& t,
    const std::vector<double>& dir,
    const std::vector<char>& mod_jerk_profile,
    const std::vector<double>& q_0,
    const std::vector<double>& v_0,
    const std::vector<double>& a_0) {
  Trajectory traj;
  return traj;
}
} // namespace long_term_planner