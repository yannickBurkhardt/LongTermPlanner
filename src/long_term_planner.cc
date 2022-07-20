#include "long_term_planner/long_term_planner.h"
#include "long_term_planner/roots.h"

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
    bool success = optSwitchTimes(i, q_goal[i], q_0[i], v_0[i], a_0[i], v_max_[i], t_opt[i], dir[i], mod_jerk_profile[i]);
    if (!success) return false;
  }
  double t_required=-1;
  int slowest_joint=-1;
  for (int i=0; i<dof_; i++) {
    if (t_opt[i][7] > t_required) {
        t_required = t_opt[i][7];
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
    if (abs(v_0[i] + 0.5*a_0[i] * abs(a_0[i])/j_max_[i]) > v_max_[i]) return false;
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
    double v_drive,
    std::array<double, 7>& t,
    double& dir,
    char& mod_jerk_profile) {
  //// Parameters for calculation
  // Time that is required for one jerk phase
  std::array<double, 7> t_rel = {}; 
  // Use the standard jerk profile if not changed during calculations
  mod_jerk_profile = 0;
  double eps = 4e-3;

  //// Calculate direction of movement
  double q_stop = 0;
  optBraking(joint, v_0, a_0, q_stop, t_rel, dir);
  double q_diff = q_goal - (q_0 + q_stop);
  if (abs(q_diff) < eps) {
      // Skip rest if that fulfils scenario
      // cumsum
      std::partial_sum(t_rel.begin(), t_rel.end(), t.begin(), std::plus<double>());
      return true;
  }
  dir = sign(q_diff);
  // If goal is in negative direction, map to pos. direction
  if(dir < 0) {
    v_0 = -v_0;
    a_0 = -a_0;
  }

  //// Calculate min. time required per joint to reach goal state
  // Check if slowing down is necessary to satisfy v_drive
  double q_brake = 0.0;
  double emp;
  if (v_0 + 0.5*a_0*abs(a_0)/j_max_[joint] > v_drive) {
    mod_jerk_profile = 1;
    // Get Joint Position after braking and required times
    optBraking(joint, v_0 - v_drive, a_0, q_brake, t_rel, emp);
  } else {
    // Constant max/ min jerk (Phase 1, 3)
    t_rel[1] = (a_max_[joint] - a_0)/j_max_[joint];
    t_rel[3] = a_max_[joint]/j_max_[joint];
    // Constant acceleration (Phase 2)
    t_rel[2] = (v_drive - v_0 - 0.5 * t_rel[1] * a_0)/a_max_[joint] - 0.5 * (t_rel[1] + t_rel[3]);
    // Check if phase 2 does not exist
    // (max acceleration cannot be reached)
    if(t_rel[2] < -eps) {
      // Check if root is positive
      double root = j_max_[joint] * (v_drive - v_0) + 0.5 * pow(a_0, 2);
      if (root > 0) {
        t_rel[3] = sqrt(root)/j_max_[joint];
        t_rel[1] = t_rel[3] - a_0/j_max_[joint];
        t_rel[2] = 0;
      } else {
        // This should never occur, only for safety
        t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return true;
      }
    }
  }

  // Constant max/ min jerk (Phase 5, 7)
  t_rel[5] = a_max_[joint]/j_max_[joint];
  t_rel[7] = t_rel[5];
  // Constant acceleration (Phase 6)
  t_rel[6] = v_drive/a_max_[joint] - 1.0/2 * (t_rel[5] + t_rel[7]);
  // Check if phase 6 does not exist
  // (max acceleration cannot be reached)
  if (t_rel[6] < -eps) {
    // Check if root is positive
    double root = v_drive/j_max_[joint];
    if (root > 0) {
      t_rel[5] = sqrt(root);
      t_rel[7] = t_rel[5];
      t_rel[6] = 0;
    } else {
      // This should never occur, only for safety
      t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      return true;
    }
  }

  // Constant velocity (Phase 4)
  double q_part1 = 0;
  if (mod_jerk_profile) {
    // Braking to satisfy v_drive
    q_part1 = q_brake + v_drive * (t_rel[1] + t_rel[2] + t_rel[3]);
  } else {
    // Acceleration to reach v_drive
    q_part1 = v_0 * (t_rel[1] + t_rel[2] + t_rel[3]) + 
              a_0 * (1.0/2 * pow(t_rel[1],2) + 
              t_rel[1] * (t_rel[2] + t_rel[3]) + 
              1.0/2 * pow(t_rel[3],2)) + 
              j_max_[joint] * (1.0/6 * pow(t_rel[1],3) + 
                               1.0/2 * pow(t_rel[1],2) * (t_rel[2] + t_rel[3]) - 
                               1.0/6 * pow(t_rel[3],3) + 
                               1.0/2 * t_rel[1] * pow(t_rel[3],2)) + 
              a_max_[joint] * (1.0/2 * pow(t_rel[2],2) + t_rel[2] * t_rel[3]);
  }
  double q_part2 = j_max_[joint] * (1.0/6 * pow(t_rel[7],3) + 
                   1.0/2 * pow(t_rel[7],2) * (t_rel[6] + t_rel[5]) - 
                   1.0/6 * pow(t_rel[5],3) + 
                   1.0/2 * t_rel[7] * pow(t_rel[5],2)) + 
                   a_max_[joint] * (1.0/2 * pow(t_rel[6],2) + 
                   t_rel[6] * t_rel[5]);
  t_rel[4] = ((q_goal - q_0) * dir - q_part1 - q_part2)/v_drive;

  // Check if phase 4 does not exist
  // (max velocity cannot be reached)
  if (t_rel[4] < -eps) {
    if (mod_jerk_profile) {
      // This case is not valid
      // (is handled in timeScaling function)
      t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      return false;
    }
    // Calculate times if max velocity is not reached 
    double root = (pow(j_max_[joint],2) * pow(t_rel[1],4))/2 - 
                  (pow(j_max_[joint],2) * pow(t_rel[3],4))/4 + 
                  (pow(j_max_[joint],2) * pow(t_rel[3],2) * pow(t_rel[5],2))/2 - 
                  (pow(j_max_[joint],2) * pow(t_rel[5],4))/4 + 
                  (pow(j_max_[joint],2) * pow(t_rel[7],4))/2 + 
                  2*j_max_[joint] * a_0 * pow(t_rel[1],3) - 
                  (2*j_max_[joint] * a_max_[joint] * pow(t_rel[1],3))/3 - 
                  2*j_max_[joint] * a_max_[joint] * t_rel[1] * pow(t_rel[3],2) + 
                  (2*j_max_[joint] * a_max_[joint] * pow(t_rel[3],3))/3 + 
                  (2*j_max_[joint] * a_max_[joint] * pow(t_rel[5],3))/3 - 
                  2*j_max_[joint] * a_max_[joint] * pow(t_rel[5],2) * t_rel[7] - 
                  (2*j_max_[joint] * a_max_[joint] * pow(t_rel[7],3))/3 + 
                  2*j_max_[joint] * v_0 * pow(t_rel[1],2) + 
                  2 * pow(a_0,2) * pow(t_rel[1],2) - 
                  2*a_0*a_max_[joint] * pow(t_rel[1],2) - 
                  2*a_0*a_max_[joint] * pow(t_rel[3],2) + 
                  4*a_0*v_0 * t_rel[1] + 
                  2 * pow(a_max_[joint],2) * pow(t_rel[3],2) + 
                  2 * pow(a_max_[joint],2) * pow(t_rel[5],2) - 
                  4*a_max_[joint] * v_0 * t_rel[1] + 
                  4*dir * (q_goal - q_0) * a_max_[joint] + 
                  2*pow(v_0,2);
    if (root > 0) {
      t_rel[6] = -(4*a_max_[joint] * t_rel[5] - 
                   2*pow(root,(1.0/2)) + 
                   j_max_[joint] * pow(t_rel[3],2) - 
                   j_max_[joint] * pow(t_rel[5],2) + 
                   2*j_max_[joint] * pow(t_rel[7],2))/(4*a_max_[joint]);
      t_rel[2] = (-v_0 - a_0 * t_rel[1] - 
                  1.0/2*j_max_[joint] * pow(t_rel[1],2) + 
                  1.0/2*j_max_[joint] * pow(t_rel[3],2) + 
                  1.0/2*j_max_[joint] * pow(t_rel[7],2) - 
                  1.0/2*j_max_[joint] * pow(t_rel[5],2))/a_max_[joint] 
                 - t_rel[3] + t_rel[6] + t_rel[5];
      t_rel[4] = 0;
    } else {
      // This should never occur, only for safety
      t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      return true;
    }

    // Check if phase 2 and/ or phase 6 does not exist 
    // (max velocity and max acceleration cannot be reached)
    if (t_rel[6] < -eps || t_rel[2] < -eps) {
      double a_4 = 12;
      double a_3 = 0;
      double a_2 = -24*pow(a_0,2) + 48*j_max_[joint] * v_0;
      double a_1 = 48 * dir * pow(j_max_[joint],2) * q_0 - 
                   48*dir * pow(j_max_[joint],2) * q_goal + 
                   16*pow(a_0,3) - 48*a_0*j_max_[joint] * v_0;
      double a_0 = -3*pow(a_0,4) + 12*pow(a_0,2)*j_max_[joint] * v_0 - 12 * pow(j_max_[joint],2) * pow(v_0,2);
      // This will be a non-complex, positive solution
      root = fourth_2deriv(a_4, a_3, a_2, a_1, a_0);

      t_rel[1] = (2*pow(root,2) - 4*a_0*root + pow(a_0,2) - 2*v_0*j_max_[joint])/(4*j_max_[joint] * root);
      // Calculate other switch times
      t_rel[7] = sqrt(4 * pow(j_max_[joint],2) * pow(t_rel[1],2) + 
                      8*a_0*j_max_[joint] * t_rel[1] + 
                      2*pow(a_0,2) + 
                      4*j_max_[joint] * v_0) / (2*j_max_[joint]);
      t_rel[5] = a_0/j_max_[joint] + t_rel[1] + t_rel[7];
      t_rel[2] = 0;
      t_rel[6] = 0;

      // Check if a_max is exceeded (Phase 2 exists)
      if (a_0 + t_rel[1] * j_max_[joint] > a_max_[joint]) {
        t_rel[1] = (a_max_[joint] - a_0) / j_max_[joint];
        t_rel[7] = 1.0/j_max_[joint] * (a_max_[joint]/2 + sqrt(
                      9 * pow(a_max_[joint],2) + 6*sqrt(
                        -12*a_max_[joint] * pow(j_max_[joint],3) * pow(t_rel[1],3) + 
                        9*pow(a_0,2) * pow(j_max_[joint],2) * pow(t_rel[1],2) - 
                        18*a_0*a_max_[joint] * pow(j_max_[joint],2) * pow(t_rel[1],2) + 
                        9 * pow(a_max_[joint],2) * pow(j_max_[joint],2) * pow(t_rel[1],2) + 
                        36*a_0 * pow(j_max_[joint],2) * t_rel[1] * v_0 - 
                        72*a_max_[joint] * dir * pow(j_max_[joint],2) * q_0 + 
                        72*a_max_[joint] * dir * pow(j_max_[joint],2) * q_goal - 
                        36*a_max_[joint] * pow(j_max_[joint],2) * t_rel[1] * v_0 + 
                        3 * pow(a_max_[joint],4) + 
                        36 * pow(j_max_[joint],2) * pow(v_0,2)))/6 - a_max_[joint]);
        t_rel[5] = t_rel[7] + a_max_[joint]/j_max_[joint];
        t_rel[2] = -(-j_max_[joint] * pow(t_rel[5],2) - 
                     2*j_max_[joint] * t_rel[5] * t_rel[7] + 
                     j_max_[joint] * pow(t_rel[7],2) + a_0 * t_rel[1] + 
                     a_max_[joint] * t_rel[1] + 
                     2*a_max_[joint] * t_rel[5] + 
                     2*a_max_[joint] * t_rel[7] + 
                     2*v_0)/(2*a_max_[joint]);
        t_rel[6] = 0;
      }

      // Check if -a_max is exceeded (Phase 6 exists)
      if (t_rel[7] * j_max_[joint] > a_max_[joint]) {
        t_rel[7] = a_max_[joint]/j_max_[joint];
        double a_4 = 12;
        double a_3 = - 24*a_max_[joint];
        double a_2 = -12*pow(a_0,2) + 12 * pow(a_max_[joint],2) + 24*j_max_[joint] * v_0;
        double a_1 = 0;
        double a_0 = 24 * dir * pow(j_max_[joint],2) * q_0*a_max_[joint] - 
                     24*dir * pow(j_max_[joint],2) * q_goal*a_max_[joint] + 
                     3*pow(a_0,4) + 8*pow(a_0,3)*a_max_[joint] + 
                     6*pow(a_0,2) * pow(a_max_[joint],2) - 
                     12*pow(a_0,2)*j_max_[joint] * v_0 - 
                     24*a_0*j_max_[joint] * v_0*a_max_[joint] - 
                     12 * pow(a_max_[joint],2) * j_max_[joint] * v_0 + 
                     12 * pow(j_max_[joint],2) * pow(v_0,2);
        // This will be a non-complex, positive solution
        root = fourth_2deriv(a_4, a_3, a_2, a_1, a_0);

        t_rel[1] = (root - a_0 - a_max_[joint])/j_max_[joint];
        // Calculate other switch times
        t_rel[5] = (a_0 + a_max_[joint])/j_max_[joint] + t_rel[1];
        t_rel[6] = (pow(j_max_[joint],2) * pow(t_rel[1],2) + 
                    2 * pow(j_max_[joint],2) * t_rel[1] * t_rel[5] - 
                    pow(j_max_[joint],2) * pow(t_rel[5],2) + 
                    2*a_0*j_max_[joint] * t_rel[1] + 2*a_0*j_max_[joint] * t_rel[5] - 
                    pow(a_max_[joint],2) + 
                    2*j_max_[joint] * v_0)/(2*j_max_[joint] * a_max_[joint]);
        t_rel[2] = 0;
      }
      // All other times are 0
      t_rel[3] = 0;
      t_rel[4] = 0;
    }
  }
  // Safety checks
  for (auto& t_rel_item : t_rel) {
    if (t_rel_item < -eps) {
      // No numeric inaccuracy
      t_rel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      break;
    } else if (t_rel_item < 0.0 && t_rel_item >= -eps) {
      t_rel_item = 0.0;
    } 
  }
  // Calculate absolute times for jerk switches
  // cumsum
  std::partial_sum(t_rel.begin(), t_rel.end(), t.begin(), std::plus<double>());
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