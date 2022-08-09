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
    bool success = optSwitchTimes(i, q_goal[i], q_0[i], v_0[i], a_0[i], v_max_[i], t_opt[i], dir[i], mod_jerk_profile[i]);
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
  traj = getTrajectory(t_scaled, dir, mod_jerk_profile, q_0, v_0, a_0, v_drive);
  for (int i = 0; i < dof_; i++) {
    if (traj.q[i][traj.length-1] < q_min_[i] || traj.q[i][traj.length-1] > q_max_[i]) return false;
  }
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
    if (abs(v_0[i] + 0.5 * a_0[i] * abs(a_0[i])/j_max_[i]) > v_max_[i]) return false;
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
  if (v_0 + 0.5 * a_0 * abs(a_0)/j_max_[joint] > v_drive) {
    mod_jerk_profile = 1;
    // Get Joint Position after braking and required times
    optBraking(joint, v_0 - v_drive, a_0, q_brake, t_rel, emp);
  } else {
    // Constant max/ min jerk (Phase 1, 3)
    t_rel[0] = (a_max_[joint] - a_0)/j_max_[joint];
    t_rel[2] = a_max_[joint]/j_max_[joint];
    // Constant acceleration (Phase 2)
    t_rel[1] = (v_drive - v_0 - 0.5 * t_rel[0] * a_0)/a_max_[joint] - 0.5 * (t_rel[0] + t_rel[2]);
    // Check if phase 2 does not exist
    // (max acceleration cannot be reached)
    if(t_rel[1] < -eps) {
      // Check if root is positive
      double root = j_max_[joint] * (v_drive - v_0) + 0.5 * pow(a_0, 2);
      if (root > 0) {
        t_rel[2] = sqrt(root)/j_max_[joint];
        t_rel[0] = t_rel[2] - a_0/j_max_[joint];
        t_rel[1] = 0;
      } else {
        // This should never occur, only for safety
        t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return true;
      }
    }
  }

  // Constant max/ min jerk (Phase 5, 7)
  t_rel[4] = a_max_[joint]/j_max_[joint];
  t_rel[6] = t_rel[4];
  // Constant acceleration (Phase 6)
  t_rel[5] = v_drive/a_max_[joint] - 1.0/2.0 * (t_rel[4] + t_rel[6]);
  // Check if phase 6 does not exist
  // (max acceleration cannot be reached)
  if (t_rel[5] < -eps) {
    // Check if root is positive
    double root = v_drive/j_max_[joint];
    if (root > 0) {
      t_rel[4] = sqrt(root);
      t_rel[6] = t_rel[4];
      t_rel[5] = 0;
    } else {
      // This should never occur, only for safety
      t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      return true;
    }
  }

  // Constant velocity (Phase 4)
  double q_part1 = 0;
  if (mod_jerk_profile == 1) {
    // Braking to satisfy v_drive
    q_part1 = q_brake + v_drive * (t_rel[0] + t_rel[1] + t_rel[2]);
  } else {
    // Acceleration to reach v_drive
    q_part1 = v_0 * (t_rel[0] + t_rel[1] + t_rel[2]) + 
              a_0 * (1.0/2.0 * pow(t_rel[0],2) + 
              t_rel[0] * (t_rel[1] + t_rel[2]) + 
              1.0/2.0 * pow(t_rel[2],2)) + 
              j_max_[joint] * (1.0/6.0 * pow(t_rel[0],3) + 
                               1.0/2.0 * pow(t_rel[0],2) * (t_rel[1] + t_rel[2]) - 
                               1.0/6.0 * pow(t_rel[2],3) + 
                               1.0/2.0 * t_rel[0] * pow(t_rel[2],2)) + 
              a_max_[joint] * (1.0/2.0 * pow(t_rel[1],2) + t_rel[1] * t_rel[2]);
  }
  double q_part2 = j_max_[joint] * (1.0/6.0 * pow(t_rel[6],3) + 
                   1.0/2.0 * pow(t_rel[6],2) * (t_rel[5] + t_rel[4]) - 
                   1.0/6.0 * pow(t_rel[4],3) + 
                   1.0/2.0 * t_rel[6] * pow(t_rel[4],2)) + 
                   a_max_[joint] * (1.0/2.0 * pow(t_rel[5],2) + 
                   t_rel[5] * t_rel[4]);
  t_rel[3] = ((q_goal - q_0) * dir - q_part1 - q_part2)/v_drive;

  // Check if phase 4 does not exist
  // (max velocity cannot be reached)
  if (t_rel[3] < -eps) {
    if (mod_jerk_profile==1) {
      // This case is not valid
      // (is handled in timeScaling function)
      t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      return false;
    }
    // Calculate times if max velocity is not reached 
    double root = (pow(j_max_[joint],2) * pow(t_rel[0],4))/2 - 
                  (pow(j_max_[joint],2) * pow(t_rel[2],4))/4 + 
                  (pow(j_max_[joint],2) * pow(t_rel[2],2) * pow(t_rel[4],2))/2 - 
                  (pow(j_max_[joint],2) * pow(t_rel[4],4))/4 + 
                  (pow(j_max_[joint],2) * pow(t_rel[6],4))/2 + 
                  2.0 * j_max_[joint] * a_0 * pow(t_rel[0],3) - 
                  (2.0 * j_max_[joint] * a_max_[joint] * pow(t_rel[0],3))/3 - 
                  2.0 * j_max_[joint] * a_max_[joint] * t_rel[0] * pow(t_rel[2],2) + 
                  (2.0 * j_max_[joint] * a_max_[joint] * pow(t_rel[2],3))/3 + 
                  (2.0 * j_max_[joint] * a_max_[joint] * pow(t_rel[4],3))/3 - 
                  2.0 * j_max_[joint] * a_max_[joint] * pow(t_rel[4],2) * t_rel[6] - 
                  (2.0 * j_max_[joint] * a_max_[joint] * pow(t_rel[6],3))/3 + 
                  2.0 * j_max_[joint] * v_0 * pow(t_rel[0],2) + 
                  2.0 * pow(a_0,2) * pow(t_rel[0],2) - 
                  2.0 * a_0 * a_max_[joint] * pow(t_rel[0],2) - 
                  2.0 * a_0 * a_max_[joint] * pow(t_rel[2],2) + 
                  4 * a_0 * v_0 * t_rel[0] + 
                  2.0 * pow(a_max_[joint],2) * pow(t_rel[2],2) + 
                  2.0 * pow(a_max_[joint],2) * pow(t_rel[4],2) - 
                  4 * a_max_[joint] * v_0 * t_rel[0] + 
                  4 * dir * (q_goal - q_0) * a_max_[joint] + 
                  2.0 * pow(v_0,2);
    if (root > 0) {
      t_rel[5] = -(4 * a_max_[joint] * t_rel[4] - 
                   2.0 * pow(root,(1.0/2)) + 
                   j_max_[joint] * pow(t_rel[2],2) - 
                   j_max_[joint] * pow(t_rel[4],2) + 
                   2.0 * j_max_[joint] * pow(t_rel[6],2))/(4 * a_max_[joint]);
      t_rel[1] = (-v_0 - a_0 * t_rel[0] - 
                  1.0/2.0 * j_max_[joint] * pow(t_rel[0],2) + 
                  1.0/2.0 * j_max_[joint] * pow(t_rel[2],2) + 
                  1.0/2.0 * j_max_[joint] * pow(t_rel[6],2) - 
                  1.0/2.0 * j_max_[joint] * pow(t_rel[4],2))/a_max_[joint] 
                 - t_rel[2] + t_rel[5] + t_rel[4];
      t_rel[3] = 0;
    } else {
      // This should never occur, only for safety
      t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      return true;
    }

    // Check if phase 2 and/ or phase 6 does not exist 
    // (max velocity and max acceleration cannot be reached)
    if (t_rel[5] < -eps || t_rel[1] < -eps) {
      double A_4 = 12;
      double A_3 = 0;
      double A_2 = -24 * pow(a_0,2) + 48 * j_max_[joint] * v_0;
      double A_1 = 48 * dir * pow(j_max_[joint],2) * q_0 - 
                   48 * dir * pow(j_max_[joint],2) * q_goal + 
                   16 * pow(a_0,3) - 48 * a_0 * j_max_[joint] * v_0;
      double A_0 = -3 * pow(a_0,4) + 12.0 * pow(a_0,2) * j_max_[joint] * v_0 - 12.0 * pow(j_max_[joint],2) * pow(v_0,2);
      // 2nd order deriv root finding
      // root = fourth_2deriv(A_4, A_3, A_2, A_1, A_0);
      // Companion matrix root finding
      {
        Eigen::VectorXd poly_vals(5); 
        poly_vals << A_4, A_3, A_2, A_1, A_0;
        Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> result = roots<double>(poly_vals);
        root = getSmallestPositiveNonComplexRoot<double>(result);
      }
      t_rel[0] = (2.0 * pow(root,2) - 4 * a_0*root + pow(a_0,2) - 2.0 * v_0 * j_max_[joint])/(4 * j_max_[joint] * root);
      // Calculate other switch times
      t_rel[6] = sqrt(4 * pow(j_max_[joint],2) * pow(t_rel[0],2) + 
                      8 * a_0 * j_max_[joint] * t_rel[0] + 
                      2.0 * pow(a_0,2) + 
                      4 * j_max_[joint] * v_0) / (2.0 * j_max_[joint]);
      t_rel[4] = a_0/j_max_[joint] + t_rel[0] + t_rel[6];
      t_rel[1] = 0;
      t_rel[5] = 0;

      // Check if a_max is exceeded (Phase 2 exists)
      if (a_0 + t_rel[0] * j_max_[joint] > a_max_[joint]) {
        t_rel[0] = (a_max_[joint] - a_0) / j_max_[joint];
        t_rel[6] = 1.0/j_max_[joint] * (a_max_[joint]/2 + sqrt(
                      9 * pow(a_max_[joint],2) + 6*sqrt(
                        -12.0 * a_max_[joint] * pow(j_max_[joint],3) * pow(t_rel[0],3) + 
                        9 * pow(a_0,2) * pow(j_max_[joint],2) * pow(t_rel[0],2) - 
                        18 * a_0 * a_max_[joint] * pow(j_max_[joint],2) * pow(t_rel[0],2) + 
                        9 * pow(a_max_[joint],2) * pow(j_max_[joint],2) * pow(t_rel[0],2) + 
                        36 * a_0 * pow(j_max_[joint],2) * t_rel[0] * v_0 - 
                        72.0 * a_max_[joint] * dir * pow(j_max_[joint],2) * q_0 + 
                        72.0 * a_max_[joint] * dir * pow(j_max_[joint],2) * q_goal - 
                        36 * a_max_[joint] * pow(j_max_[joint],2) * t_rel[0] * v_0 + 
                        3 * pow(a_max_[joint],4) + 
                        36 * pow(j_max_[joint],2) * pow(v_0,2)))/6.0 - a_max_[joint]);
        t_rel[4] = t_rel[6] + a_max_[joint]/j_max_[joint];
        t_rel[1] = -(-j_max_[joint] * pow(t_rel[4],2) - 
                     2.0 * j_max_[joint] * t_rel[4] * t_rel[6] + 
                     j_max_[joint] * pow(t_rel[6],2) + a_0 * t_rel[0] + 
                     a_max_[joint] * t_rel[0] + 
                     2.0 * a_max_[joint] * t_rel[4] + 
                     2.0 * a_max_[joint] * t_rel[6] + 
                     2.0 * v_0)/(2.0 * a_max_[joint]);
        t_rel[5] = 0;
      }

      // Check if -a_max is exceeded (Phase 6 exists)
      if (t_rel[6] * j_max_[joint] > a_max_[joint]) {
        t_rel[6] = a_max_[joint]/j_max_[joint];
        double A_4 = 12;
        double A_3 = - 24 * a_max_[joint];
        double A_2 = -12.0 * pow(a_0,2) + 12.0 * pow(a_max_[joint],2) + 24 * j_max_[joint] * v_0;
        double A_1 = 0;
        double A_0 = 24 * dir * pow(j_max_[joint],2) * q_0 * a_max_[joint] - 
                     24 * dir * pow(j_max_[joint],2) * q_goal * a_max_[joint] + 
                     3 * pow(a_0,4) + 8 * pow(a_0,3) * a_max_[joint] + 
                     6 * pow(a_0,2) * pow(a_max_[joint],2) - 
                     12.0 * pow(a_0,2) * j_max_[joint] * v_0 - 
                     24 * a_0 * j_max_[joint] * v_0 * a_max_[joint] - 
                     12.0 * pow(a_max_[joint],2) * j_max_[joint] * v_0 + 
                     12.0 * pow(j_max_[joint],2) * pow(v_0,2);
        // 2nd order deriv root finding
        // root = fourth_2deriv(A_4, A_3, A_2, A_1, A_0);
        // Companion matrix root finding
        {
          Eigen::VectorXd poly_vals(5); 
          poly_vals << A_4, A_3, A_2, A_1, A_0;
          Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> result = roots<double>(poly_vals);
          root = getSmallestPositiveNonComplexRoot<double>(result);
        }
        t_rel[0] = (root - a_0 - a_max_[joint])/j_max_[joint];
        // Calculate other switch times
        t_rel[4] = (a_0 + a_max_[joint])/j_max_[joint] + t_rel[0];
        t_rel[5] = (pow(j_max_[joint],2) * pow(t_rel[0],2) + 
                    2.0 * pow(j_max_[joint],2) * t_rel[0] * t_rel[4] - 
                    pow(j_max_[joint],2) * pow(t_rel[4],2) + 
                    2.0 * a_0 * j_max_[joint] * t_rel[0] + 
                    2.0 * a_0 * j_max_[joint] * t_rel[4] - 
                    pow(a_max_[joint],2) + 
                    2.0 * j_max_[joint] * v_0)/(2.0 * j_max_[joint] * a_max_[joint]);
        t_rel[1] = 0;
      }
      // All other times are 0
      t_rel[2] = 0;
      t_rel[3] = 0;
    }
  }
  // Safety checks
  for (int i=0; i<7;i++) {
    if (t_rel[i] < -eps) {
      // No numeric inaccuracy
      std::cerr << "t_rel[" << i << "] = " << t_rel[i] << " < -eps. q_0 = " << q_0 << ", v_0 = " << v_0 << ", a_0 = " << a_0 << ", g_goal = " << q_goal << std::endl;
      return false;
    } else if (t_rel[i] < 0.0 && t_rel[i] >= -eps) {
      t_rel[i] = 0.0;
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
    char& mod_jerk_profile) {
  // Parameters for calculation
  double tol = 0.1;
  // If goal is in negative direction, map to pos. direction
  if (dir < 0) {
    v_0 = -v_0;
    a_0 = -a_0;
  }
  //// Calculate required v_drive to reach goal at given time
  // Standard jerk profile: Phases 2 and 6 exist
  v_drive = (a_max_[joint] * j_max_[joint] * t_required/2 - 
             pow(a_0,2)/4 + a_0 * a_max_[joint]/2 - 
             pow(a_max_[joint],2)/2 + 
             v_0 * j_max_[joint]/2 - 
             sqrt(36 * pow(a_max_[joint],2) * pow(j_max_[joint],2) * pow(t_required,2) - 
                  36 * pow(a_0,2) * a_max_[joint] * j_max_[joint] * t_required + 
                  72.0 * a_0 * pow(a_max_[joint],2) * j_max_[joint] * t_required - 
                  72.0 * pow(a_max_[joint],3) * j_max_[joint] * t_required + 
                  144 * a_max_[joint] * dir * pow(j_max_[joint],2) * q_0 - 
                  144 * a_max_[joint] * dir * pow(j_max_[joint],2) * q_goal + 
                  72.0 * a_max_[joint] * pow(j_max_[joint],2) * v_0 * t_required 
                  - 9 * pow(a_0,4) 
                  + 12.0 * pow(a_0,3) * a_max_[joint] 
                  + 36 * pow(a_0,2) * pow(a_max_[joint],2) + 
                  36 * pow(a_0,2) * j_max_[joint] * v_0 - 
                  72.0 * a_0 * pow(a_max_[joint],3) - 
                  72.0 * a_0 * a_max_[joint] * j_max_[joint] * v_0 + 
                  36 * pow(a_max_[joint],4) - 
                  36 * pow(j_max_[joint],2) * pow(v_0,2))/12)/j_max_[joint];
  // Check if v_drive is real and positive
  if (!isnan(v_drive) && v_drive > 0) {
    double trash;
    bool success = optSwitchTimes(joint, q_goal, q_0, dir * v_0, dir * a_0, v_drive, scaled_t, trash, mod_jerk_profile);
    // Check time constraint was fulfilled
    if (success && t_required - scaled_t.back() < tol && t_required - scaled_t.back() > -tol/10) {
      return true;
    }
  }

  // Modified jerk profile: Phases 2 and 6 exist
  v_drive = -(dir * (q_0 - q_goal) - j_max_[joint] * (
              pow(a_0 + a_max_[joint], 3) / (6 * pow(j_max_[joint],3)) - 
              pow(a_max_[joint],3)/(6 * pow(j_max_[joint],3)) + 
              (pow(a_max_[joint],2) * (a_0 + a_max_[joint]))/(2.0 * pow(j_max_[joint],3)) + 
              (pow(a_0 + a_max_[joint], 2) * 
              ((v_0 + (a_0 * (a_0 - a_max_[joint]))/(2.0 * j_max_[joint])) / a_max_[joint] + 
              a_max_[joint]/(2.0 * j_max_[joint]) + 
              (a_0 - a_max_[joint])/(2.0 * j_max_[joint])))/(2.0 * pow(j_max_[joint],2))) + 
              a_0 * (pow(a_0 + a_max_[joint],2)/(2.0 * pow(j_max_[joint],2)) + 
              pow(a_max_[joint],2)/(2.0 * pow(j_max_[joint],2)) + 
              ((a_0 + a_max_[joint]) * ((v_0 + (a_0 * (a_0 - a_max_[joint]))/(2.0 * j_max_[joint]))/a_max_[joint] + 
              a_max_[joint]/(2.0 * j_max_[joint]) + 
              (a_0 - a_max_[joint])/(2.0 * j_max_[joint])))/j_max_[joint]) - 
              a_max_[joint] * (
              pow((v_0 + (a_0 * (a_0 - a_max_[joint]))/(2.0 * j_max_[joint]))/a_max_[joint] - a_max_[joint]/(2.0 * j_max_[joint]) + (a_0 - a_max_[joint])/(2.0 * j_max_[joint]), 2)/2 + 
              (a_max_[joint] * ((v_0 + (a_0 * (a_0 - a_max_[joint]))/(2.0 * j_max_[joint]))/a_max_[joint] - a_max_[joint]/(2.0 * j_max_[joint]) + 
              (a_0 - a_max_[joint])/(2.0 * j_max_[joint])))/j_max_[joint]) +
              v_0 * ((v_0 + (a_0 * (a_0 - a_max_[joint]))/(2.0 * j_max_[joint]))/a_max_[joint] + 
              (a_0 + a_max_[joint])/j_max_[joint] + a_max_[joint]/(2.0 * j_max_[joint]) + 
              (a_0 - a_max_[joint])/(2.0 * j_max_[joint]))) / 
              (a_max_[joint] / (2.0 * j_max_[joint]) - 
              v_0/a_max_[joint] + a_max_[joint] * (((v_0 + (a_0 * (a_0 - a_max_[joint])) / (2.0 * j_max_[joint])) /
              a_max_[joint] - a_max_[joint]/(2.0 * j_max_[joint]) + 
              (a_0 - a_max_[joint])/(2.0 * j_max_[joint]))/a_max_[joint] + 1.0/j_max_[joint]) - 
              (pow(a_0,2) + 2.0 * a_0 * a_max_[joint] + 
              4 * pow(a_max_[joint],2) - 2.0 * j_max_[joint] * t_required * a_max_[joint] + 
              2.0 * j_max_[joint] * v_0)/(2.0 * a_max_[joint] * j_max_[joint]) + 
              pow(a_0 + a_max_[joint], 2)/(2.0 * a_max_[joint] * j_max_[joint]) - 
              (a_0 * (a_0 + a_max_[joint]))/(a_max_[joint] * j_max_[joint]));
  
  // Check if v_drive is real and positive
  if (!isnan(v_drive) && v_drive > 0) {
    double trash;
    bool success = optSwitchTimes(joint, q_goal, q_0, dir * v_0, dir * a_0, v_drive, scaled_t, trash, mod_jerk_profile);
    // Check time constraint was fulfilled
    if (success && t_required - scaled_t.back() < tol && t_required - scaled_t.back() > -tol/10) {
      return true;
    }
  }

  // Standard jerk profile: Phase 2 does not exist
  double A_4 = 3;
  double A_3 = 12.0 * a_max_[joint];
  double A_2 = -24 * a_max_[joint] * j_max_[joint] * t_required - 12.0 * pow(a_0,2) - 24 * a_0 * a_max_[joint] + 12.0 * pow(a_max_[joint],2) + 24 * j_max_[joint] * v_0;
  double A_1 = 0;
  double A_0 = 48 * pow(a_0,2) * a_max_[joint] * j_max_[joint] * t_required - 
               96 * dir * pow(j_max_[joint],2) * a_max_[joint] * q_0 + 
               96 * dir * pow(j_max_[joint],2) * a_max_[joint] * q_goal - 
               96 * a_max_[joint] * pow(j_max_[joint],2) * v_0 * t_required + 
               12.0 * pow(a_0,4) + 
               16 * pow(a_0,3) * a_max_[joint] - 
               24 * pow(a_0,2) * pow(a_max_[joint],2) - 
               48 * pow(a_0,2) * j_max_[joint] * v_0 + 
               48 * pow(a_max_[joint],2) * j_max_[joint] * v_0 + 
               48 * pow(j_max_[joint],2) * pow(v_0,2);
  // 2nd order deriv root finding
  // root = fourth_2deriv(A_4, A_3, A_2, A_1, A_0);
  // Companion matrix root finding
  double root;
  {
    Eigen::VectorXd poly_vals(5); 
    poly_vals << A_4, A_3, A_2, A_1, A_0;
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> result = roots<double>(poly_vals);
    root = getSmallestPositiveNonComplexRoot<double>(result);
  }
  v_drive = (-2.0 * pow(a_0,2) + 4 * j_max_[joint] * v_0 + pow(root,2))/(4 * j_max_[joint]);
  // Check if v_drive is real and positive
  if (!isnan(v_drive) && v_drive > 0) {
    double trash;
    bool success = optSwitchTimes(joint, q_goal, q_0, dir * v_0, dir * a_0, v_drive, scaled_t, trash, mod_jerk_profile);
    // Check time constraint was fulfilled
    if (success && t_required - scaled_t.back() < tol && t_required - scaled_t.back() > -tol/10) {
      return true;
    }
  }

  // Standard jerk profile: Phase 6 does not exist
  A_4 = 12;
  A_3 = 24 * a_max_[joint];
  A_2 = -24 * a_max_[joint] * j_max_[joint] * t_required + 24 * pow(a_0,2) - 48 * a_0 * a_max_[joint] + 24 * pow(a_max_[joint],2) - 24 * j_max_[joint] * v_0 + 12.0 * a_0 - 12.0 * a_max_[joint];
  A_1 = 0;
  A_0 = -24 * dir * pow(j_max_[joint],2) * a_max_[joint] * q_0 + 
               24 * dir * pow(j_max_[joint],2) * a_max_[joint] * q_goal + 
               9 * pow(a_0,4) - 
               12.0 * pow(a_0,3) * a_max_[joint] - 
               24 * pow(a_0,2) * j_max_[joint] * v_0 + 
               48 * a_0 * a_max_[joint] * j_max_[joint] * v_0 + 
               4 * pow(a_max_[joint],4) - 
               24 * pow(a_max_[joint],2) * j_max_[joint] * v_0 + 
               12.0 * pow(j_max_[joint],2) * pow(v_0,2) + 
               6 * pow(a_0,3) + 
               6 * pow(a_0,2) * a_max_[joint] - 
               12.0 * a_0 * pow(a_max_[joint],2) - 
               12.0 * a_0 * j_max_[joint] * v_0 + 
               12.0 * a_max_[joint] * j_max_[joint] * v_0 + 
               4 * a_0 * a_max_[joint] - 
               4 * pow(a_max_[joint],2);
  // 2nd order deriv root finding
  // root = fourth_2deriv(A_4, A_3, A_2, A_1, A_0);
  // Companion matrix root finding
  {
    Eigen::VectorXd poly_vals(5); 
    poly_vals << A_4, A_3, A_2, A_1, A_0;
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> result = roots<double>(poly_vals);
    root = getSmallestPositiveNonComplexRoot<double>(result);
  }
  v_drive = pow(root,2)/j_max_[joint];
  // Check if v_drive is real and positive
  if (!isnan(v_drive) && v_drive > 0) {
    double trash;
    bool success = optSwitchTimes(joint, q_goal, q_0, dir * v_0, dir * a_0, v_drive, scaled_t, trash, mod_jerk_profile);
    // Check time constraint was fulfilled
    if (success && t_required - scaled_t.back() < tol && t_required - scaled_t.back() > -tol/10) {
      return true;
    }
  }

  // Standard jerk profile: Phases 2 and 6 do not exist
  double A_5 = (144 * j_max_[joint] * t_required + 144 * a_0);
  A_4 = (-72.0 * pow(j_max_[joint],2) * pow(t_required,2) - 144 * a_0 * j_max_[joint] * t_required + 36 * pow(a_0,2) - 216 * j_max_[joint] * v_0);
  A_3 = (144 * dir * pow(j_max_[joint],2) * q_0 - 144 * dir * pow(j_max_[joint],2) * q_goal + 48 * pow(a_0,3) - 144 * a_0 * j_max_[joint] * v_0);
  A_2 = (-144 * dir * pow(j_max_[joint],3) * q_0 * t_required + 144 * dir * pow(j_max_[joint],3) * q_goal * t_required - 48 * pow(a_0,3) * j_max_[joint] * t_required - 144 * a_0 * dir * pow(j_max_[joint],2) * q_0 + 144 * a_0 * dir * pow(j_max_[joint],2) * q_goal + 144 * a_0 * pow(j_max_[joint],2) * v_0 * t_required + 6 * pow(a_0,4) - 72.0 * pow(a_0,2) * j_max_[joint] * v_0 + 216 * pow(j_max_[joint],2) * pow(v_0,2));
  A_1 = 0;
  A_0 = -72.0 * pow(dir,2) * pow(j_max_[joint],4) * pow(q_0,2) + 144 * pow(dir,2) * pow(j_max_[joint],4) * q_0 * q_goal - 72.0 * pow(dir,2) * pow(j_max_[joint],4) * pow(q_goal,2) - 48 * pow(a_0,3) * dir * pow(j_max_[joint],2) * q_0 + 48 * pow(a_0,3) * dir * pow(j_max_[joint],2) * q_goal + 144 * a_0 * dir * pow(j_max_[joint],3) * q_0 * v_0 - 144 * a_0 * dir * pow(j_max_[joint],3) * q_goal * v_0 + pow(a_0,6) - 6 * pow(a_0,4) * j_max_[joint] * v_0 + 36 * pow(a_0,2) * pow(j_max_[joint],2) * pow(v_0,2) - 72.0 * pow(j_max_[joint],3) * pow(v_0,3);
  // 2nd order deriv root finding
  // root = fifth_2deriv(A_5, A_4, A_3, A_2, A_1, A_0);
  // Companion matrix root finding
  {
    Eigen::VectorXd poly_vals(6); 
    poly_vals << A_5, A_4, A_3, A_2, A_1, A_0;
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> result = roots<double>(poly_vals);
    root = getSmallestPositiveNonComplexRoot<double>(result);
  }
  v_drive = pow(root,2)/j_max_[joint];
  // Check if v_drive is real and positive
  if (!isnan(v_drive) && v_drive > 0) {
    double trash;
    bool success = optSwitchTimes(joint, q_goal, q_0, dir * v_0, dir * a_0, v_drive, scaled_t, trash, mod_jerk_profile);
    // Check time constraint was fulfilled
    if (success && t_required - scaled_t.back() < tol && t_required - scaled_t.back() > -tol/10) {
      return true;
    }
  }

  // Modified profile: Phase 2 does not exist
  A_4 = 3;
  A_3 = - 6*sqrt(2) * a_max_[joint];
  A_2 = (12.0 * a_max_[joint] * j_max_[joint] * t_required - 6 * pow(a_0,2) - 12.0 * a_0 * a_max_[joint] - 6 * pow(a_max_[joint],2) - 12.0 * j_max_[joint] * v_0);
  A_1 = 0;
  A_0 = -12.0 * pow(a_0,2) * a_max_[joint] * j_max_[joint] * t_required - 24 * dir * pow(j_max_[joint],2) * a_max_[joint] * q_0 + 24 * dir * pow(j_max_[joint],2) * a_max_[joint] * q_goal - 24 * a_max_[joint] * pow(j_max_[joint],2) * v_0 * t_required + 3 * pow(a_0,4) + 4 * pow(a_0,3) * a_max_[joint] + 6 * pow(a_0,2) * pow(a_max_[joint],2) + 12.0 * pow(a_0,2) * j_max_[joint] * v_0 + 12.0 * pow(a_max_[joint],2) * j_max_[joint] * v_0 + 12.0 * pow(j_max_[joint],2) * pow(v_0,2);
  // 2nd order deriv root finding
  // root = fourth_2deriv(A_4, A_3, A_2, A_1, A_0);
  // Companion matrix root finding
  {
    Eigen::VectorXd poly_vals(5); 
    poly_vals << A_4, A_3, A_2, A_1, A_0;
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> result = roots<double>(poly_vals);
    root = getSmallestPositiveNonComplexRoot<double>(result);
  }
  v_drive = -(pow(root,2) - pow(a_0,2) - 2.0 * j_max_[joint] * v_0)/(2.0 * j_max_[joint]);
  // Check if v_drive is real and positive
  if (!isnan(v_drive) && v_drive > 0) {
    double trash;
    bool success = optSwitchTimes(joint, q_goal, q_0, dir * v_0, dir * a_0, v_drive, scaled_t, trash, mod_jerk_profile);
    // Check time constraint was fulfilled
    if (success && t_required - scaled_t.back() < tol && t_required - scaled_t.back() > -tol/10) {
      return true;
    }
  }

  // Modified profile: Phase 6 does not exist
  A_4 = 12;
  A_3 = - 24 * a_max_[joint];
  A_2 = (24 * a_max_[joint] * j_max_[joint] * t_required - 12.0 * pow(a_0,2) - 24 * a_0 * a_max_[joint] - 12.0 * pow(a_max_[joint],2) - 24 * j_max_[joint] * v_0);
  A_1 = 0;
  A_0 = 24 * dir * pow(j_max_[joint],2) * a_max_[joint] * q_0 - 24 * dir * pow(j_max_[joint],2) * a_max_[joint] * q_goal + 3 * pow(a_0,4) + 8 * pow(a_0,3) * a_max_[joint] + 6 * pow(a_0,2) * pow(a_max_[joint],2) + 12.0 * pow(a_0,2) * j_max_[joint] * v_0 + 24 * a_0 * a_max_[joint] * j_max_[joint] * v_0 + 12.0 * pow(a_max_[joint],2) * j_max_[joint] * v_0 + 12.0 * pow(j_max_[joint],2) * pow(v_0,2);
  // 2nd order deriv root finding
  // root = fourth_2deriv(A_4, A_3, A_2, A_1, A_0);
  // Companion matrix root finding
  {
    Eigen::VectorXd poly_vals(5); 
    poly_vals << A_4, A_3, A_2, A_1, A_0;
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> result = roots<double>(poly_vals);
    root = getSmallestPositiveNonComplexRoot<double>(result);
  }
  v_drive = pow(root,2)/j_max_[joint];
  
  // Check if v_drive is real and positive
  if (!isnan(v_drive) && v_drive > 0) {
    double trash;
    bool success = optSwitchTimes(joint, q_goal, q_0, dir * v_0, dir * a_0, v_drive, scaled_t, trash, mod_jerk_profile);
    // Check time constraint was fulfilled
    if (success && t_required - scaled_t.back() < tol && t_required - scaled_t.back() > -tol/10) {
      return true;
    }
  }

  // Modified profile: Phases 2 and 6 do not exist
  double A_6 = 144;
  A_5 = (-144 * j_max_[joint] * t_required + 144 * a_0);
  A_4 = (72.0 * pow(j_max_[joint],2) * pow(t_required,2) - 144 * a_0 * j_max_[joint] * t_required - 36 * pow(a_0,2) - 216 * j_max_[joint] * v_0);
  A_3 = (-144 * dir * pow(j_max_[joint],2) * q_0 + 144 * dir * pow(j_max_[joint],2) * q_goal - 48 * pow(a_0,3) - 144 * a_0 * j_max_[joint] * v_0);
  A_2 = (144 * dir * pow(j_max_[joint],3) * q_0 * t_required - 144 * dir * pow(j_max_[joint],3) * q_goal * t_required + 48 * pow(a_0,3) * j_max_[joint] * t_required - 144 * a_0 * dir * pow(j_max_[joint],2) * q_0 + 144 * a_0 * dir * pow(j_max_[joint],2) * q_goal + 144 * a_0 * pow(j_max_[joint],2) * v_0 * t_required + 6 * pow(a_0,4) + 72.0 * pow(a_0,2) * j_max_[joint] * v_0 + 216 * pow(j_max_[joint],2) * pow(v_0,2));
  A_1 = 0;
  A_0 = 72.0 * pow(dir,2) * pow(j_max_[joint],4) * pow(q_0,2) -
               144 * pow(dir,2) * pow(j_max_[joint],4) * q_0 * q_goal + 
               72.0 * pow(dir,2) * pow(j_max_[joint],4) * pow(q_goal,2) + 
               48 * pow(a_0,3) * dir * pow(j_max_[joint],2) * q_0 - 
               48 * pow(a_0,3) * dir * pow(j_max_[joint],2) * q_goal + 
               144 * a_0 * dir * pow(j_max_[joint],3) * q_0 * v_0 - 
               144 * a_0 * dir * pow(j_max_[joint],3) * q_goal * v_0 - pow(a_0,6) - 
               6 * pow(a_0,4) * j_max_[joint] * v_0 - 
               36 * pow(a_0,2) * pow(j_max_[joint],2) * pow(v_0,2) - 
               72.0 * pow(j_max_[joint],3) * pow(v_0,3);
  {
    Eigen::VectorXd poly_vals(7); 
    poly_vals << A_6, A_5, A_4, A_3, A_2, A_1, A_0;
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> result = roots(poly_vals);
    root = getSmallestPositiveNonComplexRoot(result);
  }
  // WAS root(4) --> Debug this
  v_drive = pow(root,2)/j_max_[joint];
  // Check if v_drive is real and positive
  if (!isnan(v_drive) && v_drive > 0) {
    double trash;
    bool success = optSwitchTimes(joint, q_goal, q_0, dir * v_0, dir * a_0, v_drive, scaled_t, trash, mod_jerk_profile);
    // Check time constraint was fulfilled
    if (success && t_required - scaled_t.back() < tol && t_required - scaled_t.back() > -tol/10) {
      return true;
    }
  }

  // No valid solution found, reset return parameters
  mod_jerk_profile = 0;
  scaled_t = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  v_drive = v_max_[joint];
  return false;
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
  // Set direction opposite to maximal velocity to be reached
  if (v_0*a_0 > 0) {
    // v and a in same direction
    dir = -sign(v_0);
  } else {
    // If initial acceleration will cause the robot to
    // eventually move into opposite direction of velocity, use 
    // this direction
    if (abs(v_0) > 1.0/2.0*pow(a_0,2)/j_max_[joint]) {
      dir = -sign(v_0);
    } else {
      dir = -sign(a_0);
    }
  }

  // If stopping dir. is negative, map scenario to pos. direction
  if (dir < 0) {
    a_0 = -a_0;
    v_0 = -v_0;
  }

  // Bring velocity to zero
  t_rel[0] = (a_max_[joint] - a_0)/j_max_[joint];
  t_rel[2] = a_max_[joint]/j_max_[joint];
  t_rel[1] = (- v_0 - 1.0/2.0*t_rel[0]*a_0)/a_max_[joint] - 1.0/2.0*(t_rel[0] + t_rel[2]);
  
  // Check if phase 2 does not exist 
  // (max acceleration is not reached)
  if (t_rel[1] < -t_sample_) {
    t_rel[0] = -a_0/j_max_[joint] + sqrt(pow(a_0,2)/(2*pow(j_max_[joint],2)) - v_0/j_max_[joint]);
    t_rel[2] = t_rel[0] + a_0/j_max_[joint];
    t_rel[1] = 0;
  }
  
  // Calculate position after breaking
  q = v_0*(t_rel[0] + t_rel[1] + t_rel[2]) + 
      a_0*(1.0/2.0*pow(t_rel[0],2) + t_rel[0]*(t_rel[1] + t_rel[2]) + 1.0/2.0*pow(t_rel[2],2)) + 
      j_max_[joint]*(1.0/6.0*pow(t_rel[0],3) + 1.0/2.0*pow(t_rel[0],2)*(t_rel[1] + t_rel[2]) - 
      1.0/6.0*pow(t_rel[2],3) + 1.0/2.0*t_rel[0]*pow(t_rel[2],2)) + 
      a_max_[joint]*(1.0/2.0*pow(t_rel[1],2) + t_rel[1]*t_rel[2]);

  // Correct direction
  q = dir * q;
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
    const std::vector<double>& a_0,
    const std::vector<double>& v_drive) {
  Trajectory traj;
  // Calculate length of trajectory in samples
  int traj_len = 0;
  for (int i=0; i<dof_; i++) {
    traj_len = std::max(traj_len, (int)ceil(t[i][6]/t_sample_) + 1);
  }
  // Jerk switch times in samples
  std::vector<std::array<int, 7>> sampled_t(dof_);

  //// Calculate jerk trajectories
  // j_traj[joint][sample]
  std::vector<std::vector<double>> j_traj(dof_, std::vector<double>(traj_len, 0.0));
  std::vector<std::vector<double>> a_traj(dof_, std::vector<double>(traj_len, 0.0));
  std::vector<std::vector<double>> v_traj(dof_, std::vector<double>(traj_len, 0.0));
  std::vector<std::vector<double>> q_traj(dof_, std::vector<double>(traj_len, 0.0));
  for (int joint=0; joint<dof_; joint++) {
    // Save fractions lost when discretizing switch times
    std::array<double, 7> sampled_t_trans = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    std::array<double, 7> jerk_profile = {};
    // Check which jerk profile to use
    std::array<int, 7> profile = {};
    if (mod_jerk_profile[joint] == 1) {
      // Only used for specific scenarios in time scaling
      profile = {-1, 0, 1, 0, -1, 0, 1};
    } else {
      // Standard case
      profile = {1, 0, -1, 0, -1, 0, 1};
    }
    for (int j=0; j<7; j++) {
      jerk_profile[j] = dir[joint] * j_max_[joint] * profile[j];
    }
    // Calculate inaccuraties when sampling times
    for (int j=0; j<7; j++) {
      sampled_t_trans[j] = t[joint][j] - t_sample_ * floor(t[joint][j]/t_sample_);
    }

    // Round towards phases with zero jerk
    sampled_t[joint][0] = (int) floor(t[joint][0]/t_sample_);
    sampled_t[joint][1] = (int) ceil(t[joint][1]/t_sample_);
    sampled_t[joint][2] = (int) floor(t[joint][2]/t_sample_);
    sampled_t[joint][3] = (int) ceil(t[joint][3]/t_sample_);
    sampled_t[joint][4] = (int) floor(t[joint][4]/t_sample_);
    sampled_t[joint][5] = (int) ceil(t[joint][5]/t_sample_);
    sampled_t[joint][6] = (int) floor(t[joint][6]/t_sample_);
    // Calculate sampled jerk trajectory
    if (sampled_t[joint][0] > 0) {
      std::fill(j_traj[joint].begin(), (j_traj[joint].begin()+sampled_t[joint][0]), jerk_profile[0]);
    }
    for (int j=1; j<7; j++) {
      if (sampled_t[joint][j] - sampled_t[joint][j-1] > 0) {
        std::fill(j_traj[joint].begin()+sampled_t[joint][j-1], (j_traj[joint].begin()+sampled_t[joint][j]), jerk_profile[j]);
      }
    }
    //// Add partial jerk of sample fractions to increase accuracy
    if(sampled_t[joint][2] >= sampled_t[joint][1]) {
      // Phase 2 exists: 
      // Fractions can be added to its beginning and end
      j_traj[joint][sampled_t[joint][0] + 1] = j_traj[joint][sampled_t[joint][0] + 1] + sampled_t_trans[0]/t_sample_ * jerk_profile[0];
      if (sampled_t[joint][1] > 0) {
        j_traj[joint][sampled_t[joint][1]] = j_traj[joint][sampled_t[joint][1]] + (1 - sampled_t_trans[1]/t_sample_) * jerk_profile[2];
      }
      // Add fraction to end of phase 3
      j_traj[joint][sampled_t[joint][2] + 1] = j_traj[joint][sampled_t[joint][2] + 1] + sampled_t_trans[2]/t_sample_ * jerk_profile[2];
    } else {
      // Phase 2 does not exist: 
      // Calculate transition sample
      if (sampled_t[joint][1] > 0) {
        j_traj[joint][sampled_t[joint][1]] = j_traj[joint][sampled_t[joint][1]] + sampled_t_trans[0]/t_sample_ * jerk_profile[0] + (sampled_t_trans[2]-sampled_t_trans[0])/t_sample_ * jerk_profile[2];
      }
    }
    
    // Add fraction to end of phase 4
    if (sampled_t[joint][3] > 0) {
      j_traj[joint][sampled_t[joint][3]] = j_traj[joint][sampled_t[joint][3]] + (1 - sampled_t_trans[3]/t_sample_) * jerk_profile[4];
    }
    
    if (sampled_t[joint][2] - sampled_t[joint][0] > 0) {
      // Phase 2 and/ or 3 exist:
      // Add fraction to beginning of phase 6
      j_traj[joint][sampled_t[joint][4] + 1] = j_traj[joint][sampled_t[joint][4] + 1] + sampled_t_trans[4]/t_sample_ * jerk_profile[4];
    } else {
      // Phase 2 and 3 do not exist:
      // Add fractions from previous phases to end of phase 5
      if (sampled_t[joint][4] > 0) {
        j_traj[joint][sampled_t[joint][4]] = j_traj[joint][sampled_t[joint][4]] + sampled_t_trans[4]/t_sample_ * jerk_profile[4]  + sampled_t_trans[0]/t_sample_ * jerk_profile[0] + (sampled_t_trans[2]-sampled_t_trans[0])/t_sample_ * jerk_profile[2];
      }
    }
    
    // Add fraction to end of phase 4
    if (sampled_t[joint][5] > 0) {
      j_traj[joint][sampled_t[joint][5]] = j_traj[joint][sampled_t[joint][5]] + (1 - sampled_t_trans[5]/t_sample_) * jerk_profile[6];
    }
    // Add fraction to end of trajectory (after phase 7)
    j_traj[joint][sampled_t[joint][6] + 1] = j_traj[joint][sampled_t[joint][6] + 1] + sampled_t_trans[6]/t_sample_ * jerk_profile[6];

    //// Calculate joint trajectories
    a_traj[joint][0] = a_0[joint] + t_sample_ * j_traj[joint][0];
    v_traj[joint][0] = v_0[joint] + t_sample_ * a_traj[joint][0];
    q_traj[joint][0] = q_0[joint] + t_sample_ * v_traj[joint][0];
    bool phase4 = sampled_t[joint][3] - sampled_t[joint][2] > 2;
    for (int i = 1; i < traj_len; i++) {
      if (i <= sampled_t[joint][6]) {
        a_traj[joint][i] = a_traj[joint][i-1] + t_sample_ * j_traj[joint][i];
      } else {
        // Set final accelation to exactly 0 (increases accuracy)
        a_traj[joint][i] = 0.0;
      }
      // Set constant velocity periods to exactly v_drive (increases accuracy)
      if (phase4 && i >= sampled_t[joint][2]+1 && i< sampled_t[joint][3]-1) {
        v_traj[joint][i] = v_drive[joint] * dir[joint];
      } else if (i <= sampled_t[joint][6]) {
        v_traj[joint][i] = v_traj[joint][i-1] + t_sample_ * a_traj[joint][i];
      } else {
        // Set final velocity to exactly 0 (increases accuracy)
        v_traj[joint][i] = 0.0;
      }
      q_traj[joint][i] = q_traj[joint][i-1] + t_sample_ * v_traj[joint][i];
    }
  }
  traj.dof = dof_;
  traj.length = traj_len;
  traj.t_sample = t_sample_;
  traj.q = q_traj;
  traj.v = v_traj;
  traj.a = a_traj;
  traj.j = j_traj;
  return traj;
}
} // namespace long_term_planner