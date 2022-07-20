// -*- lsst-c++ -*/
/**
 * @file long_term_planner.h
 * @brief Defines the LongTermPlanner class
 * @version 0.1
 * @copyright This file is part of the LongTermPlanner.
 * LongTermPlanner is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 * LongTermPlanner is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with LongTermPlanner. 
 * If not, see <https://www.gnu.org/licenses/>. 
 */

#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <math.h>
#include <cmath>

#ifndef long_term_planner_H
#define long_term_planner_H

namespace long_term_planner {

/**
 * @brief Trajectory structure.
 */
struct Trajectory {        
  int dof;
  double t_sample;
  int length;
  std::vector<std::vector<double>> q;
  std::vector<std::vector<double>> v;
  std::vector<std::vector<double>> a;
  std::vector<std::vector<double>> j;
};

/**
 * @brief Sign function
 * 
 * @tparam T 
 * @param val value
 * @return -1 if negaitve, 0 if 0, and 1 if positive 
 */
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief Plans a trajectory for multiple joints.
 */
class LongTermPlanner {
 private:

  /**
   * @brief Degrees of freedom, i.e., number of joints.
   */
  int dof_;

  /**
   * @brief Time between two samples of the planned trajectories.
   */
  double t_sample_;

  /**
   * @brief Output length in samples.
   */
  int num_samples_;

  /**
   * @brief Minimum joint positions.
   */
  std::vector<double> q_min_;

  /**
   * @brief Maximum joint positions.
   */
  std::vector<double> q_max_;

  /**
   * @brief Maximum allowed velocities per DoF.
   */
  std::vector<double> v_max_;

  /**
   * @brief Maximum allowed accelerations per DoF.
   */
  std::vector<double> a_max_;

  /**
   * @brief Maximum allowed jerk per DoF.
   */
  std::vector<double> j_max_;

 public:
  /**
   * @brief Construct a new dummy Long Term Planner object.
   */
  LongTermPlanner() :
    dof_(0),
    t_sample_(0.001),
    num_samples_(1) {};

  /**
   * @brief Construct a new Long Term Planner object
   * 
   * @param[in] dof Degrees of freedom, i.e., number of joints.
   * @param[in] t_sample Time between two samples of the planned trajectories.
   * @param[in] num_samples Output length in samples.
   * @param[in] q_min Minimum joint positions.
   * @param[in] q_max Maximum joint positions.
   * @param[in] v_max Maximum allowed velocities per DoF.
   * @param[in] a_max Maximum allowed accelerations per DoF.
   * @param[in] j_max Maximum allowed jerk per DoF.
   */
  LongTermPlanner(int dof,
    double t_sample,
    int num_samples,
    std::vector<double> q_min,
    std::vector<double> q_max,
    std::vector<double> v_max,
    std::vector<double> a_max,
    std::vector<double> j_max) {};

  /**
   * @brief Plan a trajectory from the given start state to the goal position and zero velocity/acc.
   * 
   * @param[in] q_goal Goal positions.
   * @param[in] q_0 Start positions.
   * @param[in] v_0 Start velocities.
   * @param[in] a_0 Start accelerations.
   * @param[out] traj The trajectory structure to return.
   * @return true if successful.
   * @return false if planning not possible.
   */
   bool planTrajectory(
    const std::vector<double>& q_goal,
    const std::vector<double>& q_0,
    const std::vector<double>& v_0,
    const std::vector<double>& a_0,
    Trajectory& traj
  );

  /**
   * @brief Check if the input values are valid.
   * 
   * @param[in] q_0 Start positions. 
   * @param[in] v_0 Start velocities.
   * @param[in] a_0 Start accelerations.
   * @return true if inputs okay
   * @return false if inputs not in bounds.
   */
  bool checkInputs(
    const std::vector<double>& q_0,
    const std::vector<double>& v_0,
    const std::vector<double>& a_0
  );

 protected:
  /**
   * @brief Calculate time-optimal jerk swtiches.
   * 
   * @param[in] joint Joint id to access joint limit vectors.
   * @param[in] q_goal Goal position of the joint.
   * @param[in] q_0 Start position of the joint.
   * @param[in] v_0 Start velocity of the joint.
   * @param[in] a_0 Start acceleration of the joint.
   * @param[in] v_drive Constant velocity in phase 4.
   * @param[out] t Time points of jerk switches.
   * @param[out] dir Direction of the goal.
   * @param[out] mod_jerk_profile true if slowing down is necessary to satisfy v_drive, false otherwise.
   * @return true if successful.
   * @return false if planning not possible.
   */
  bool optSwitchTimes(int joint, 
    double q_goal, 
    double q_0, 
    double v_0, 
    double a_0,
    double v_drive,
    std::array<double, 7>& t,
    double& dir,
    char& mod_jerk_profile);

  /**
   * @brief Calculate switching times to fulfil a given time by adjusting the maximally reached velocity.
   * 
   * @param[in] joint Joint id to access joint limit vectors.
   * @param[in] q_goal Goal position of the joint.
   * @param[in] q_0 Start position of the joint.
   * @param[in] v_0 Start velocity of the joint.
   * @param[in] a_0 Start acceleration of the joint.
   * @param[in] dir Direction of the goal.
   * @param[in] t_required Required end time.
   * @param[out] scaled_t Scaled time points of jerk switches.
   * @param[out] v_drive Driving velocity.
   * @param[out] mod_jerk_profile true if slowing down is necessary to satisfy v_drive, false otherwise.
   * @return true if successful. 
   * @return false if planning not possible.
   */
  bool timeScaling(
    int joint, 
    double q_goal, 
    double q_0, 
    double v_0, 
    double a_0, 
    double dir,
    double t_required,
    std::array<double, 7>& scaled_t,
    double& v_drive,
    char& mod_jerk_profile);

  /**
   * @brief Calculate time and joint angles required to bring velocity to zero.
   * @details This function can be used to:
   *           - bring a joint to a full stop as fast as possible
   *           - calculate in which direction a joint has to be actuated to
   *             reach a goal
   *           - slow a joint down to a desired velocity (v_0 must be set to
   *             v_current - v_desired)
   *
   * @param[in] joint Joint id to access joint limit vectors.
   * @param[in] v_0 Start velocity of the joint.
   * @param[in] a_0 Start acceleration of the joint.
   * @param[out] q Position after braking.
   * @param[out] t_rel Time points of jerk switches
   * @param[out] dir Direction of the goal.
   * @return true if successful. 
   * @return false if planning not possible.
   */
  bool optBraking(
    int joint, 
    double v_0, 
    double a_0, 
    double& q,
    std::array<double, 7>& t_rel,
    double& dir);

  /**
   * @brief Calculate a trajectory from the given jerk profile.
   * 
   * @param[in] t Time points of jerk switches.
   * @param[in] dir Direction of the goal.
   * @param[in] mod_jerk_profile true if slowing down is necessary to satisfy v_drive, false otherwise.
   * @param[in] q_0 Start positions. 
   * @param[in] v_0 Start velocities.
   * @param[in] a_0 Start accelerations.
   * @return Trajectory 
   */
  Trajectory getTrajectory(
    const std::vector<std::array<double, 7>>& t,
    const std::vector<double>& dir,
    const std::vector<char>& mod_jerk_profile,
    const std::vector<double>& q_0,
    const std::vector<double>& v_0,
    const std::vector<double>& a_0
  );
};
} // namespace long_term_planner

#endif // long_term_planner_H