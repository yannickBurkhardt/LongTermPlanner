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
   * @param[in] q_goal Goal position.
   * @param[in] q_0 Start position.
   * @param[in] v_0 Start velocity
   * @param[in] a_0 Start acceleration.
   * @param[out] traj The trajectory structure to return.
   * @return true if successful.
   * @return false if planning not possible.
   */
   bool planTrajectory(
    std::vector<double> q_goal,
    std::vector<double> q_0,
    std::vector<double> v_0,
    std::vector<double> a_0,
    Trajectory& traj
  );

 protected:
  /**
   * @brief Calculate time-optimal jerk swtiches.
   * 
   * @param[in] joint Joint id to access joint limit vectors.
   * @param[in] q_goal goal position of the joint.
   * @param[in] q_0 start position of the joint.
   * @param[in] v_0 start velocity of the joint.
   * @param[in] a_0 start acceleration of the joint.
   * @param[out] t time points of jerk switches.
   * @param[out] dir direction of the goal.
   * @param[out] mod_jerk_profile true if slowing down is necessary to satisfy v_drive, false otherwise.
   * @return true if successful.
   * @return false if planning not possible.
   */
  bool optSwitchTimes(int joint, 
    double q_goal, 
    double q_0, 
    double v_0, 
    double a_0, 
    std::vector<double>& t, 
    double& dir,
    bool& mod_jerk_profile);
};
} // namespace long_term_planner

#endif // long_term_planner_H