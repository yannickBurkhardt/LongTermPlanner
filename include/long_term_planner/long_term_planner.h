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


};
} // namespace long_term_planner

#endif // long_term_planner_H