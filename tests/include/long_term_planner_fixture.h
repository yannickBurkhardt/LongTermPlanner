// -*- lsst-c++ -*/
/**
 * @file long_term_planner_fixture.h
 * @brief Defines the test fixture for the long term planner object
 * @version 0.1
 * @copyright This file is part of LongTermPlanner.
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
#include <functional>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "long_term_planner/long_term_planner.h"

#ifndef LONG_TERM_PLANNER_FIXTURE_H
#define LONG_TERM_PLANNER_FIXTURE_H

namespace long_term_planner {

class LongTermPlannerExposed : public LongTermPlanner {
 public:
  using LongTermPlanner::optSwitchTimes;
  using LongTermPlanner::timeScaling;
  using LongTermPlanner::optBraking;
  using LongTermPlanner::getTrajectory;

  LongTermPlannerExposed() {}

  explicit LongTermPlannerExposed(int dof,
    double t_sample,
    std::vector<double> q_min,
    std::vector<double> q_max,
    std::vector<double> v_max,
    std::vector<double> a_max,
    std::vector<double> j_max) : LongTermPlanner(
        dof,
        t_sample,
        q_min,
        q_max,
        v_max,
        a_max,
        j_max) {}
};

/**
 * @brief Test fixture for long term planner class
 */
class LongTermPlannerTest1DoF : public ::testing::Test {
 protected:
  /**
   * @brief The long term planner object
   */
  LongTermPlannerExposed ltp_;

  /**
   * @brief Create the safety shield object
   */
  void SetUp() override {
    int dof = 1;
    double t_sample = 0.001;
    std::vector<double> q_min = {-3.1};
    std::vector<double> q_max = {3.1};
    std::vector<double> v_max = {10};
    std::vector<double> a_max = {2};
    std::vector<double> j_max = {4};
    ltp_ = LongTermPlannerExposed(dof, t_sample, q_min, q_max, v_max, a_max, j_max);
  }
};

/**
 * @brief Test fixture for long term planner class
 */
class LongTermPlannerTest6DoF : public ::testing::Test {
 protected:
  /**
   * @brief The long term planner object
   */
  LongTermPlannerExposed ltp_;

  /**
   * @brief Create the safety shield object
   */
  void SetUp() override {
    int dof = 6;
    double t_sample = 0.001;
    std::vector<double> q_min(dof);
    std::fill(q_min.begin(), q_min.end(), -3.1);
    std::vector<double> q_max(dof);
    std::fill(q_max.begin(), q_max.end(), 3.1);
    std::vector<double> v_max(dof);
    std::fill(v_max.begin(), v_max.end(), 10.0);
    std::vector<double> a_max = {2, 2, 2, 4, 4, 4};
    std::vector<double> j_max = {4, 4, 4, 4, 4, 2};
    ltp_ = LongTermPlannerExposed(dof, t_sample, q_min, q_max, v_max, a_max, j_max);
  }
};
} // namespace long_term_planner

#endif // LONG_TERM_PLANNER_FIXTURE_H    