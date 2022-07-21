#include <gtest/gtest.h>

#include "long_term_planner_fixture.h"
#include "long_term_planner/long_term_planner.h"

namespace long_term_planner {

TEST_F(LongTermPlannerTest1DoF, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

TEST_F(LongTermPlannerTest1DoF, OptBrakingTest) {
  // Define test values
  double eps = 0.01;
  int num_scenarios = 6;
  std::vector<double> v_0 = {0, -1.875, -1.875, -0.875, -0.875, 0.5};
  std::vector<double> a_0 = {0, 1, -1, 1, -1, -2};
  std::vector<std::vector<double>> q_min = {{-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}}; 
  std::vector<std::vector<double>> q_max = {{3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}}; 
  std::vector<std::vector<double>> v_max  = {{10}, {10}, {10}, {10}, {10}, {10}}; 
    std::vector<std::vector<double>> a_max = {{2}, {2}, {2}, {4}, {4}, {4}};
  std::vector<std::vector<double>> j_max = {{4}, {4}, {4}, {4}, {4}, {2}};

  // Define pre-calculated results
  std::vector<double> q_goal = {0, -1.0104, -1.9896, -0.2604, -0.7396, -0.4167};
  std::vector<std::vector<double>> t_rel  = {{0, 0, 0}, {0.25, 0.5, 0.5}, {0.75, 0.5, 0.5}, {0.25, 0, 0.5}, {0.75, 0, 0.5}, {1.5, 0, 0.5}};
  // Test all scenarios
  for (int i=0; i<num_scenarios; i++) {
    ltp_.setLimits(q_min[i], q_max[i], v_max[i], a_max[i], j_max[i]);
    // Compare times to pre-calculation
    double q_ltp;
    std::array<double, 7> t_ltp;
    double dir;
    ltp_.optBraking(0, v_0[i], a_0[i], q_ltp, t_ltp, dir);
    // all(abs(t_ltp - t_rel(i,:)) < eps) && (abs(q_ltp - q_goal(i)) < eps)
    bool success = true;
    for (int j=0; j<3; j++) {
      std::cerr << "Positive test t_rel[" << i << "][" << j << "] = " << t_rel[i][j] << std::endl;
      EXPECT_NEAR(t_ltp[j], t_rel[i][j], eps);
    }
    std::cerr << "dir = " << dir << std::endl;
    EXPECT_NEAR(q_ltp, q_goal[i], eps);
    // Skip for first test
    if (i == 0) continue;

    // Execute same scenario in opposite direction
    ltp_.optBraking(0, -v_0[i], -a_0[i], q_ltp, t_ltp, dir);
    for (int j=0; j<3; j++) {
      std::cerr << "Negative test t_rel[" << i << "][" << j << "] = " << t_rel[i][j] << std::endl;
      EXPECT_NEAR(t_ltp[j], t_rel[i][j], eps);
    }
    std::cerr << "dir = " << dir << std::endl;
    EXPECT_NEAR(q_ltp, -q_goal[i], eps);
  }
}

TEST_F(LongTermPlannerTest6DoF, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

} // namespace long_term_planner

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}