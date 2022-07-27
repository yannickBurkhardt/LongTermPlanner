#include <gtest/gtest.h>

#include "long_term_planner_fixture.h"
#include "long_term_planner/long_term_planner.h"

namespace long_term_planner {

TEST_F(LongTermPlannerTest1DoF, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
};

TEST_F(LongTermPlannerTest1DoF, OptBrakingTest) {
  // Define test values
  double eps = 0.01;
  int num_scenarios = 6;
  std::vector<double> v_0 = {0, -1.875, -1.875, -0.875, -0.875, 0.5};
  std::vector<double> a_0 = {0, 1, -1, 1, -1, -2};
  std::vector<std::vector<double>> q_min = {{-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}}; 
  std::vector<std::vector<double>> q_max = {{3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}}; 
  std::vector<std::vector<double>> v_max = {{10}, {10}, {10}, {10}, {10}, {10}}; 
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
    EXPECT_TRUE(ltp_.optBraking(0, v_0[i], a_0[i], q_ltp, t_ltp, dir));
    // all(abs(t_ltp - t_rel(i,:)) < eps) && (abs(q_ltp - q_goal(i)) < eps)
    for (int j=0; j<3; j++) {
      EXPECT_NEAR(t_ltp[j], t_rel[i][j], eps);
    }
    EXPECT_NEAR(q_ltp, q_goal[i], eps);
    // Skip for first test
    if (i == 0) continue;

    // Execute same scenario in opposite direction
    EXPECT_TRUE(ltp_.optBraking(0, -v_0[i], -a_0[i], q_ltp, t_ltp, dir));
    for (int j=0; j<3; j++) {
      EXPECT_NEAR(t_ltp[j], t_rel[i][j], eps);
    }
    EXPECT_NEAR(q_ltp, -q_goal[i], eps);
  }
};

TEST_F(LongTermPlannerTest1DoF, OptSwitchTimesTest) {
  // Define test values
  double eps = 0.001;
  int num_scenarios = 9;
  std::vector<std::vector<double>> q_min = {{-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}}; 
  std::vector<std::vector<double>> q_max = {{3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}}; 
  std::vector<std::vector<double>> v_max = {{2}, {2}, {2}, {1}, {1}, {8}, {8}, {8}, {8}};
  std::vector<std::vector<double>> a_max = {{2}, {2}, {2}, {2}, {2}, {2}, {2}, {2}, {2}};
  std::vector<std::vector<double>> j_max = {{4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}};
  std::vector<double> q_goal = {-1.0, 2.927, 2.8854, 0.2396, 0.6354, 1.927, 1.8854, -0.2604, 0.1354};
  std::vector<double> q_0    = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  std::vector<double> v_0    = { 0.0, 0.625, 1.875, -0.875, 0.875, 0.625, 1.875, -0.875, 0.875};
  std::vector<double> a_0    = { 0.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

  // Define pre-calculated results
  std::vector<std::vector<double>> t_rel  = 
    {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.250000000000000,	0.250000000000000,	0.500000000000000,	0.500000000000000,	0.500000000000000,	0.500000000000000,	0.500000000000000},
    {0.500000000000000, 0.0, 0.250000000000000,	0.500000000000000,	0.500000000000000,	0.500000000000000,	0.500000000000000},
    {0.250000000000000,	0.500000000000000,	0.500000000000000,	0.500000000000000,	0.500000000000000, 0.0, 0.500000000000000},
    {0.500000000000000, 0.0, 0.250000000000000,	0.500000000000000,	0.500000000000000, 0.0, 0.500000000000000},
    {0.250000000000000,	0.250000000000000,	0.500000000000000, 0.0, 0.500000000000000,	0.500000000000000,	0.500000000000000},
    {0.500000000000000, 0.0, 0.0, 0.0, 0.750000000000000,	0.500000000000000,	0.500000000000000},
    {0.250000000000000,	0.500000000000000,	0.500000000000000, 0.0, 0.500000000000000, 0.0, 0.500000000000000},
    {0.500000000000000, 0.0, 0.0, 0.0, 0.750000000000000, 0.0, 0.500000000000000}};
  std::vector<std::vector<double>> t = 
    {{0.0,	0.0, 0.0,	0.0, 0.0,	0.0, 0.0},
    {0.250000000000000,	0.500000000000000, 1.0,	1.50000000000000, 2.0,	2.50000000000000, 3.0},
    {0.500000000000000,	0.500000000000000, 0.750000000000000,	1.25000000000000, 1.75000000000000,	2.25000000000000, 2.75000000000000},
    {0.250000000000000,	0.750000000000000, 1.25000000000000,	1.75000000000000, 2.25000000000000,	2.25000000000000, 2.75000000000000},
    {0.500000000000000,	0.500000000000000, 0.750000000000000,	1.25000000000000, 1.75000000000000,	1.75000000000000, 2.25000000000000},
    {0.250000000000000,	0.500000000000000, 1.0,	1.0, 1.50000000000000,	2.0, 2.50000000000000},
    {0.500000000000000,	0.500000000000000, 0.500000000000000,	0.500000000000000, 1.25000000000000,	1.75000000000000, 2.25000000000000},
    {0.250000000000000,	0.750000000000000, 1.25000000000000,	1.25000000000000, 1.75000000000000,	1.75000000000000, 2.25000000000000},
    {0.500000000000000,	0.500000000000000, 0.500000000000000,	0.500000000000000, 1.25000000000000,	1.25000000000000, 1.75000000000000}};

  for (int i=0; i<num_scenarios; i++) {
    // Set limtits
    ltp_.setLimits(q_min[i], q_max[i], v_max[i], a_max[i], j_max[i]);
    
    // Compare times to pre-calculation
    std::array<double, 7> t_ltp;
    double dir;
    char mod_jerk_profile;
    EXPECT_TRUE(ltp_.optSwitchTimes(0, q_goal[i], q_0[i], v_0[i], a_0[i], v_max[i][0], t_ltp, dir, mod_jerk_profile));
    for (int j=0; j<3; j++) {
      EXPECT_NEAR(t_ltp[j], t[i][j], eps);
    }
    // Skip for first test
    if (i == 0) continue;

    // Execute same scenario in opposite direction
    EXPECT_TRUE(ltp_.optSwitchTimes(0, -q_goal[i], -q_0[i], -v_0[i], -a_0[i], v_max[i][0], t_ltp, dir, mod_jerk_profile));
    for (int j=0; j<3; j++) {
      EXPECT_NEAR(t_ltp[j], t[i][j], eps);
    }
  }
};

TEST_F(LongTermPlannerTest1DoF, TimeScalingTest) {
  // Define test values
  double eps = 0.1;
  int num_scenarios = 12;
  std::vector<std::vector<double>> q_min = {{-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}, {-3.1}}; 
  std::vector<std::vector<double>> q_max = {{3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}, {3.1}}; 
  // v_max increased compared to testOptSwitchTimes
  std::vector<std::vector<double>> v_max  = {{4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}};
  std::vector<std::vector<double>> a_max  = {{2}, {2}, {2}, {2}, {2}, {2}, {2}, {2}, {2}, {4}, {4}, {4}};
  std::vector<std::vector<double>> j_max  = {{4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {4}, {2}};
  std::vector<double> q_goal = {-1.0, 2.927, 2.8854, 0.2396, 0.6354, -7.0104, -8.9896, -3.896, -7.9433, -5.1746, -6.6538, -8.4167};
  std::vector<double> q_0 = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  std::vector<double> v_0    = {0.0, 0.625, 1.875, -0.875, 0.875, -3.875, -3.875, -1.875, -1.875, -2.875, -2.875, -1.5};
  std::vector<double> a_0    = {0.0, 1, -1, 1, -1, 1, -1, 1, -2, 1, -1, -2};
  std::vector<double> dir    = {1.0, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1};
  std::vector<std::vector<double>> t_rel =
    {{0,0,0,0,0,0,0},
    {0.25,0.25,0.5,0.5,0.5,0.5,0.5},
    {0.5,0,0.25,0.5,0.5,0.5,0.5},
    {0.25,0.5,0.5,0.5,0.5,0,0.5},
    {0.5,0,0.25,0.5,0.5,0,0.5},
    {0.25,0.5,0.5,0.5,0.5,0.5,0.5},
    {0.75,0.5,0.5,0.5,0.5,0.5,0.5},
    {0.25,0.25,0.5,3.2928,0.3536,0,0.3536},
    {0.75,0,0.25,1.4372,0.5,0.5625,0.5},
    {0.25,0,0.5,0.5,0.7071,0,0.7071},
    {0.75,0,0.5,0.5,0.7071,0,0.7071},
    {1.5,0,0.5,0.5,1,0,1}};
  std::vector<std::vector<double>> t =
    {{0,0,0,0,0,0,0},
    {0.25,0.5,1,1.5,2,2.5,3},
    {0.5,0.5,0.75,1.25,1.75,2.25,2.75},
    {0.25,0.75,1.25,1.75,2.25,2.25,2.75},
    {0.5,0.5,0.75,1.25,1.75,1.75,2.25},
    {0.25,0.75,1.25,1.75,2.25,2.75,3.25},
    {0.75,1.25,1.75,2.25,2.75,3.25,3.75},
    {0.25,0.5,1,4.2928,4.6464,4.6464,5},
    {0.75,0.75,1,2.4372,2.9372,3.4997,3.9997},
    {0.25,0.25,0.75,1.25,1.9571,1.9571,2.6642},
    {0.75,0.75,1.25,1.75,2.4571,2.4571,3.1642},
    {1.5,1.5,2,2.5,3.5,3.5,4.5}};
  std::vector<double> t_required = {0, 3, 2.75000000000000, 2.75000000000000, 2.25000000000000, 3.25000000000000, 3.75000000000000, 5, 3.99970000000000, 2.66420000000000, 3.16420000000000, 4.50000000000000};

  for (int i=0; i<num_scenarios; i++) {
    // Set limtits
    ltp_.setLimits(q_min[i], q_max[i], v_max[i], a_max[i], j_max[i]);
    
    // Compare times to pre-calculation
    std::array<double, 7> t_ltp;
    double v_drive;
    char mod_jerk_profile;
    EXPECT_TRUE(ltp_.timeScaling(0, q_goal[i], q_0[i], v_0[i], a_0[i], dir[i], t_required[i], t_ltp, v_drive, mod_jerk_profile));
    for (int j=0; j<3; j++) {
      EXPECT_NEAR(t_ltp[j], t[i][j], eps);
    }
    // Skip for first test
    if (i == 0) continue;

    // Execute same scenario in opposite direction
    EXPECT_TRUE(ltp_.timeScaling(0, -q_goal[i], -q_0[i], -v_0[i], -a_0[i], -dir[i], t_required[i], t_ltp, v_drive, mod_jerk_profile));
    for (int j=0; j<3; j++) {
      EXPECT_NEAR(t_ltp[j], t[i][j], eps);
    }
  }
};

TEST_F(LongTermPlannerTest1DoF, gridTestOneJoint) {
  // Initialize parameters
  double eps = 1e-6;
  double tol = 0.02;
  double step = 0.1;
  std::vector<double> q_min = {-3.1};
  std::vector<double> q_max = {3.1};
  std::vector<double> v_max = {1.0};
  std::vector<double> a_max = {2.0};
  std::vector<double> j_max = {15.0};
  double t_sample = 0.004;
  double q_0 = 0.5;

  // Initialize planner
  ltp_.setSampleTime(t_sample);
  ltp_.setLimits(q_min, q_max, v_max, a_max, j_max);

  // Set goal and joint angle
  for (int i=(int)q_min[0]/step; i<=(int)q_max[0]/step; i++) {
    double q_goal = i * step;
    // Velocity in Limits
    for (int j = (int)-v_max[0]/step; j<(int)v_max[0]/step; j++) {
      double v_0 = j * step;
      // Calculate maximal acceleration to not violate velocity limit
      double a_lb, a_ub;
      if (v_0 >= 0) {
        a_lb = -(a_max[0]-eps);
        a_ub = std::min(a_max[0]-eps, sqrt(2*j_max[0]*(v_max[0] - v_0)));
      } else {
        a_lb = std::max(-(a_max[0]-eps), -sqrt(2*j_max[0]*(v_max[0] - abs(v_0))));
        a_ub = a_max[0];
      }
      // Acceleration in limits
      for (int k=(int)a_lb/step; k<(int)a_ub/step; k++) {
        double a_0 = k*step - eps;
        // Plan trajectory
        std::array<double, 7> t_ltp;
        double dir;
        char mod_jerk_profile;
        EXPECT_TRUE(ltp_.optSwitchTimes(0, q_goal, q_0, v_0, a_0, v_max[0], t_ltp, dir, mod_jerk_profile));
        std::vector<std::array<double, 7>> t = {t_ltp};
        std::vector<double> dirs = {dir};
        std::vector<char> mod_jerk_profiles = {mod_jerk_profile};
        std::vector<double> q0s = {q_0};
        std::vector<double> v0s = {v_0};
        std::vector<double> a0s = {a_0};
        Trajectory traj = ltp_.getTrajectory(t, dirs, mod_jerk_profiles, q0s, v0s, a0s, v_max);
        // Check if goal was reached
        EXPECT_NEAR(traj.q[0][traj.length-1], q_goal, tol);
        if (abs(traj.q[0][traj.length-1] - q_goal) > tol) {
          std::cerr << "q_goal = " << q_goal << 
                       ", v_0 = " << v_0 << 
                       ", a_0 = " << a_0 <<
                       ", dir = " << dir <<
                       ", t_ltp = [" << t_ltp[0] << ", " << t_ltp[1] << ", " << t_ltp[2] << ", " << t_ltp[3] << ", " << t_ltp[4] << ", " << t_ltp[5] << ", " << t_ltp[6] << "]." << std::endl;
        }
      }
    }
  }
}

TEST_F(LongTermPlannerTest1DoF, GridTimeScalingTest) {
  // Initialize parameters
  double eps = 1e-6;
  double tol_q = 0.02;
  double tol_t = 0.1;
  double step = 0.1;
  std::vector<double> q_min = {-6};
  std::vector<double> q_max = {7};
  std::vector<double> v_max = {1.0};
  std::vector<double> a_max = {2.0};
  std::vector<double> j_max = {15.0};
  double t_sample = 0.004;
  double q_0 = 0.5;
  std::vector<double> time_increments = {0.05, 0.1, 0.2, 0.5, 1.0, 2.0};

  // Initialize planner
  ltp_.setSampleTime(t_sample);
  ltp_.setLimits(q_min, q_max, v_max, a_max, j_max);

  // Set goal and joint angle
  for (int i=(int)q_min[0]/step; i<=(int)q_max[0]/step; i++) {
    double q_goal = i * step;
    // Velocity in Limits
    for (int j = (int)-v_max[0]/step; j<(int)v_max[0]/step; j++) {
      double v_0 = j * step;
      v_0 = (j>0) ? v_0 - eps : v_0 + eps; 
      double a_lb, a_ub;
      if (v_0 >= 0) {
        a_lb = -(a_max[0]-eps);
        a_ub = std::min(a_max[0]-eps, sqrt(2*j_max[0]*(v_max[0] - v_0)));
      } else {
        a_lb = std::max(-(a_max[0]-eps), -sqrt(2*j_max[0]*(v_max[0] - abs(v_0))));
        a_ub = a_max[0];
      }
      // Acceleration in limits
      double a_range = a_ub-a_lb;
      int n_steps = (int) floor(a_range/step);
      for (int k=0; k<n_steps; k++) {
        double a_0 = a_lb + k*step;
        std::array<double, 7> t_ltp;
        double dir;
        char mod_jerk_profile;
        bool successOptSwitchTimes = ltp_.optSwitchTimes(0, q_goal, q_0, v_0, a_0, v_max[0], t_ltp, dir, mod_jerk_profile);
        EXPECT_TRUE(successOptSwitchTimes);
        if (!successOptSwitchTimes) continue;      
        // Scale up trajectory
        for (int l=0; l<time_increments.size(); l++) {
          // Skip if already at goal position
          if (t_ltp[t_ltp.size()-1] < tol_q) {
            break;
          }
          
          // Calculate scaled times
          std::array<double, 7> t_scaled;
          double v_drive;
          char mod_jerk_profile;
          if (i == 1 && j == -9 && k == 2 && l == 2) {
            mod_jerk_profile = false;
          }
          bool successTimeScaling = ltp_.timeScaling(0, q_goal, q_0, v_0, a_0, dir, t_ltp[t_ltp.size()-1] + time_increments[l], t_scaled, v_drive, mod_jerk_profile);
          if (!successTimeScaling) {
            t_scaled = t_ltp;
          }
          // Calculate trajectories
          std::vector<std::array<double, 7>> ts = {t_scaled};
          std::vector<double> dirs = {dir};
          std::vector<double> v_drives = {v_drive};
          std::vector<char> mod_jerk_profiles = {mod_jerk_profile};
          std::vector<double> q0s = {q_0};
          std::vector<double> v0s = {v_0};
          std::vector<double> a0s = {a_0};
          Trajectory traj = ltp_.getTrajectory(ts, dirs, mod_jerk_profiles, q0s, v0s, a0s, v_drives);
          EXPECT_NEAR(traj.q[0][traj.length-1], q_goal, tol_q);
          if (abs(t_ltp[6] + time_increments[l] - t_scaled[6]) > tol_t) {
            EXPECT_NEAR(traj.v[0][traj.length-1], 0.0, tol_q);
            EXPECT_NEAR(traj.a[0][traj.length-1], 0.0, tol_q);
            EXPECT_NEAR(t_scaled[2], t_scaled[6], eps);
          }
        }
      }
    }  
  }
}

TEST_F(LongTermPlannerTest6DoF, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
};

} // namespace long_term_planner

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
};