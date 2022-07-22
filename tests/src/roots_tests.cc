#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "long_term_planner/roots.h"

namespace long_term_planner {

TEST(RootsTests, SimpleTest4) {
  float a_4 = 1.0;
  float a_3 = 0.0;
  float a_2 = 0.0;
  float a_1 = 0.0;
  float a_0 = -625.0;
  float r = fourth_2deriv(a_4, a_3, a_2, a_1, a_0);
  EXPECT_NEAR(r, 5, 1e-6);
};

TEST(RootsTests, ComplicatedTest4) {
  float a_4 = 5.0;
  float a_3 = -3.0;
  float a_2 = 1000.0;
  float a_1 = 2.5;
  float a_0 = -80.0;
  float r = fourth_2deriv(a_4, a_3, a_2, a_1, a_0);
  EXPECT_NEAR(r, 0.281658, 1e-5);
};

TEST(RootsTests, SimpleTest5) {
  float a_5 = 1.0;
  float a_4 = 0.0;
  float a_3 = 0.0;
  float a_2 = 0.0;
  float a_1 = 0.0;
  float a_0 = -3125.0;
  float r = fifth_2deriv(a_5, a_4, a_3, a_2, a_1, a_0);
  EXPECT_NEAR(r, 5, 1e-6);
};

TEST(RootsTests, ComplexRootsTest6) {
  float a_6 = 144;
  float a_5 = -1008;
  float a_4 =	2448;
  float a_3 =	3024.01920000000;
  float a_2 =	-15768.1344000000;
  float a_1 = 0;
  float a_0 =	22752.4032012800;
  Eigen::VectorXf poly_vals(7); 
  poly_vals << a_6, a_5, a_4, a_3, a_2, a_1, a_0;
  Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic> r = roots<float>(poly_vals);
  EXPECT_NEAR(r(0,0).real(), -1.67276, 1e-5);
  EXPECT_NEAR(r(0,0).imag(), 0.0, 1e-9);
  EXPECT_NEAR(r(1,0).real(), -1.35687, 1e-5);
  EXPECT_NEAR(r(1,0).imag(), 0.0, 1e-9);
  EXPECT_NEAR(r(2,0).real(), 2.00001, 1e-5);
  EXPECT_NEAR(r(2,0).imag(), 0.0, 1e-9);
  EXPECT_NEAR(r(3,0).real(), 2.09261, 1e-5);
  EXPECT_NEAR(r(3,0).imag(), 0.0, 1e-9);
  EXPECT_NEAR(r(4,0).real(), 2.9685, 1e-5);
  EXPECT_NEAR(r(4,0).imag(), 2.79663, 1e-5);
  EXPECT_NEAR(r(5,0).real(), 2.9685, 1e-5);
  EXPECT_NEAR(r(5,0).imag(), -2.79663, 1e-5);
};
} // namespace long_term_planner

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "verify iso tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}