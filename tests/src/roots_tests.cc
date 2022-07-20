#include <gtest/gtest.h>

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
}

TEST(RootsTests, ComplicatedTest4) {
  float a_4 = 5.0;
  float a_3 = -3.0;
  float a_2 = 1000.0;
  float a_1 = 2.5;
  float a_0 = -80.0;
  float r = fourth_2deriv(a_4, a_3, a_2, a_1, a_0);
  EXPECT_NEAR(r, 0.281658, 1e-5);
}

TEST(RootsTests, SimpleTest5) {
  float a_5 = 1.0;
  float a_4 = 0.0;
  float a_3 = 0.0;
  float a_2 = 0.0;
  float a_1 = 0.0;
  float a_0 = -3125.0;
  float r = fifth_2deriv(a_5, a_4, a_3, a_2, a_1, a_0);
  EXPECT_NEAR(r, 5, 1e-6);
}
} // namespace long_term_planner

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "verify iso tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}