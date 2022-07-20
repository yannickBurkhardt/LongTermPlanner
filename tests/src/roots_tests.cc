#include <gtest/gtest.h>

#include "long_term_planner/roots.h"

namespace long_term_planner {
TEST(RootsTests, SimpleTest) {
  EXPECT_EQ(0, 0);
} 
} // namespace long_term_planner

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "verify iso tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}