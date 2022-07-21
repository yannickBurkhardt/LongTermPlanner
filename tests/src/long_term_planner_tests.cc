#include <gtest/gtest.h>

#include "long_term_planner_fixture.h"
#include "long_term_planner/long_term_planner.h"

namespace long_term_planner {

TEST_F(LongTermPlannerTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

} // namespace long_term_planner

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}