cmake_minimum_required(VERSION 3.0.2)
project(long_term_planner)

## Compile as C++17
add_compile_options(-std=c++17)

INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )

include_directories(
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/long_term_planner.cc 
    )

# target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME} DESTINATION /usr/lib)

#############
## GTest ##
#############
enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(roots_tests tests/src/roots_tests.cc)
target_link_libraries(roots_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
gtest_discover_tests(roots_tests)

add_executable(long_term_planner_tests tests/src/long_term_planner_tests.cc)
target_include_directories(long_term_planner_tests PUBLIC tests/include)
target_link_libraries(long_term_planner_tests ${PROJECT_NAME} GTest::GTest GTest::Main)
gtest_discover_tests(long_term_planner_tests)