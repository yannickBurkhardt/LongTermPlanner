cmake_minimum_required(VERSION 3.0.2)
project(long_term_planner)

## Compile as C++17
add_compile_options(-std=c++17)

find_package(Eigen3 3.4 REQUIRED)

INCLUDE_DIRECTORIES ( ${EIGEN3_INCLUDE_DIR})

include_directories(
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/long_term_planner.cc 
)

# Install the library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the header files
install(DIRECTORY include/
    DESTINATION include
)
# Generate and install the export set for use with the install-tree
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

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