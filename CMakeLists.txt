cmake_minimum_required(VERSION 3.0.2)
project(long_term_planner)

## Compile as C++17
add_compile_options(-std=c++17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/long_term_planner.cc 
    )

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME} DESTINATION /usr/lib)