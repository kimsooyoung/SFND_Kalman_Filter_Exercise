cmake_minimum_required (VERSION 3.0)
project(test)

add_executable(test src/test.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (test Eigen3::Eigen)