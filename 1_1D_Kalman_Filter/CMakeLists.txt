cmake_minimum_required (VERSION 3.0)
project(1D_Kalman_Filter)

add_executable(1D_Kalman_Filter src/test.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (1D_Kalman_Filter Eigen3::Eigen)