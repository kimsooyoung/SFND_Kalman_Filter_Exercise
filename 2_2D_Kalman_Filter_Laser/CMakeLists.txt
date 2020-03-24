cmake_minimum_required (VERSION 3.0)
project(2D_Kalman_Filter)

set(MAIN_FILE src/main.cpp) 

add_executable(2D_Kalman_Filter ${MAIN_FILE} src/tracking.cpp src/kalman_filter.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (2D_Kalman_Filter Eigen3::Eigen)