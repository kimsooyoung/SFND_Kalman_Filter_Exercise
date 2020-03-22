cmake_minimum_required (VERSION 3.0)
project(test)

set(MAIN_FILE src/main.cpp) 

add_executable(test ${MAIN_FILE} src/tracking.cpp src/kalman_filter.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (test Eigen3::Eigen)