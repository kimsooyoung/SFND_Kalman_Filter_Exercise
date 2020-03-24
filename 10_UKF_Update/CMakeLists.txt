cmake_minimum_required (VERSION 3.0)
project(UKF_Update)

set(MAIN_FILE src/main.cpp) 

add_executable(UKF_Update ${MAIN_FILE} src/ukf.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (UKF_Update Eigen3::Eigen)