cmake_minimum_required (VERSION 3.0)
project(Calculate_Jacobian)

set(MAIN_FILE src/main.cpp) 

add_executable(Calculate_Jacobian ${MAIN_FILE})
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (Calculate_Jacobian Eigen3::Eigen)