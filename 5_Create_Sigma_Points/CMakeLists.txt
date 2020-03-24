cmake_minimum_required (VERSION 3.0)
project(Create_Sigma_Points)

set(MAIN_FILE src/main.cpp) 

add_executable(Create_Sigma_Points ${MAIN_FILE} src/ukf.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (Create_Sigma_Points Eigen3::Eigen)