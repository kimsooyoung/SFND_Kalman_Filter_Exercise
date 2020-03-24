cmake_minimum_required (VERSION 3.0)
project(Sigma_Point_Prediction)

set(MAIN_FILE src/main.cpp) 

add_executable(Sigma_Point_Prediction ${MAIN_FILE} src/ukf.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (Sigma_Point_Prediction Eigen3::Eigen)