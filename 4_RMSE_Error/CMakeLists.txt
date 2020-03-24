cmake_minimum_required (VERSION 3.0)
project(RMSE_Error)

set(MAIN_FILE src/main.cpp) 

add_executable(RMSE_Error ${MAIN_FILE})
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (RMSE_Error Eigen3::Eigen)