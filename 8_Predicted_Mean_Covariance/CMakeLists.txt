cmake_minimum_required (VERSION 3.0)
project(Predicted_Mean_Covariance)

set(MAIN_FILE src/main.cpp) 

add_executable(Predicted_Mean_Covariance ${MAIN_FILE} src/ukf.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (Predicted_Mean_Covariance Eigen3::Eigen)