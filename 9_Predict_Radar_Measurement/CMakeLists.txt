cmake_minimum_required (VERSION 3.0)
project(Predict_Radar_Measurement)

set(MAIN_FILE src/main.cpp) 

add_executable(Predict_Radar_Measurement ${MAIN_FILE} src/ukf.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (Predict_Radar_Measurement Eigen3::Eigen)