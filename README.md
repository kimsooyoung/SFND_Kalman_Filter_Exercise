# Sensor Fusion Self-Driving Car Course - Kalman Filter Exercise

Implement Basic Kalman Filter using C++

### Project Status:

![issue_badge](https://img.shields.io/badge/build-Passing-green) ![issue_badge](https://img.shields.io/badge/UdacityRubric-Passing-green)

---

## Build Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* PCL 1.2
* Eigen 3.3.7
  * Can be downloaded from this [link](https://video.udacity-data.com/topher/2017/March/58b7604e_eigen/eigen.zip)
 

## Basic Build Instructions

1. Clone this repo.
2. Select one project in this repo
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run Builded executable file : ex) `./ukf_highway`

---

### Project Explanation

This repo contains various C++ implementaions From Basic `1D Kalman filter` to `Unscented Kalman Filter (UKF)`

### 1D Basic Kalman Filter

Implements a multi-dimensional Kalman Filter for the example given

Kalman Filter process is consist of 2 step

* Measurement Update Step

```c++
        // KF Measurement update step
        VectorXd y = z - H * x;
        VectorXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();
```

* Prediction Step

```
        // new state
        x = x + K * y;
        P = (I - K * H) * P;

        // KF Prediction step
        x = F * x + u;
        P = F * P * F.transpose();
```

### 2D Basic Kalman Filter



### Calculate Jabobian

### RMSE Error 

### Create Sigma Points

### Augment Sigma Points 

### Sigma Point Prediction

### Predicted Mean Covariance 

### Predict Radar Measurement

### UKF Update
---

### Reference

* [Udacity Sensor Fusion Nanodegree](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)
