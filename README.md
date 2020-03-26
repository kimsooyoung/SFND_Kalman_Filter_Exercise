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

Same with 1D Kalman Filter, Just Expanded to 3D dimension.
But, In this step use, we'll try to adapt more advanced concept, `Process Covariance Matrix`.

In Real World, None of all objects moves with constant velocity. They're lots of inner/external Powers (Occured by Motor or By 
someone's kicking...) that makes `Acceleration`. 
In order to consider those acceleration from various directions. An Uncertainty matrix suitable for the model used can be founded.

That Matrix is reffered to `Process Covariance Matrix`

```c++

    // 2. Set the process covariance matrix Q
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    kf_.Q_ = MatrixXd(4, 4);
    kf_.Q_ << dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
        0, dt_4 * noise_ay / 4, 0, dt_3 * noise_ay / 2,
        dt_3 * noise_ax / 2, 0, dt_2 * noise_ax, 0,
        0, dt_3 * noise_ay / 2, 0, dt_2 * noise_ay;
    // 3. Call the Kalman Filter predict() function
```

See how it can be calculated.

### Calculate Jabobian

The power of Kalman Filter is came from Gaussian.
Because of the special feature (Uni-Modal) of Gaussian, we could predict certain one objects point.

But Basic Kalman Filter cannot useful when, is used for non-linear transformation.
It's almost impossible to ensure that prediction output produce one object point.

But, It can be solved By linearlizing transformation model by performing [Taylor's Series Expansion](https://en.wikipedia.org/wiki/Taylor_series).

New concept named `Jabobian` need to calulated for that.
Especially, This project shows linearlized transformation between `Lidar` sensed data and `Radar` sensed data.

```c++
    // check division by zero
    if (fabs(c1) < 0.0001)
    {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }
```
 
watch out for division-by-zero

### RMSE Error 

Check the performance of implemented tracking algorithm in terms of how far the estimated result is from the ground-truth.
`Root Mean Squared Error (RMSE)` is the most common method to do that.

![rmse_error](https://user-images.githubusercontent.com/12381733/77613266-9e873180-6f6d-11ea-9823-a955b5613862.PNG)

[Image Source](https://s3-ap-south-1.amazonaws.com/av-blog-media/wp-content/uploads/2018/05/rmse.png)

Lower value means higher estimate accurancy.

---

Basic Kalman Filter is already powerful. But it performs poors result when object moves along the non-straight line.
In order to handle those kinds of issue. Another Model such like CTRV (Constant Turn Rate and Velocity magnitude) can be used.

Moreover, Rather than using Basic Kalman Filter & Extended Kalman Filter, There's another great method named `UKF(Unscented Kalman Filter)` that can greatfully handle non-linear transformation.

The difference between `EKF` and `UKF` is how deal with non-linear measurement/process model.
For that, `UKF` uses an approach that called `Unscented transformation`.

And `UKF` can be splited into three steps.

1. Create `Sigma Points`

2. Predict next state of `Sigma Points` (Just insert them into Process Function)

3. Calculate prediction meam/covariance from predicted sigma points 

Can you get a hunch about `Sigma Point`??

We can approximate predicted mean/covariance through selecting certain few points from original gaussian distribution and applying process step only using them.

Steps below shows about those processes.

### Create Sigma Points

There's few rules about generating Simga Points.

1. The number of sigma points depends on the state dimension.

2. Select spreading factor. (It is related to how far away from the mean you will choose.)

<img width="324" alt="paper" src="https://user-images.githubusercontent.com/12381733/77551320-8d9ad980-6ef5-11ea-8183-41cb8d7d9ae7.png">

According to formula above, Square Root of certain Matrix Px is required. 
This can be calculated by given functions in `Eigen` Library.

```c++
MatrixXd A = P.llt().matrixL();
```

### Augment Sigma Points 

### Sigma Point Prediction

### Predicted Mean Covariance 

### Predict Radar Measurement

### UKF Update
---

### Reference

* [Udacity Sensor Fusion Nanodegree](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)
* [The Unscented Kalman Filter for Nonlinear Estimation](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf)
