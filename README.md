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

<img width="324" alt="paper" src="https://user-images.githubusercontent.com/12381733/77613266-9e873180-6f6d-11ea-9823-a955b5613862.PNG">

[_Image Source_](https://s3-ap-south-1.amazonaws.com/av-blog-media/wp-content/uploads/2018/05/rmse.png)

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

```c++
    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
```

If your state vector has 5 states, you must select 5 * 2 + 1 = **11** points.

Generally, If your state vector has `N` states, you must select `2 * N + 1` sigma points.

2. Select spreading factor. (It is related to how far away from the mean you will choose.)

<img width="324" alt="paper" src="https://user-images.githubusercontent.com/12381733/77551320-8d9ad980-6ef5-11ea-8183-41cb8d7d9ae7.png">

According to formula above, Square Root of certain Matrix Px is required. 
This can be calculated by given functions in `Eigen` Library.

```c++
MatrixXd A = P.llt().matrixL();
```

### Augment Sigma Points 

Augmentation must be considered Before putting Sigma Points into process function.

`Augmentation` means considering process noise vector. And this also has a non-linear effect. (In CTRV model, It'll be noise about acceleration and yaw acceleration)

It can be applied simply by adding noise vector to state vector.

See how can implement that.

```c++
    // create example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
        -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
        0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
        -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
        -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;
 
    ...
   
    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P;
    P_aug(5, 5) = std_a * std_a;
    P_aug(6, 6) = std_yawdd * std_yawdd;
```


### Sigma Point Prediction

In this step, We'll gonna insert augmented Sigma Points that had been created at previous step into process model.
Pay attention to the dimension of state vectors.

This exercise uses `CTRV` model, So Input vector dimension is 7 (5 state dimension + 2 augmented dimension).
And the output vector dimension will be 5 (same with dimension of `CTRV` model)

Here's Helpful Equation for calculating predicted sigma points.

<img width="600" alt="helpful_equation" src="https://user-images.githubusercontent.com/12381733/77659910-fea4c480-6fbb-11ea-850e-515db99d9b89.png">

Be careful that there are exceptions.

### Predicted Mean Covariance 

The standard rule for calculating predicted mean and covariance of a group of state samples is given by these equations.
Not just same as basic mean/covariance calcuation, there's additional weights in each step.
There's several suggestions about that. But this example stick with rules from literature.

<img width="1000" alt="predicted_mean_covariance" src="https://user-images.githubusercontent.com/12381733/77661289-c7371780-6fbd-11ea-8387-f56b2c2dcbd2.png">


```c++
    // create vector for weights
    VectorXd weights = VectorXd(2 * n_aug + 1);

    // create vector for predicted state
    VectorXd x = VectorXd(n_x);

    // create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x, n_x);
```

Calculate `P` and `x` and `weights` with given parameters

> Be aware that there's angel subtracting in calculation. 


### Predict Radar Measurement

This example assumes that our system has 5 dimensional state vector. (According to CTRV Model)
And let's think about the situation that we'd like to update our state vector by using Radar sensor.

Radar sensor gives us 3 data. Distance from object, Yaw angle from object, velocity of object.
But our predicted mean and covariance elements have different dimension and also different unit from them.

So, need to transform them into the radar measurement space.
See how it can be calculated.

![measurement_model](https://user-images.githubusercontent.com/12381733/77735142-59d2c780-704d-11ea-879c-e0a1b82eafe8.PNG)

Hold on, This transformation is also non-linear transformation :( 
Should we need to do things that we've done before?? (Create Sigma Points, Augmentaion, Prediction...)
Actually, **No** :) Sigma Points are already created and Measurement noise are purely additive factor.

So, The only things must be done in this step is transforming into radar measurement space and add the measurement covariance noise.



### UKF Update
---

### Reference

* [Udacity Sensor Fusion Nanodegree](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)
* [The Unscented Kalman Filter for Nonlinear Estimation](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf)
