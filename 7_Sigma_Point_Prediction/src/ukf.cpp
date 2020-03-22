#include <iostream>
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF()
{
    Init();
}

UKF::~UKF()
{
}

void UKF::Init()
{
}

/**
 * Programming assignment functions: 
 */

void UKF::SigmaPointPrediction(MatrixXd *Xsig_out)
{

    // set state dimension
    int n_x = 5;

    // set augmented dimension
    int n_aug = 7;

    // create example sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
    Xsig_aug << 5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441,
        1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 1.38, 1.38,
        2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049,
        0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015,
        0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.3528,
        0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641, 0,
        0, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641;

    // create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

    double delta_t = 0.1; // time diff in sec

    /**
   * Student part begin
   */
    for (int i = 0; i < 2 * n_aug + 1; i++)
    {
        float p_x = Xsig_aug(0, i);
        float p_y = Xsig_aug(1, i);
        float v = Xsig_aug(2, i);
        float yaw = Xsig_aug(3, i);
        float yawd = Xsig_aug(4, i);
        float nu_a = Xsig_aug(5, i);
        float nu_yawdd = Xsig_aug(6, i);

        // avoid division by zero
        if (yawd != 0)
        {
            Xsig_pred(0, i) = p_x + (v * (sin(yaw + yawd * delta_t) - sin(yaw)) / yawd) + (.5 * delta_t * delta_t * cos(yaw) * nu_a);
            Xsig_pred(1, i) = p_y + (v * (-cos(yaw + yawd * delta_t) + cos(yaw)) / yawd) + (.5 * delta_t * delta_t * sin(yaw) * nu_a);
            Xsig_pred(2, i) = v + 0 + (delta_t * nu_a);
            Xsig_pred(3, i) = yaw + (yawd * delta_t) + (0.5 * delta_t * delta_t * nu_yawdd);
            Xsig_pred(4, i) = yawd + 0 + (delta_t * nu_yawdd);
        }
        else
        {
            Xsig_pred(0, i) = p_x + (v * cos(yaw) * delta_t) + (.5 * delta_t * delta_t * cos(yaw) * nu_a);
            Xsig_pred(1, i) = p_y + (v * sin(yaw) * delta_t) + (.5 * delta_t * delta_t * sin(yaw) * nu_a);
            Xsig_pred(2, i) = v + 0 + (delta_t * nu_a);
            Xsig_pred(3, i) = yaw + (yawd * delta_t) + (0.5 * delta_t * delta_t * nu_yawdd);
            Xsig_pred(4, i) = yawd + 0 + (delta_t * nu_yawdd);
        }
    }

    // predict sigma points

    // write predicted sigma points into right column

    /**
   * Student part end
   */

    // print result
    std::cout << "Xsig_pred = " << std::endl
              << Xsig_pred << std::endl;

    // write result
    *Xsig_out = Xsig_pred;
}