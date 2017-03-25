#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO: Done.
    * Calculate the RMSE here.
  */

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if ((estimations.size() != ground_truth.size()) || (estimations.size() == 0)) {
        // cout << "Exception! Invalid estimations: " << estimations;
        throw std::invalid_argument( "Error: Estimation vector of invalid size");
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO: Done.
    * Calculate a Jacobian here.
  */

    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);


    //check division by zero
    //compute the Jacobian matrix
    if (px != 0 || py !=0) {
        float px2py2 = px*px + py*py;
        Hj << px/(sqrt(px2py2)), py/(sqrt(px2py2)), 0, 0,
                -py/(px2py2), px/px2py2, 0, 0,
                py*(vx*py-vy*px)/pow(px2py2, 1.5), px*(vy*px-vx*py)/pow(px2py2, 1.5), px/sqrt(px2py2), py/sqrt(px2py2);
    }

    return Hj;
}


VectorXd Tools::CalculateHNonLinear(const VectorXd& x_state) {
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    VectorXd h_non_linear (3);
    h_non_linear << 0,0,0;

    //compute the polar coordinates
    float px2py2 = px*px + py*py;
    h_non_linear(0) =  sqrt(px2py2);
    h_non_linear(1) = (px != 0 ? atan(py/px) : M_PI/2);
    h_non_linear(2) = (px2py2 != 0 ? (px * vx + py * vy) / (sqrt(px2py2)) : 0);
    return h_non_linear;
}

// From: http://stackoverflow.com/a/29871193/1321129
/* change to `float/fmodf` or `long double/fmodl` or `int/%` as appropriate */
/* wrap x -> [0,max) */
double Tools::wrapMax(double x, double max)
{
    /* integer math: `(max + x % max) % max` */
    return fmod(max + fmod(x, max), max);
}

/* wrap x -> [min,max) */
double Tools::wrapMinMax(double x, double min, double max)
{
    return min + wrapMax(x - min, max - min);
}
