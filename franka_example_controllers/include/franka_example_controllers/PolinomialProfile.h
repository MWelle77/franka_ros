#pragma once
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

using namespace std;

class PolinomialProfile {
private:
    double init_time; 
    double tf; 
    double qi, qf, vi;  // vi is the initial velocity
    Eigen::Matrix<double, 6, 1> coeff; 

public:
    PolinomialProfile(void);
    void init(double qi, double vi, double qf, double tf, double init_time = 0);
    void getPosFromPlanner(double curr_time, double &q, double &dq, double &ddq);
};