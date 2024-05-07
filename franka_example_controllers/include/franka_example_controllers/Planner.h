#pragma once
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include "franka_example_controllers/PolinomialProfile.h"

#define DEFAULT_M 6
#define DEFAULT_VEL_POS 0.03
#define DEFAULT_VEL_OR 0.1
#define DEFAULT_VEL_Q 0.06
#define POS_THRESHOLD 0.1
#define OR_THRESHOLD 0.1
#define Q_THRESHOLD 0.1
//#define MIN_EXEC_TIME 0.5
#define MIN_EXEC_TIME 0.01

using namespace std;

class Planner {
private:
    vector<PolinomialProfile> prof_vec_;
    int m_;
    double default_velocity_pos_, default_velocity_or_, default_vel_q_;
    double or_threshold_, pos_threshold_, q_threshold_, min_execution_time_;
    Eigen::Vector3d des_pos_, des_or_;
    Eigen::VectorXd des_q_;
    double tf_;
    double init_time_;

public:
    Planner(int m = DEFAULT_M, double pos_threshold = POS_THRESHOLD, double or_threshold = OR_THRESHOLD,
            double q_threshold = Q_THRESHOLD, double default_vel_pos = DEFAULT_VEL_POS,  
            double default_vel_or = DEFAULT_VEL_OR, double default_vel_q = DEFAULT_VEL_Q, 
            double min_execution_time = MIN_EXEC_TIME);
    void init(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& curr_eul_angles, 
              const Eigen::Vector3d& curr_ang_vel, double init_time = 0);
    void plan(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& curr_or, 
              const Eigen::Vector3d& curr_ang_vel, const Eigen::Vector3d& des_pos, const Eigen::Vector3d& des_or, 
              double current_time, double tf = 0);
    void getPose(double current_time, Eigen::VectorXd& x, Eigen::VectorXd& dx, Eigen::VectorXd& ddx);
    bool goalReached(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_or, double curr_time);

    void initQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& curr_vel_q, double init_time = 0);
    void planQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& curr_vel_q, const Eigen::VectorXd& des_q, 
               double current_time, double tf = 0);
    void getPoseQ(double current_time, Eigen::VectorXd& q, Eigen::VectorXd& dq, Eigen::VectorXd& ddq);
    bool goalReachedQ(const Eigen::VectorXd& curr_q, double curr_time);
};
