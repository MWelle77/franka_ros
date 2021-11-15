#pragma once


// Std includes
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

// Tf and eigen includes
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "franka_example_controllers/PolinomialProfile.h"

#define DEFAULT_M 6
#define DEFAULT_N 2
#define DEFAULT_VEL_POS_C 0.03
#define DEFAULT_VEL_OR_C 0.1
#define POS_THRESHOLD 0.1
#define OR_THRESHOLD 0.1
#define Q_THRESHOLD 0.1
#define MIN_EXEC_TIME_C 0.8


using namespace std; 

class PlannerCoop {

private: 
	vector<PolinomialProfile> prof_vec_; 
	int m_; // size
	double default_velocity_pos_, default_velocity_or_, or_threshold_, pos_threshold_, q_threshold_, min_execution_time_; 
	Eigen::VectorXd des_sigma_; 
	double tf_; 
	double init_time_; 
	int N_; 
public:
	PlannerCoop(int N = DEFAULT_N, int m = DEFAULT_M, double pos_threshold = POS_THRESHOLD, double or_threshold = OR_THRESHOLD, double q_threshold = Q_THRESHOLD, double default_vel_pos = DEFAULT_VEL_POS_C,  double default_vel_or = DEFAULT_VEL_OR_C, double min_execution_time = MIN_EXEC_TIME_C);
	
	// General functions 
	void init(const Eigen::VectorXd& curr_q, double init_time = 0); 
	void plan(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& des_q, double current_time, double tf = 0); 
	void getPose(double curr_time, Eigen::VectorXd& q, Eigen::VectorXd& dq, Eigen::VectorXd& ddq);
	bool goalReached(const Eigen::VectorXd& curr_q, double curr_time); 
	
	
	};




