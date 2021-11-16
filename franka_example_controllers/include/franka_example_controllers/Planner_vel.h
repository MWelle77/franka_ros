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

#include "franka_example_controllers/LinearProfile.h"

#define DEFAULT_M 6
#define DEFAULT_VEL_POS 0.03
#define DEFAULT_VEL_Q 0.06
#define VEL_THRESHOLD 0.01
#define Q_THRESHOLD 0.1
#define MIN_EXEC_TIME 0.1


using namespace std; 

class Planner_vel {

private: 
	vector<LinearProfile> prof_vec_; 
	int m_; // size
	double default_velocity_pos_, default_vel_q_, vel_threshold_, dq_threshold_, min_execution_time_; 
	Eigen::VectorXd des_vel_; 
	Eigen::VectorXd des_dq_; 
	double tf_; 
	double init_time_; 
	
public:
	Planner_vel(int m = DEFAULT_M, double vel_threshold = VEL_THRESHOLD, double q_threshold = Q_THRESHOLD, double default_vel_pos = DEFAULT_VEL_POS, double default_vel_q = DEFAULT_VEL_Q, double min_execution_time = MIN_EXEC_TIME);
	// Functions for operative space 
	void init(const Eigen::VectorXd& qi, const Eigen::VectorXd& curr_dqi, const Eigen::VectorXd& des_vel, double init_time); 
	void plan(const Eigen::VectorXd& qi, const Eigen::VectorXd& curr_vel, const Eigen::VectorXd& des_vel, double current_time, double duration); 
	void getPose(double curr_time, Eigen::VectorXd& x, Eigen::VectorXd& dx, Eigen::VectorXd& ddx);
	bool velReached(const Eigen::VectorXd& curr_vel, double curr_time ); 

	
	// General functions 
	void initdQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& curr_dq, const Eigen::VectorXd& des_dq, double init_time); 
	void plandQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& curr_dq, const Eigen::VectorXd& des_dq, double current_time, double duration); 
	void getPoseQ(double curr_time, Eigen::VectorXd& q, Eigen::VectorXd& dq, Eigen::VectorXd& ddq);
	bool velReachedQ(const Eigen::VectorXd& curr_dq, double curr_time); 
	
	
	};




