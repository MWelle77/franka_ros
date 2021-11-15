
#include "franka_example_controllers/PlannerCoop.h"

inline double sign(double n){
	if (n > 0){
		return 1; 
	}else if (n < 0){
		return -1; 
	}else{
		return 0; 
	}	
}


PlannerCoop::PlannerCoop(int N, int m, double pos_threshold, double or_threshold,  double q_threshold,  double default_vel_pos,  double default_vel_or,  double min_execution_time){
	//m_ = m; 
	//prof_vec_.resize(m_); 
	m_ = 0; 
	N_ = 2; 
	tf_ = 0; 
	pos_threshold_ = pos_threshold; 
	or_threshold_ = or_threshold; 
	q_threshold_ = q_threshold; 
	default_velocity_pos_ = default_vel_pos;
	default_velocity_or_ = default_vel_or; 
	min_execution_time_ = min_execution_time; 
	cout << "PlannerCoop: Creating PlannerCoop with m = "<< m_ << "and  default velocity pos " << default_velocity_pos_ << ", default velocity or " << default_velocity_or_ <<endl; 
	cout << "PlannerCoop: min exectution time: "<< min_execution_time <<endl; 
}



void PlannerCoop::init(const Eigen::VectorXd& curr_sigma, double init_time ){
	m_ = curr_sigma.size(); 
	prof_vec_.resize(m_); 
	init_time_ = init_time; 
	tf_ = 0; 
	for (int i = 0; i<m_; i++){
		prof_vec_[i].init(curr_sigma[i], curr_sigma[i], 0, init_time); 
	} 
	des_sigma_.resize(m_); 
	des_sigma_ = curr_sigma; 
}

void PlannerCoop::plan(const Eigen::VectorXd& curr_sigma, const Eigen::VectorXd& des_sigma, double current_time, double duration){
	double tf = duration;
	des_sigma_ = des_sigma; 
	init_time_ = current_time; 
	tf_ = 0; 
	for (int j = 0; j<N_ ; j++){
		for (int i = 0; i<3; i++){
		if(duration == 0){
			tf = abs(curr_sigma[j*6+i] - des_sigma_[j*6+i])/default_velocity_pos_; 
			if (tf < min_execution_time_){
				tf = min_execution_time_; 
			}
		}
		// Position 
		prof_vec_[j*6+i].init(curr_sigma[j*6+i], des_sigma_[j*6+i], tf, current_time); 
		if (tf > tf_) tf_ = tf; 
	
		// Orientation
		if ( abs(curr_sigma[j*6+i+3] - des_sigma_[j*6+i+3]) > 2*M_PI){
			des_sigma_[j*6+i+3] = des_sigma_[j*6+i+3] + sign(curr_sigma[j*6+i+3])*2*M_PI; 
		}
		if(duration == 0){
			tf = abs(curr_sigma[j*6+i+3] - des_sigma_[j*6+i+3])/default_velocity_or_;
			if (tf < min_execution_time_){
				tf = min_execution_time_; 
			} 
		}
		prof_vec_[j*6+i+3].init(curr_sigma[j*6+i+3], des_sigma_[j*6+i+3], tf, current_time); 
		if (tf > tf_) tf_ = tf; 
	}

	}

}

void PlannerCoop::getPose(double curr_time, Eigen::VectorXd& qd, Eigen::VectorXd& dqd, Eigen::VectorXd& ddqd){
	for (int i = 0; i<m_; i++){
		prof_vec_[i].getPosFromPlanner(curr_time, qd[i], dqd[i], ddqd[i]); 
	}

}

bool PlannerCoop::goalReached(const Eigen::VectorXd& curr_sigma, double curr_time){
	bool success = false; 
	/*
	TO CHANGE DIFFERENZIANDO POS E OR
	double pos_error = (curr_pos-des_pos_).norm(); 
	double or_error = (curr_or - des_or_).norm(); 
	if ((curr_time - init_time_) >= tf_ && pos_error < pos_threshold_ && or_error < or_threshold_)
		success = true; 
	*/
	double q_error = (curr_sigma - des_sigma_).norm(); 
	if (curr_time - init_time_ >= tf_ &&  q_error < q_threshold_)
		success = true; 
	//cout << "Norm error: " << q_error <<endl; 
	return success; 

} 
















