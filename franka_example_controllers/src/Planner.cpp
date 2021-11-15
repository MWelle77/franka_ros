
#include "franka_example_controllers/Planner.h"

inline double sign(double n){
	if (n > 0){
		return 1; 
	}else if (n < 0){
		return -1; 
	}else{
		return 0; 
	}	
}


Planner::Planner(int m, double pos_threshold, double or_threshold,  double q_threshold,  double default_vel_pos,  double default_vel_or, double default_vel_q,  double min_execution_time){
	//m_ = m; 
	//prof_vec_.resize(m_); 
	m_ = 0; 
	tf_ = 0; 
	pos_threshold_ = pos_threshold; 
	or_threshold_ = or_threshold; 
	q_threshold_ = q_threshold; 
	default_velocity_pos_ = default_vel_pos;
	default_velocity_or_ = default_vel_or; 
	default_vel_q_ = default_vel_q; 
	min_execution_time_ = min_execution_time; 
	cout << "PLANNER: Creating planner with m = "<< m_ << "and  default velocity pos " << default_velocity_pos_ << ", default velocity or " << default_velocity_or_ <<endl; 
	cout << "PLANNER: min exectution time: "<< min_execution_time <<endl; 
}


void Planner::init(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_eul_angles, double init_time){	
	m_ = 6; 
	init_time_ = init_time; 
	tf_ = 0; 
	prof_vec_.resize(m_); 
	for (int i = 0; i<3; i++){
		prof_vec_[i].init(curr_pos[i], curr_pos[i], 0, init_time); 
    	prof_vec_[i+3].init(curr_eul_angles[i], curr_eul_angles[i], 0, init_time); 
	} 
	des_pos_ = curr_pos; 
	des_or_ = curr_eul_angles; 
}

void Planner::plan(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_or, const Eigen::Vector3d& des_pos, const Eigen::Vector3d& des_or, double current_time, double duration){
	
	double tf = duration;
	des_pos_ = des_pos; 
	des_or_ = des_or; 
	init_time_ = current_time; 
	tf_ = 0; 
	for (int i = 0; i<3; i++){
		// TO ADD: CHECK FEASIBILITY IF TAKEN AS INPUT
		if(duration == 0){
			tf = abs(curr_pos[i] - des_pos_[i])/default_velocity_pos_; 
			if (tf < min_execution_time_){
				tf = min_execution_time_; 
			}
		}
		// Position 
		prof_vec_[i].init(curr_pos[i], des_pos_[i], tf, current_time); 
		if (tf > tf_) tf_ = tf; 
		// Orientation
		if ( abs(curr_or[i] - des_or_[i]) >= 5){
			des_or_[i] = des_or_[i] + sign(curr_or[i])*2*M_PI; 
		}
		if(duration == 0){
			tf = abs(curr_or[i] - des_or_[i])/default_velocity_or_;
			if (tf < min_execution_time_){
				tf = min_execution_time_; 
			} 
		}
		prof_vec_[i+3].init(curr_or[i], des_or_[i], tf, current_time); 
		if (tf > tf_) tf_ = tf; 
	}
	
	cout << "Planned position motion from "<<curr_pos.transpose() << " to " << des_pos_.transpose() <<endl; 
	cout << "Planned orientation motion from "<<curr_or.transpose() << " to " << des_or_.transpose() <<endl; 

}

void Planner::getPose(double current_time, Eigen::VectorXd& xd, Eigen::VectorXd& dxd, Eigen::VectorXd& ddxd){
	// Desired position	
	for (int i = 0; i<3; i++){
		prof_vec_[i].getPosFromPlanner(current_time, xd[i], dxd[i], ddxd[i]); 
		prof_vec_[i+3].getPosFromPlanner(current_time, xd[i+3], dxd[i+3], ddxd[i+3]); 
	}
}


bool Planner::goalReached(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_or, double curr_time){
	bool success = false; 
	double pos_error = (curr_pos-des_pos_).norm(); 
	double or_error = (curr_or - des_or_).norm(); 
	if ((curr_time - init_time_) >= tf_ && pos_error < pos_threshold_ && or_error < or_threshold_)
		success = true; 
	//cout << "Norm position error: " << pos_error << " Norm orientation error " <<or_error<<endl; 
	return success; 
	
}


void Planner::initQ(const Eigen::VectorXd& curr_q, double init_time ){
	m_ = curr_q.size(); 
	prof_vec_.resize(m_); 
	init_time_ = init_time; 
	tf_ = 0; 
	for (int i = 0; i<m_; i++){
		prof_vec_[i].init(curr_q[i], curr_q[i], 0, init_time); 
	} 
	des_q_.resize(curr_q.size()); 
	des_q_ = curr_q; 

}

void Planner::planQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& des_q, double current_time, double duration){
	double tf = duration;
	des_q_ = des_q; 
	init_time_ = current_time; 
	tf_ = 0; 
	for (int i = 0; i<m_; i++){
		/*
		if ( abs(curr_q[i] - des_q_[i]) > M_PI){
			des_q_[i] = des_q_[i] + sign(curr_q[i])*2*M_PI; 
		}
		*/
		if(duration == 0){
			tf = abs(curr_q[i] - des_q_[i])/default_vel_q_;
			if (tf < min_execution_time_){
				tf = min_execution_time_; 
			} 
		}
		prof_vec_[i].init(curr_q[i], des_q_[i], tf, current_time); 
		if (tf > tf_) tf_ = tf; 
	}
}

void Planner::getPoseQ(double curr_time, Eigen::VectorXd& qd, Eigen::VectorXd& dqd, Eigen::VectorXd& ddqd){
	for (int i = 0; i<m_; i++){
		prof_vec_[i].getPosFromPlanner(curr_time, qd[i], dqd[i], ddqd[i]); 
	}

}

bool Planner::goalReachedQ(const Eigen::VectorXd& curr_q, double curr_time){
	bool success = false; 
	double q_error = (curr_q - des_q_).norm(); 
	if (curr_time - init_time_ >= tf_ &&  q_error < q_threshold_)
		success = true; 
	//cout << "Norm error: " << q_error <<endl; 
	return success; 

} 
















