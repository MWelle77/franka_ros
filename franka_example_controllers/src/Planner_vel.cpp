
#include "franka_example_controllers/Planner_vel.h"

inline double sign(double n){
	if (n > 0){
		return 1; 
	}else if (n < 0){
		return -1; 
	}else{
		return 0; 
	}	
}


Planner_vel::Planner_vel(int m, double vel_threshold,  double dq_threshold,  double default_vel_pos,  double default_vel_q,  double min_execution_time){
	//m_ = m; 
	//prof_vec_.resize(m_); 
	m_ = 0; 
	tf_ = 0; 
	vel_threshold_ = vel_threshold; 
	dq_threshold_ = dq_threshold; 
	default_velocity_pos_ = default_vel_pos;
	default_vel_q_ = default_vel_q; 
	min_execution_time_ = min_execution_time; 
	cout << "PLANNER: Creating planner with m = "<< m_ << "and  default velocity pos " << default_velocity_pos_ << endl; 
	cout << "PLANNER: min exectution time: "<< min_execution_time <<endl; 
}


void Planner_vel::init(const Eigen::VectorXd& qi, const Eigen::VectorXd& curr_dqi, const Eigen::VectorXd& des_vel, double init_time){	
	m_ = 6; 
	init_time_ = init_time; 
	tf_ = 0; 
	prof_vec_.resize(m_); 
	for (int i = 0; i<6; i++){
		prof_vec_[i].init(qi[i], curr_dqi[i], des_vel[i], 0.1, init_time); 
	} 
	des_vel_ = des_vel; 
}

void Planner_vel::plan(const Eigen::VectorXd& qi, const Eigen::VectorXd& curr_vel, const Eigen::VectorXd& des_vel, double current_time, double duration){
	
	double tf = duration;
	des_vel_ = des_vel; 
	init_time_ = current_time; 
	tf_ = 0; 
	for (int i = 0; i<6; i++){
		// TO ADD: CHECK FEASIBILITY IF TAKEN AS INPUT
		if(duration == 0){
			tf = abs(curr_vel[i] - des_vel_[i])/default_velocity_pos_; 
			if (tf < min_execution_time_){
				tf = min_execution_time_; 
			}
		}
		// XYZ
		prof_vec_[i].init(qi[i], curr_vel[i], des_vel_[i], tf, current_time); 
		if (tf > tf_) tf_ = tf; 
		
	}
	
	//cout << "Planned vell motion from "<<curr_vel.transpose() << " to " << des_vel_.transpose() <<endl; 

}

void Planner_vel::getPose(double current_time, Eigen::VectorXd& xd, Eigen::VectorXd& dxd, Eigen::VectorXd& ddxd){
	// Desired position	
	for (int i = 0; i<6; i++){
		prof_vec_[i].getPosFromPlanner(current_time, xd[i], dxd[i], ddxd[i]); 
	}
}


bool Planner_vel::velReached(const Eigen::VectorXd& curr_vel, double curr_time){
	bool success = false; 
	double vel_error = (curr_vel-des_vel_).norm(); 
	if ((curr_time - init_time_) >= tf_ && vel_error < vel_threshold_)
		success = true; 
	//cout << "Norm position error: " << pos_error << " Norm orientation error " <<or_error<<endl; 
	return success; 
	
}


void Planner_vel::initdQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& curr_dq, const Eigen::VectorXd& des_dq, double init_time){
	m_ = curr_q.size(); 
	prof_vec_.resize(m_); 
	init_time_ = init_time; 
	tf_ = 0; 
	for (int i = 0; i<m_; i++){
		prof_vec_[i].init(curr_q[i], curr_dq[i], des_dq[i], 0.1, init_time); 
		//prof_vec_[i].init(curr_dq[i], curr_dq[i], 0, init_time); 
	} 
	des_dq_.resize(curr_dq.size()); 
	des_dq_ = curr_dq*0; //when init we want 0 vel 

}

void Planner_vel::plandQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& curr_dq, const Eigen::VectorXd& des_dq, double current_time, double duration){
	double tf = duration;
	des_dq_ = des_dq; 
	init_time_ = current_time; 
	tf_ = 0; 
	for (int i = 0; i<m_; i++){

		if(duration == 0){
			tf = abs(curr_dq[i] - des_dq_[i])/default_vel_q_;
			if (tf < min_execution_time_){
				tf = min_execution_time_; 
			} 
		}
		//prof_vec_[i].init(curr_q[i], des_dq_[i], tf, current_time); 
		prof_vec_[i].init(curr_q[i], curr_dq[i], des_dq[i], 0.1, current_time); 
		if (tf > tf_) tf_ = tf; 
	}
}

void Planner_vel::getPoseQ(double curr_time, Eigen::VectorXd& qd, Eigen::VectorXd& dqd, Eigen::VectorXd& ddqd){
	for (int i = 0; i<m_; i++){
		prof_vec_[i].getPosFromPlanner(curr_time, qd[i], dqd[i], ddqd[i]); 
		//cout << "get pose ddqdddqd: " << ddqd[i] <<endl; 
	}


}

bool Planner_vel::velReachedQ(const Eigen::VectorXd& curr_dq, double curr_time){
	bool success = false; 
	double dq_error = (curr_dq - des_dq_).norm(); 
	if (curr_time - init_time_ >= tf_ &&  dq_error < dq_threshold_)
		success = true; 
	//cout << "Norm error: " << q_error <<endl; 
	return success; 

} 
















