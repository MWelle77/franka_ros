#include "franka_example_controllers/Planner.h"

inline double sign(double n){
    return (n > 0) ? 1 : ((n < 0) ? -1 : 0);
}

Planner::Planner(int m, double pos_threshold, double or_threshold,  double q_threshold,  
                 double default_vel_pos,  double default_vel_or, double default_vel_q,  
                 double min_execution_time) : m_(m), pos_threshold_(pos_threshold), or_threshold_(or_threshold), 
                 q_threshold_(q_threshold), default_velocity_pos_(default_vel_pos), default_velocity_or_(default_vel_or), 
                 default_vel_q_(default_vel_q), min_execution_time_(min_execution_time) {
    prof_vec_.resize(m_);
    cout << "PLANNER: Creating planner with m = " << m_ << " and default velocity pos " 
         << default_velocity_pos_ << ", default velocity or " << default_velocity_or_ << endl; 
    cout << "PLANNER: min execution time: " << min_execution_time << endl; 
}

void Planner::init(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& curr_eul_angles, const Eigen::Vector3d& curr_ang_vel, double init_time) {    
    init_time_ = init_time;
    tf_ = 0; 
    des_pos_ = curr_pos; 
    des_or_ = curr_eul_angles; 
    for (int i = 0; i < 3; i++) {
        prof_vec_[i].init(curr_pos[i], curr_vel[i], curr_pos[i], 0, init_time); 
        prof_vec_[i+3].init(curr_eul_angles[i], curr_ang_vel[i], curr_eul_angles[i], 0, init_time); 
    } 
}

void Planner::plan(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& curr_or, const Eigen::Vector3d& curr_ang_vel, const Eigen::Vector3d& des_pos, const Eigen::Vector3d& des_or, double current_time, double duration) {
    if (duration == 0) {
        double max_duration = 0;
        for (int i = 0; i < 3; i++) {
            double pos_duration = std::abs(des_pos[i] - curr_pos[i]) / default_velocity_pos_;
            double or_duration = std::abs(des_or[i] - curr_or[i]) / default_velocity_or_;
            max_duration = std::max(max_duration, std::max(pos_duration, or_duration));
        }
        duration = std::max(max_duration, min_execution_time_);
    }
    tf_ = duration;
    init_time_ = current_time;
    for (int i = 0; i < 3; i++) {
        prof_vec_[i].init(curr_pos[i], curr_vel[i], des_pos[i], duration, current_time);
        prof_vec_[i+3].init(curr_or[i], curr_ang_vel[i], des_or[i], duration, current_time);
    }
}

void Planner::getPose(double current_time, Eigen::VectorXd& x, Eigen::VectorXd& dx, Eigen::VectorXd& ddx) {
    x.resize(6);
    dx.resize(6);
    ddx.resize(6);
    for (int i = 0; i < 6; i++) {
        prof_vec_[i].getPosFromPlanner(current_time, x[i], dx[i], ddx[i]);
    }
}

bool Planner::goalReached(const Eigen::Vector3d& curr_pos, const Eigen::Vector3d& curr_or, double curr_time) {
    double pos_error = (curr_pos - des_pos_).norm();
    double or_error = (curr_or - des_or_).norm();
    return (curr_time - init_time_ >= tf_ && pos_error < pos_threshold_ && or_error < or_threshold_);
}

void Planner::initQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& curr_vel_q, double init_time) {
    m_ = curr_q.size(); 
    prof_vec_.resize(m_);
    init_time_ = init_time;
    tf_ = 0;
    des_q_ = curr_q; // Initially set desired joint states to current ones
    for (int i = 0; i < m_; i++) {
        prof_vec_[i].init(curr_q[i], curr_vel_q[i], curr_q[i], 0, init_time); // Assuming zero target velocity
    }
}

void Planner::planQ(const Eigen::VectorXd& curr_q, const Eigen::VectorXd& curr_vel_q, const Eigen::VectorXd& des_q, double current_time, double duration) {
    if (duration == 0) {
        double max_duration = 0;
        for (int i = 0; i < m_; i++) {
            double joint_duration = std::abs(des_q[i] - curr_q[i]) / default_vel_q_;
            max_duration = std::max(max_duration, joint_duration);
        }
        duration = std::max(max_duration, min_execution_time_);
    }
    des_q_ = des_q; 
    init_time_ = current_time;
    tf_ = duration;
    for (int i = 0; i < m_; i++) {
        prof_vec_[i].init(curr_q[i], curr_vel_q[i], des_q[i], duration, current_time); 
    }
}

void Planner::getPoseQ(double current_time, Eigen::VectorXd& q, Eigen::VectorXd& dq, Eigen::VectorXd& ddq) {
    q.resize(m_);
    dq.resize(m_);
    ddq.resize(m_);
    for (int i = 0; i < m_; i++) {
        prof_vec_[i].getPosFromPlanner(current_time, q[i], dq[i], ddq[i]);
    }
}

bool Planner::goalReachedQ(const Eigen::VectorXd& curr_q, double curr_time) {
    double q_error = (curr_q - des_q_).norm();
    return (curr_time - init_time_ >= tf_ && q_error < q_threshold_);
}
