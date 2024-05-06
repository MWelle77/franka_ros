// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/dual_arm_dressing_controller.h>

#include <cmath>
#include <functional>
#include <memory>

#include <controller_interface/controller_base.h>
#include <eigen_conversions/eigen_msg.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_example_controllers/CoopCentroidFormationDelta.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>



namespace franka_example_controllers {

//------------------- CHECK FUNCTIONS 

inline bool DualArmDressingController::checkTorqueLimits(const Eigen::Matrix<double, 7, 1>& tau, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(tau[i]) > perc_allowed_torque_limit_*torque_limits_[i]) {
          ROS_ERROR("%s: Close to torque %d limit: torque value=%f, torque limit=%f", arm_id.c_str(), i, tau[i],torque_limits_[i]);
          return false; 
      } 
  }  
  return true; 
}

inline  bool DualArmDressingController::checkDerivativeTorqueLimits(const Eigen::Matrix<double, 7, 1>& dtau, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(dtau[i]) > perc_allowed_dtorque_limit_*dtorque_limits_[i]) {
          ROS_ERROR("%s: Close to derivative torque %d limit: dtorque value=%f, dtorque limit=%f", arm_id.c_str(),  i, dtau[i],dtorque_limits_[i]);
          return false; 
      } 
  }  
  return true; 
}

inline bool DualArmDressingController::checkJointLimits(const Eigen::Matrix<double, 7, 1>& q, const string& arm_id){
   
  for(int i = 0; i < 7; i++){
      if ( q[i] < perc_allowed_q_limit_*q_min_limits_[i] || q[i] > perc_allowed_q_limit_*q_max_limits_[i] ) {
          ROS_ERROR("%s: Close to joint %d limit: joint value=%f, joint range=[%f, %f]", arm_id.c_str(), i, q[i],q_min_limits_[i], q_max_limits_[i] );
          return false; 
      } 
  }  
  return true; 
}

inline bool DualArmDressingController::checkVelocityJointLimits(const Eigen::Matrix<double, 7, 1>& dq, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(dq[i]) > perc_allowed_dq_limit_*dq_limits_[i]) {
          ROS_ERROR("%s: Close to joint velocity %d limit: dq value=%f, dq limit=%f", arm_id.c_str(), i, dq[i],dq_limits_[i]);
          return false; 
      } 
  }  
  return true; 

}

// --------------------------------------



bool DualArmDressingController::initArm(
    hardware_interface::RobotHW* robot_hw, const std::string& arm_id, const std::vector<std::string>& joint_names, ros::NodeHandle& node_handle, const Eigen::Affine3d& w_T_0, int ind_robot) {
  CustomFrankaDataContainerDressing arm_data;
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DualArmDressingController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }

  // Desired inertia  
  double Mdpos = 1;
  double Mdor = 1;  
  if (!node_handle.getParam("Mdpos", Mdpos)) {
    ROS_ERROR_STREAM("DualArmDressingController: Could not read parameter Mdpos");
    return false;
  }
  if (!node_handle.getParam("Mdor", Mdor)) {
    ROS_ERROR_STREAM("DualArmDressingController: Could not read parameter Mdor");
    return false;
  }
  arm_data.Md_.setIdentity(); 
  arm_data.Md_.topLeftCorner(3, 3) << Mdpos* Eigen::Matrix3d::Identity();
  arm_data.Md_.bottomRightCorner(3, 3) << Mdor * Eigen::Matrix3d::Identity();

  // Damping and stiffness
  std::vector<double> K_list; 
  string arm_prefix; 
  if (arm_id == "panda_left") {
    arm_prefix = "left"; 
  }else{
    arm_prefix = "right"; 
  }

  arm_data.Kd_.setIdentity();
  arm_data.Dd_.setIdentity();
  if( !node_handle.getParam(arm_prefix+"/Kp", K_list) || K_list.size() != p_arm )
      ROS_ERROR("DualArmDressingController: Could not read parameter Kp.");
  else{
    for (int i = 0; i< p_arm; i++){
      arm_data.Kd_(i,i) = K_list[i];  
      arm_data.Dd_(i,i) = 2*sqrt(K_list[i]);  
    }
  }

  cout <<  arm_id << " Md_ "<<endl << arm_data.Md_<<endl; 
  cout <<  arm_id << " Kd_ "<<endl << arm_data.Kd_<<endl; 
  cout <<  arm_id << " Dd_ "<<endl << arm_data.Dd_<<endl; 
  
  arm_data.Kd_q_.setIdentity(); 
  arm_data.Kd_q_.topLeftCorner(4,4) << 600* Eigen::MatrixXd::Identity(4,4);
  arm_data.Kd_q_(4,4) = 250.0; 
  arm_data.Kd_q_(5,5) = 150.0; 
  arm_data.Kd_q_(6,6) = 50.0; 

  arm_data.Dd_q_.setIdentity(); 
  arm_data.Dd_q_.topLeftCorner(3,3) << 50* Eigen::MatrixXd::Identity(3,3);
  arm_data.Dd_q_(3,3) = 20.0; 
  arm_data.Dd_q_(4,4) = 20.0; 
  arm_data.Dd_q_(5,5) = 20.0; 
  arm_data.Dd_q_(6,6) = 10.0; 

  arm_data.Kp_ik_.setIdentity(); 
  double kp_ik = 200; 
  arm_data.Kp_ik_ = kp_ik*Eigen::MatrixXd::Identity(6,6); 
  arm_data.Kd_ik_ = 2*sqrt(kp_ik)*Eigen::MatrixXd::Identity(6,6); 
  arm_data.dq_filtered_.setZero(); 

  cout <<  arm_id << " Kd_q_ "<<endl << arm_data.Kd_q_<<endl; 
  cout <<  arm_id << " Dd_q_ "<<endl << arm_data.Dd_q_<<endl; 
  // Get impedance flag (if 1 impedance controller is active)
  int impedence_active = 0; 
  if (!node_handle.getParam(arm_prefix+"/impedance", impedence_active)) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Could not read parameter impedance");
    return false;
  }
  arm_data.impedance_ = impedence_active; 
  cout << arm_id << " impedance active: " <<  arm_data.impedance_  <<endl; 

  // Get static trajectory flag (if 1 impedance controller is active)
  int static_traj = 1; 
  if (!node_handle.getParam(arm_prefix+"/static", static_traj) ) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Could not read parameter static_traj  ");
    return false;
  }
  if (static_traj == 0 && impedence_active == 0){
    ROS_WARN("Could not move the robot if the impedance controller is not active (impedance = 0)"); 
    static_traj = 1; 

  }
  arm_data.static_traj_ = static_traj; 
  cout << arm_id << " static trajectory: " <<  arm_data.static_traj_  <<endl; 


  arm_data.position_d_.setZero();
  arm_data.orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  arm_data.position_d_target_.setZero();
  arm_data.orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  arm_data.w_T_0 = w_T_0;
  arm_data.f0_T_w = w_T_0.inverse();
   
  arm_data.conf_initialized_ = false; 
  arm_data.initial_des_q_ << 0, 0, 0, -1.57, 0, 1.57, 0.78; 
  cout << "w_T_0 "  << arm_id << " "<< arm_data.w_T_0.translation().transpose()<<endl; 

  
  arm_data.learning_q_conf_requested_ = false; 
  //set learning poses q
  cout << arm_id << endl;
  if(arm_id=="panda_left"){
    arm_data.learning_pose_des_q_ << -0.33592609494760295, 0.06552340160501058, 0.0018736362563746359, -2.212750642003139, -1.7843937059773336, 1.8297946228736464, 1.5094995050429316; 
  }
  if(arm_id=="panda_right"){
    arm_data.learning_pose_des_q_ << 1.1806447589532785, -0.055027268280941795, -0.7565481623836025, -2.072917388549629, 0.3403112790849521, 2.1545090057328595, -0.5217275363174221; 
  }
  

  
  arm_data.Gamma_i_.setZero(); 
  arm_data.Gamma_i_.block<6,6>(0,ind_robot*6) = Eigen::Matrix<double, 6, 6>::Identity(); 
  arm_data.ind_robot_ = ind_robot; 
  cout << arm_id << " Gamma_i: " <<endl << arm_data.Gamma_i_ <<endl; 

  arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));


  return true;
}


bool DualArmDressingController::waitForInitialTransform(const tf::TransformListener& listener, tf::StampedTransform& transform, const string& frame1, const string& frame2){
  try {
    if (listener.waitForTransform(frame1, frame2, ros::Time(0), ros::Duration(4.0))) {
      listener.lookupTransform(frame1, frame2, ros::Time(0), transform);
    } else {
      ROS_ERROR( "DualArmDressingController: Failed to read transform from %s to %s. Aborting init!", frame1.c_str(), frame2.c_str());
      return false;
    }
  } catch (tf::TransformException& ex) {
    ROS_ERROR("DualArmDressingController: %s", ex.what());
    return false;
  }
  return true; 
}


bool DualArmDressingController::init(hardware_interface::RobotHW* robot_hw,
                                                      ros::NodeHandle& node_handle) {
  
  cout << "---------------------Start init call ----------------- "<<endl; 
  // Get arm id
  if (!node_handle.getParam("left/arm_id", left_arm_id_)) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Could not read parameter left_arm_id_");
    return false;
  }
  if (!node_handle.getParam("right/arm_id", right_arm_id_)) {
    ROS_ERROR_STREAM(
        "DualArmDressingController: Could not read parameter right_arm_id_");
    return false;
  }
  // Get joint names
  std::vector<std::string> left_joint_names;
  if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7) {
    ROS_ERROR(
        "DualArmDressingController: Invalid or no left_joint_names parameters provided, aborting controller init!");
    return false;
  }
  std::vector<std::string> right_joint_names;
  if (!node_handle.getParam("right/joint_names", right_joint_names) ||  right_joint_names.size() != 7) {
    ROS_ERROR(
        "DualArmDressingController: Invalid or no right_joint_names parameters provided, aborting controller init!");
    return false;
  }

  // ------------ Subscribers

  // Desired poses for the robots
  boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> callbackLeft =
      boost::bind(&DualArmDressingController::targetPoseCallback, this, _1, left_arm_id_);

  boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> callbackRight =
      boost::bind(&DualArmDressingController::targetPoseCallback, this, _1, right_arm_id_);


  ros::SubscribeOptions subscribe_options;
  subscribe_options.init("panda_left/equilibrium_pose", 1, callbackLeft);
  subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  sub_target_pose_left_ = node_handle.subscribe(subscribe_options);


  subscribe_options.init("panda_right/equilibrium_pose", 1, callbackRight);
  subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  sub_target_pose_right_ = node_handle.subscribe(subscribe_options);

  // Desired centroid 
  // boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> callbackCentroid =
  //     boost::bind(&DualArmDressingController::coopCentroidCallback, this, _1);

  // subscribe_options.init("coop/centroid_displacement", 1, callbackCentroid);
  // subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  // sub_target_centroid_displ_ = node_handle.subscribe(subscribe_options);


  // Desired distance 
  // boost::function<void(const std_msgs::Float64::ConstPtr&)> callbackFormationDistance =
  //     boost::bind(&DualArmDressingController::coopDistanceFormationCallback, this, _1);

  // subscribe_options.init("coop/formation_distance_displacement", 1, callbackFormationDistance);
  // subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  // sub_target_formation_distance_displ_ = node_handle.subscribe(subscribe_options);
  

  boost::function<void(const CoopCentroidFormationDelta::ConstPtr&)> callbackCentrFormationDistance =
      boost::bind(&DualArmDressingController::coopCentroidFormCallback, this, _1);

  subscribe_options.init("coop/centroid_formation_distance_displacement", 1, callbackCentrFormationDistance);
  subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  sub_target_centroid_distance_displ_= node_handle.subscribe(subscribe_options);

//calback for q motion to planning pose
  boost::function<void(const std_msgs::Bool::ConstPtr&)> callbackRequestLearingQ =
      boost::bind(&DualArmDressingController::requestLearningPoseCallback, this, _1);

  subscribe_options.init("coop/request_learning_qs", 1, callbackRequestLearingQ);
  subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  sub_request_learning_q_poses_= node_handle.subscribe(subscribe_options);


 // adding the subscribers for initial pose on demand
  boost::function<void(const std_msgs::Bool::ConstPtr&)> callbackRequestInitQ =
  boost::bind(&DualArmDressingController::requestInitPoseCallback, this, _1);

  subscribe_options.init("coop/request_init_qs", 1, callbackRequestInitQ);
  subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  sub_request_init_q_poses_= node_handle.subscribe(subscribe_options);


  // ---------- end of subscribers 

  // Get the transformation from right_O_frame to left_O_frame
  tf::StampedTransform transform;
  tf::TransformListener listener;
  waitForInitialTransform(listener, transform, left_arm_id_ + "_link0", right_arm_id_ + "_link0"); 
  tf::transformTFToEigen(transform, Ol_T_Or_); 

  Eigen::Affine3d w_T_0l, w_T_0r; // world to zero left annd world to zero right
  waitForInitialTransform(listener, transform, "base", left_arm_id_ + "_link0"); 
  tf::transformTFToEigen(transform, w_T_0l); 
  waitForInitialTransform(listener, transform, "base", right_arm_id_ + "_link0"); 
  tf::transformTFToEigen(transform, w_T_0r); 
  cout << "Initial translation w_0r" << w_T_0r.translation().transpose() << endl;  
  cout << "Initial translation w_0l" << w_T_0l.translation().transpose() << endl;

  int ind_robot = 0; 
  bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names, node_handle, w_T_0l, ind_robot++ );
  bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names, node_handle, w_T_0r, ind_robot);


  rate_trigger_ = franka_hw::TriggerRate(DEFAULT_PUBLISH_RATE); 
  // Setup publisher for the centering frame.
  center_frame_pub_.init(node_handle, "coop/centroid_pose", 1, true);
  coop_motion_finished_pub_.init(node_handle, "coop/motion_finished",1, true); 
  distance_pub_.init(node_handle, "coop/distance",1, true); 

  left_arm_error_pub_.init(node_handle, left_arm_id_+"_error", 1, true);
  right_arm_error_pub_.init(node_handle, right_arm_id_+"_error", 1, true);
  left_arm_error_pub_.msg_.header.frame_id = left_arm_id_ + "_link0";
  right_arm_error_pub_.msg_.header.frame_id = right_arm_id_ + "_link0";

  left_tau_d_pub_.init(node_handle, left_arm_id_+"_tau", 1, true);
  right_tau_d_pub_.init(node_handle, right_arm_id_+"_tau", 1, true);

  left_gravity_pub_.init(node_handle, left_arm_id_+"_gravity", 1, true);
  right_gravity_pub_.init(node_handle, right_arm_id_+"_gravity", 1, true);
  

  left_q_error_pub_.init(node_handle, left_arm_id_+"_q_error", 1, true);
  right_q_error_pub_.init(node_handle, right_arm_id_+"_q_error", 1, true);

  left_des_traj_pub_.init(node_handle, left_arm_id_+"_des_traj", 1, true); 
  right_des_traj_pub_.init(node_handle, right_arm_id_+"_des_traj", 1, true); 

  left_der_des_traj_pub_.init(node_handle, left_arm_id_+"_der_des_traj", 1, true); 
  right_der_des_traj_pub_.init(node_handle, right_arm_id_+"_der_des_traj", 1, true); 

  left_dder_des_traj_pub_.init(node_handle, left_arm_id_+"_dder_des_traj", 1, true); 
  right_dder_des_traj_pub_.init(node_handle, right_arm_id_+"_dder_des_traj", 1, true); 


  left_qd_ik_pub_.init(node_handle, left_arm_id_+"_qd_ik", 1, true);
  right_qd_ik_pub_.init(node_handle, right_arm_id_+"_qd_ik", 1, true);

  left_dqd_ik_pub_.init(node_handle, left_arm_id_+"_dqd_ik", 1, true);
  right_dqd_ik_pub_.init(node_handle, right_arm_id_+"_dqd_ik", 1, true);

  left_x_error_ik_pub_.init(node_handle, left_arm_id_+"_x_error_ik", 1, true);
  right_x_error_ik_pub_.init(node_handle, right_arm_id_+"_x_error_ik", 1, true);
  left_dx_error_ik_pub_.init(node_handle, left_arm_id_+"_dx_error_ik", 1, true);
  right_dx_error_ik_pub_.init(node_handle, right_arm_id_+"_dx_error_ik", 1, true);


  time_pub_.init(node_handle, "current_time", 1, true); 
  // Torque limits
  torque_limits_ << 87, 87, 87, 87, 12, 12, 12; 
  dtorque_limits_ << 1000, 1000, 1000, 1000, 1000, 1000, 1000; 
  perc_allowed_torque_limit_ = 0.8; 
  perc_allowed_dtorque_limit_ = 0.7; 
  // Joint limits 
  perc_allowed_q_limit_ = 0.9; 
  perc_allowed_dq_limit_ = 0.95; 
  q_min_limits_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973; 
  q_max_limits_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973; 
  dq_limits_<< 2.1750, 2.1750, 2.1750, 2.1750, 2.61, 2.61, 2.61; 

  // Initialization euler angles (2,1,0 for ZYX angles)
  eul_seq_.resize(3); 
  eul_seq_[0] = 2; eul_seq_[1] = 1; eul_seq_[2] = 0; 
  cout << "Euler sequence: " << eul_seq_[0] << eul_seq_[1] << eul_seq_[2] <<endl; 

  // Task space variables
  Eigen::Matrix<double,6, 12> Jsig_centroid, Jsig_formation; 
  Eigen::Matrix<double,6, 6> I_6;
  I_6.setIdentity();  
  Jsig_centroid << 0.5*I_6, 0.5*I_6; 
  Jsig_formation << -I_6, I_6; 
  Jsig_.block<6, 12>(0,0) = Jsig_centroid; 
  Jsig_.block<6, 12>(6,0) = Jsig_formation; 
  coop_motion_ = true; 
  coop_motion_ready_ = false; 
  coop_motion_satisfied_ = false; 

  pJsig_ = Jsig_.transpose()*(Jsig_* Jsig_.transpose()).inverse(); 
  cout << "Jsig   " << Jsig_<<endl; 
  cout << "pJsig_   " << pJsig_<<endl; 
  cout << "---------------------End init -----------"<<endl; 
  return left_success && right_success;
}

void DualArmDressingController::startingArm(CustomFrankaDataContainerDressing& arm_data){
 // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array = arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set target point to current state
  arm_data.position_d_ = initial_transform.translation();
  arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  arm_data.position_d_target_ = initial_transform.translation();
  arm_data.orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace target configuration to initial q
  arm_data.q_d_nullspace_ = q_initial;

  // Init pose   
  // Roll-pitch-yaw -> R(r, p, y) = R_z(y)R_y(p)R_x(r)  -> yaw = euler(0); pitch = euler(1); roll = euler(2)
  Eigen::Vector3d eul; 
  R2Eul(initial_transform.linear(), eul); 
  
  arm_data.eul_d_ = eul; 
  arm_data.previous_eul_ = eul; 
  arm_data.previous_eul_ik_ = eul; 
  arm_data.init_x_.head(3) = arm_data.position_d_; 
  arm_data.init_x_.tail(3) = eul; 
  arm_data.xtilde_.setZero(); 
  arm_data.initial_q_ = q_initial; 
  arm_data.qd_ = q_initial; 
  arm_data.dqd_.setZero(); 
  arm_data.ddqd_.setZero(); 
  arm_data.pos_ = arm_data.position_d_;
  cout << "Init ZYX: "<< eul.transpose() <<endl;
  // Init analytic jacobian
  analyticalJacobian(arm_data.previous_Ja_, jacobian, eul); 
  arm_data.previous_Ja_ik_ = arm_data.previous_Ja_; 
  arm_data.dx_ = arm_data.previous_Ja_ *arm_data.dq_filtered_;
  // init planners
  // arm_data.planner_.init(arm_data.position_d_, eul, current_time_); 
  arm_data.planner_.init(arm_data.position_d_,arm_data.dx_.head(3), eul,arm_data.dx_.tail(3), current_time_); 

  //arm_data.planner_q_.initQ(arm_data.initial_q_, current_time_);
  arm_data.planner_q_.initQ(arm_data.initial_q_,arm_data.dqd_, current_time_); 
  cout << "init q " << arm_data.initial_q_ << endl;

  arm_data.planner_q_.planQ(arm_data.initial_q_,arm_data.dqd_, arm_data.initial_des_q_, current_time_); 

  cout << "q desired " << arm_data.initial_des_q_ << endl;
  Eigen::VectorXd x_w(6); 
  transformCartesianPose(arm_data.init_x_, x_w, arm_data.w_T_0, eul); 
  x_coop_.segment(arm_data.ind_robot_*6, 6) = x_w; 
}

void DualArmDressingController::starting(const ros::Time& /*time*/) {

  for (auto& arm_data : arms_data_) {
    startingArm(arm_data.second);
  }
 
  current_time_ = 0;
  cout << "---- Time initialized -----" <<endl;  
  stop_command_ = false; 
  Eigen::VectorXd curr_sigma = Jsig_*x_coop_; 
  coop_planner_.init(curr_sigma, current_time_); 
  
}

void DualArmDressingController::update(const ros::Time& /*time*/,
                                                        const ros::Duration& period) {


  ros::Time starttime_upd = ros::Time::now(); 
  // Update arms
  for (auto& arm_data : arms_data_) {
    updateArm(arm_data.second, arm_data.first, period); 
  }

  sigma_ = Jsig_*x_coop_; 


  auto& left_arm_data = arms_data_.at(left_arm_id_);
  auto& right_arm_data = arms_data_.at(right_arm_id_);

  if (coop_motion_ && !coop_motion_ready_ && left_arm_data.conf_initialized_ && right_arm_data.conf_initialized_ ) { //&& !left_arm_data.learning_q_conf_requested_ && !right_arm_data.learning_q_conf_requested_){ 
    cout << "INITIALIZING COOPERATIVE PLANNER "<< endl; 
    coop_planner_.init(sigma_, current_time_); 
    coop_motion_ready_ = true; 
  }
 // ROS_INFO_THROTTLE(5, "Planner info: %d", coop_planner_.goalReached(sigma_,  current_time_));
  if ( (coop_motion_ && coop_motion_ready_ && coop_planner_.goalReached(sigma_,  current_time_)) || 
      (!coop_motion_ && left_arm_data.planner_.goalReached(left_arm_data.pos_, left_arm_data.previous_eul_, current_time_) && right_arm_data.planner_.goalReached(right_arm_data.pos_, right_arm_data.previous_eul_, current_time_)  )    ){ 
    coop_motion_satisfied_ = true;
  }else{
    coop_motion_satisfied_ = false; 
  }
  // Publishers
  ros::Time timestamp = ros::Time::now(); 
  if (rate_trigger_()){
    
    publishMotionFinished(coop_motion_finished_pub_); 

    // Publish EE error
    publishError(left_arm_error_pub_, left_arm_data.xtilde_,timestamp);
    publishError(right_arm_error_pub_, right_arm_data.xtilde_,timestamp);

    // Publish desired tau
    publishTau(left_tau_d_pub_, left_arm_data.tau_d_,timestamp);
    publishTau(right_tau_d_pub_, right_arm_data.tau_d_,timestamp);
    publishTau(left_gravity_pub_, left_arm_data.gravity_,timestamp);
    publishTau(right_gravity_pub_, right_arm_data.gravity_,timestamp);

    // Publish q error
    publishTau(left_q_error_pub_, left_arm_data.q_error_,timestamp); 
    publishTau(right_q_error_pub_, right_arm_data.q_error_,timestamp); 

    // Publish desired pose,  velocity and acceleration
    publishError(left_des_traj_pub_, left_arm_data.xd_,timestamp); 
    publishError(right_des_traj_pub_, right_arm_data.xd_,timestamp); 
    publishError(left_der_des_traj_pub_, left_arm_data.dxd_,timestamp); 
    publishError(right_der_des_traj_pub_, right_arm_data.dxd_,timestamp); 
    publishError(left_dder_des_traj_pub_, left_arm_data.ddxd_,timestamp); 
    publishError(right_dder_des_traj_pub_, right_arm_data.ddxd_,timestamp); 

    // Publish desired q and dq
    publishTau(left_qd_ik_pub_, left_arm_data.qd_,timestamp); 
    publishTau(left_qd_ik_pub_, right_arm_data.qd_,timestamp); 
    publishTau(left_dqd_ik_pub_, left_arm_data.dqd_,timestamp); 
    publishTau(right_dqd_ik_pub_, right_arm_data.dqd_,timestamp); 

    // Publish ik error
    publishError(left_x_error_ik_pub_, left_arm_data.xtilde_ik_,timestamp); 
    publishError(right_x_error_ik_pub_, right_arm_data.xtilde_ik_,timestamp); 
    publishError(left_dx_error_ik_pub_, left_arm_data.dxtilde_ik_,timestamp); 
    publishError(right_dx_error_ik_pub_, right_arm_data.dxtilde_ik_,timestamp); 

    publishTime(time_pub_, current_time_); 
    publishCenteringPose(timestamp);
    publishFormationDistance();
  }
  ros::Time endtime_upd = ros::Time::now(); 
  ros::Duration difftime = endtime_upd-starttime_upd; 
  current_time_ = current_time_ + period.toSec(); 
  //cout << difftime.toSec()<<endl; 
}

void DualArmDressingController::updateArm(CustomFrankaDataContainerDressing& arm_data, const string& arm_id, const ros::Duration& period){
  // get state variables
  franka::RobotState robot_state = arm_data.state_handle_->getRobotState();
  std::array<double, 49> inertia_array = arm_data.model_handle_->getMass();
  std::array<double, 7> coriolis_array = arm_data.model_handle_->getCoriolis();
  std::array<double, 7> gravity_array = arm_data.model_handle_->getGravity();
  std::array<double, 42> jacobian_array = arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  
  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> inertia(inertia_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d( robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());  
  Eigen::Quaterniond orientation(transform.linear());
 
  arm_data.gravity_ = gravity; 
  arm_data.pos_ = position; 

  // Check position and velocity limits 
  bool check_joint = checkJointLimits(q, arm_id); 
  bool check_velocity = checkVelocityJointLimits(dq, arm_id); 
  if (!check_joint || !check_velocity) stop_command_ = true; 

  // ----------- Impedance controller -----------------
  // Get tranformation matrix from EE to K frame
  Eigen::Affine3d T_EE_K(Eigen::Matrix4d::Map(robot_state.EE_T_K.data())); 

 // Get Euler angles and check continuity
  Eigen::Vector3d eul_angles; // = orientation.toRotationMatrix().eulerAngles(eul_seq_[0], eul_seq_[1], eul_seq_[2]);  
  R2Eul(transform.linear(), eul_angles); 
  getContinuousEulerAngles(eul_angles, arm_data.previous_eul_ ); 
  arm_data.previous_eul_ = eul_angles;

  // Wrench in base frame
  Eigen::Map<Eigen::Matrix<double, 6, 1>> K_F_ext_hat_K(robot_state.K_F_ext_hat_K.data()); // wrenches estimation in K frame
  Eigen::Affine3d T_0_K = transform*T_EE_K; 
  Eigen::VectorXd F_ext_hat_0(6); // wrenches estimation in K frame
  F_ext_hat_0.head(3) = T_0_K.linear()*K_F_ext_hat_K.head(3); 
  F_ext_hat_0.tail(3) = T_0_K.linear()*K_F_ext_hat_K.tail(3);

  // Compute analytic jacobian derivative 
  Eigen::Matrix<double, 6, 7> Ja, dJa; 
  analyticalJacobian(Ja, jacobian, eul_angles);  
  double sampling_time = period.toSec(); 
  dJa = (Ja-arm_data.previous_Ja_)/sampling_time; 
  arm_data.previous_Ja_ = Ja; 

  // Compute pseudo inverse of the analytic jacobian 
  Eigen::MatrixXd pJa; 
  //franka_example_controllers::pseudoInverse(Ja, pJa); 
  pJa = Ja.transpose()*(Ja*Ja.transpose()).inverse(); 

  // Get desired trajectory
  Eigen::VectorXd xd(6), dxd(6), ddxd(6); 
  /*
  if (arm_data.static_traj_ == 0){
    Eigen::Matrix<double, 6, 1> xd2, dxd2, ddxd2; 
    getSinTrajectory(current_time_, arm_data.init_x_, xd2, dxd2, ddxd2); 
    xd = xd2; 
    dxd = dxd2; 
    ddxd = ddxd2; 
  }else{ 
    arm_data.planner_.getPose(current_time_, xd, dxd, ddxd); 
  }*/
  if (coop_motion_ && coop_motion_ready_){
    // If coop_motion the task space trajectory is computed 
    Eigen::VectorXd sigmad(12), dsigmad(12), ddsigmad(12);
    coop_planner_.getPose(current_time_, sigmad, dsigmad, ddsigmad); 
    Eigen::VectorXd xd_w(6), dxd_w(6), ddxd_w(6); 
    xd_w = arm_data.Gamma_i_ * pJsig_ * sigmad; 
    dxd_w = arm_data.Gamma_i_ * pJsig_ * dsigmad; 
    ddxd_w = arm_data.Gamma_i_ * pJsig_ * ddsigmad; 

    transformCartesianPose(xd_w, xd, arm_data.f0_T_w, eul_angles ); 

    // TO CHANGE!!! in case of R0_w != I_3 the rotation must be included
    dxd = dxd_w; 
    ddxd = ddxd_w; 
   // cout << "ROBOT "<< arm_data.ind_robot_<< "coop planner"<<endl; 

 /* Eigen::VectorXd temp(6); 
  temp.head(3) = position; 
  temp.tail(3) = eul_angles; 
  cout << "xd " <<xd.transpose() <<endl; 
  cout << "x  "<< temp.transpose() <<endl; 
*/
// TEST 
 //   arm_data.planner_.getPose(current_time_, xd, dxd, ddxd); 

  }else{
   // cout << "ROBOT "<< arm_data.ind_robot_<< "cartesian planner"<<endl; 
    arm_data.planner_.getPose(current_time_, xd, dxd, ddxd); 
  }
  arm_data.xd_ = xd; 
  arm_data.dxd_ = dxd; 
  arm_data.ddxd_ = ddxd; 
   // Compute velocity error 
  Eigen::VectorXd dxtilde(6);
  dxtilde = dxd - Ja*dq;
  arm_data.dxtilde_ = dxtilde; 

  // Compute pose error 
  Eigen::VectorXd xtilde(6);
  Eigen::VectorXd x(6);
  x.head(3) = position; 
  x.tail(3) = eul_angles; 
  xtilde = xd - x;  
  arm_data.xtilde_ = xtilde; 

  Eigen::VectorXd x_w(6); 
  transformCartesianPose(x, x_w, arm_data.w_T_0, eul_angles); 
  x_coop_.segment(arm_data.ind_robot_*6, 6) = x_w; 

  Eigen::Matrix<double, 6, 6> Ta; 
  transfAnalyticalJacobian(Ta, eul_angles); 
  Eigen::VectorXd tau_task(7), y(7); 
  // TO CHANGE!! non considero wrench
  F_ext_hat_0 = - 0*F_ext_hat_0; // in this way we consider the forces that the robot exerts on the environment
  
  //cout << "Iner:"<< inertia << endl; 
  //cout << "Norm: " << inertia.norm() << endl; 
/*
  y = pJa*arm_data.Md_.inverse()*(arm_data.Md_*ddxd + arm_data.Dd_*dxtilde + arm_data.Kd_*xtilde - arm_data.Md_*dJa*dq - Ta.transpose()*F_ext_hat_0); 
  Eigen::Matrix<double, 7, 1> pr1 = pJa*arm_data.Md_.inverse()*arm_data.Kd_*xtilde; 
  Eigen::Matrix<double, 7, 1> pr2 = inertia*pr1; 
  Eigen::Matrix<double, 6, 6> inertiaOp = (Ja*inertia.inverse()*Ja.transpose()).inverse(); 
  //cout << "I="<<endl <<inertiaOp<<endl;  
 // cout << "Pr1 " << pr1.transpose()<<endl;  
 // cout << "Pr2 " << pr2.transpose()<<endl; 
  tau_task << inertia*y + coriolis + jacobian.transpose()*F_ext_hat_0; 
 
  // Saturate torque rate to avoid discontinuities
  tau_task << saturateTorqueRate(arm_data, tau_task, tau_J_d);
*/

// PD control
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - xd.head(3);

  // orientation error
  if (arm_data.orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * arm_data.orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_example_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  if (arm_data.conf_initialized_ && !arm_data.learning_q_conf_requested_ && !arm_data.initial_q_conf_requested_){
    // If the initial joint space configuration has been reached, the cartesian space motion is performed
    std::array<double, 7> qd_array; 
    for (int i = 0; i< 7; i++){
      qd_array[i] = arm_data.qd_[i]; 
    }
    std::array<double, 16> array_0_EE_ik = arm_data.model_handle_->getPose( franka::Frame::kEndEffector, qd_array, robot_state.F_T_EE, robot_state.EE_T_K ); 
    Eigen::Affine3d T_0_EE_ik(Eigen::Matrix4d::Map(array_0_EE_ik.data())); 
    
    //
    Eigen::Vector3d eul_angles_ik; // = T_0_EE_ik.linear().eulerAngles(eul_seq_[0], eul_seq_[1], eul_seq_[2]);
    R2Eul(T_0_EE_ik.linear(), eul_angles_ik);   
    getContinuousEulerAngles(eul_angles_ik, arm_data.previous_eul_ik_ ); 
    arm_data.previous_eul_ik_ = eul_angles_ik;
    Eigen::VectorXd x_ik(6); 
    x_ik.head(3) = T_0_EE_ik.translation(); 
    x_ik.tail(3) = eul_angles_ik; 

    // Jacobian matrix
    std::array<double, 42> jacobian_ik_array = arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector, qd_array , robot_state.F_T_EE, robot_state.EE_T_K );
    Eigen::Map<Eigen::Matrix<double, 6, 7>>  jacobian_ik(jacobian_ik_array.data()); 
    Eigen::Matrix<double, 6, 7> Ja_ik; 
    analyticalJacobian(Ja_ik, jacobian_ik, eul_angles_ik);  
    Eigen::Matrix<double, 6, 7> dJa_ik; 
    dJa_ik = (Ja_ik - arm_data.previous_Ja_ik_)/period.toSec(); 
    arm_data.previous_Ja_ik_ = Ja_ik; 
    Eigen::MatrixXd pJa_ik = Ja_ik.transpose()*(Ja_ik*Ja_ik.transpose()).inverse(); 

    arm_data.xtilde_ik_ = (xd-x_ik); 
    arm_data.dxtilde_ik_ = (dxd-Ja_ik*arm_data.dqd_); 

    // second order CLIK
    Eigen::MatrixXd I_n(7,7); 
    I_n.setIdentity(); 
    arm_data.ddqd_ = pJa_ik*(ddxd + arm_data.Kd_ik_*(dxd - Ja_ik*arm_data.dqd_) + arm_data.Kp_ik_*(xd-x_ik) - dJa_ik*arm_data.dqd_)+(I_n-pJa_ik*Ja_ik)*(-2*arm_data.dqd_); 

    // Integration with forward euler
    arm_data.qd_ = arm_data.qd_ + arm_data.dqd_*period.toSec(); 
    arm_data.dqd_ = arm_data.dqd_ +arm_data.ddqd_ *period.toSec(); 
  }
  //Joint space motion on request
  else if(arm_data.learning_q_conf_requested_) {
    Eigen::VectorXd des_q(7),des_dq(7), des_ddq(7); 
    // arm_data.planner_q_.getPoseQ(current_time_, arm_data.qd_, arm_data.dqd_, arm_data.ddqd_);
      arm_data.planner_q_.getPoseQ(current_time_, des_q, des_dq, des_ddq);
      arm_data.qd_ = des_q; 
      arm_data.dqd_ = des_dq; 
      arm_data.ddqd_ = des_ddq; 
      if (arm_data.planner_q_.goalReachedQ(q, current_time_)){
        cout << arm_id <<" learning config reached"<< endl; 
        // Initialization of the cartesian space motion  
        Eigen::Vector3d eul; 
        R2Eul(transform.linear(), eul); 
        arm_data.eul_d_ = eul; 
        arm_data.previous_eul_ = eul; 
        arm_data.previous_eul_ik_ = eul; 
        arm_data.position_d_ = position; 
        arm_data.init_x_.head(3) = arm_data.position_d_; 
        arm_data.init_x_.tail(3) = eul; 
        
        // Init analytic jacobian
        analyticalJacobian(arm_data.previous_Ja_, jacobian, eul); 
        arm_data.previous_Ja_ik_ = arm_data.previous_Ja_; 
        // init planners
        //arm_data.planner_.init(arm_data.position_d_, eul, current_time_); 
        arm_data.dx_ = arm_data.previous_Ja_ *arm_data.dq_filtered_;
        arm_data.planner_.init(arm_data.position_d_,arm_data.dx_.head(3), eul,arm_data.dx_.tail(3), current_time_); 

        arm_data.learning_q_conf_requested_ = false; 
        coop_motion_ = false; 

      }
  }
    else if(arm_data.initial_q_conf_requested_) {
    Eigen::VectorXd des_q(7),des_dq(7), des_ddq(7);
    // arm_data.planner_q_.getPoseQ(current_time_, arm_data.qd_, arm_data.dqd_, arm_data.ddqd_);
      arm_data.planner_q_.getPoseQ(current_time_, des_q, des_dq, des_ddq);
      arm_data.qd_ = des_q;
      arm_data.dqd_ = des_dq;
      arm_data.ddqd_ = des_ddq;
      if (arm_data.planner_q_.goalReachedQ(q, current_time_)){
        cout << arm_id <<"init config reached?"<< endl;
        // Initialization of the cartesian space motion
        Eigen::Vector3d eul;
        R2Eul(transform.linear(), eul);
        arm_data.eul_d_ = eul;
        arm_data.previous_eul_ = eul;
        arm_data.previous_eul_ik_ = eul;
        arm_data.position_d_ = position;
        arm_data.init_x_.head(3) = arm_data.position_d_;
        arm_data.init_x_.tail(3) = eul;

        // Init analytic jacobian
        analyticalJacobian(arm_data.previous_Ja_, jacobian, eul);
        arm_data.previous_Ja_ik_ = arm_data.previous_Ja_;
        // init planners
        //arm_data.planner_.init(arm_data.position_d_, eul, current_time_);
        arm_data.dx_ = arm_data.previous_Ja_ *arm_data.dq_filtered_;
        arm_data.planner_.init(arm_data.position_d_,arm_data.dx_.head(3), eul,arm_data.dx_.tail(3), current_time_); 

        arm_data.initial_q_conf_requested_ = false;
        coop_motion_ = false;

      }
  }


  //Joint space motion (should be moved seprate function)
  else{
    // Initial joint space motion
    Eigen::VectorXd des_q(7),des_dq(7), des_ddq(7); 
   // arm_data.planner_q_.getPoseQ(current_time_, arm_data.qd_, arm_data.dqd_, arm_data.ddqd_);
    arm_data.planner_q_.getPoseQ(current_time_, des_q, des_dq, des_ddq);
    arm_data.qd_ = des_q; 
    arm_data.dqd_ = des_dq; 
    arm_data.ddqd_ = des_ddq; 
    if (arm_data.planner_q_.goalReachedQ(q, current_time_)){
      cout << arm_id <<" inital config reached"<< endl; 
      // Initialization of the cartesian space motion  
      Eigen::Vector3d eul; 
      R2Eul(transform.linear(), eul); 
      arm_data.eul_d_ = eul; 
      arm_data.previous_eul_ = eul; 
      arm_data.previous_eul_ik_ = eul; 
      arm_data.position_d_ = position; 
      arm_data.init_x_.head(3) = arm_data.position_d_; 
      arm_data.init_x_.tail(3) = eul; 
      
      // Init analytic jacobian
      analyticalJacobian(arm_data.previous_Ja_, jacobian, eul); 
      arm_data.previous_Ja_ik_ = arm_data.previous_Ja_; 
      // init planners
      //arm_data.planner_.init(arm_data.position_d_, eul, current_time_); 
      arm_data.dx_ = arm_data.previous_Ja_ *arm_data.dq_filtered_;
      arm_data.planner_.init(arm_data.position_d_,arm_data.dx_.head(3), eul,arm_data.dx_.tail(3), current_time_); 

      arm_data.conf_initialized_ = true; 
    }
  }
  //cout << "qd "<< arm_data.qd_.transpose() << endl; 
 // cout << "q "<< q.transpose() << endl; 
  //cout << "dqd "<< arm_data.dqd_.transpose() << endl; 
  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    arm_data.dq_filtered_[i] = (1 - alpha) * arm_data.dq_filtered_[i] + alpha * robot_state.dq[i];
  }
  tau_task << coriolis + inertia*arm_data.ddqd_ + arm_data.Kd_q_ * (arm_data.qd_ - q) +  arm_data.Dd_q_ *( arm_data.dqd_ - arm_data.dq_filtered_);
  //tau_task << coriolis + jacobian.transpose() * (-arm_data.Kd_ * error - arm_data.Dd_ * (jacobian * dq));
  tau_task << saturateTorqueRate(arm_data, tau_task, tau_J_d);
  arm_data.q_error_ = arm_data.qd_ - q; 

 // tau_task.setZero(); 
//  double sin_per = 10.0; 
//  double sin_w = 2*M_PI/sin_per; 
//  tau_task(4) = 1*sin(sin_w*current_time_); 
//  tau_task(6) = 8; 


  // Check torque commands 
  bool check_torque = checkTorqueLimits(tau_task, arm_id); 
  if (!check_torque) stop_command_ = true;

  arm_data.tau_d_ = tau_task; 
  if (arm_data.impedance_ && !stop_command_ )
   for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles_[i].setCommand(tau_task(i));
  }else{
    for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles_[i].setCommand(0);
    }
  }

  
  arm_data.position_d_ = arm_data.filter_params_ * arm_data.position_d_target_ +
                         (1.0 - arm_data.filter_params_) * arm_data.position_d_;
  arm_data.orientation_d_ =
      arm_data.orientation_d_.slerp(arm_data.filter_params_, arm_data.orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> DualArmDressingController::saturateTorqueRate(
    const CustomFrankaDataContainerDressing& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}
void DualArmDressingController::getDesiredTrajectory(bool stationary, double time, const Eigen::Matrix<double, 6, 1>& init_x, Eigen::Matrix<double, 6, 1>& xd, Eigen::Matrix<double, 6, 1>& dxd, Eigen::Matrix<double, 6, 1>& ddxd){
  if (stationary){
    getStationaryTrajectory(time, init_x, xd, dxd, ddxd); 
  }else{
    getSinTrajectory(time, init_x, xd, dxd, ddxd); 
  }
}

void DualArmDressingController::getStationaryTrajectory(double time, const Eigen::Matrix<double, 6, 1>& init_x, Eigen::Matrix<double, 6, 1>& xd, Eigen::Matrix<double, 6, 1>& dxd, Eigen::Matrix<double, 6, 1>& ddxd){
  xd = init_x; 
  dxd.setZero(); 
  ddxd.setZero(); 
}

void DualArmDressingController::getSinTrajectory(double time, const Eigen::Matrix<double, 6, 1>& init_x, Eigen::Matrix<double, 6, 1>& xd, Eigen::Matrix<double, 6, 1>& dxd, Eigen::Matrix<double, 6, 1>& ddxd){
  double period = 7.0; 
  double w = 2*M_PI/period; 
  double A = 0.1; 
  int ind = 1; 
  xd = init_x; 
  dxd.setZero(); 
  ddxd.setZero();
  xd(ind) = init_x(ind) + A*(1-cos(w*time)); 
  dxd(ind) = A*w*sin(w*time); 
  ddxd(ind) = A*w*w*cos(w*time); 
}


// void DualArmDressingController::coopCentroidCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  
//   auto& left_arm_data = arms_data_.at(left_arm_id_);
//   auto& right_arm_data = arms_data_.at(right_arm_id_);
//   if (!coop_motion_ready_ && left_arm_data.conf_initialized_ && right_arm_data.conf_initialized_){ 
//     coop_planner_.init(sigma_, current_time_); 
//     coop_motion_ready_ = true; 
//   }
//   // Data in the message is considered as a displacement
//   if (coop_motion_ready_){
//     Eigen::Matrix<double, 12, 1> sigmad = sigma_; 
//     sigmad[0] = sigmad[0] + msg->pose.position.x; 
//     sigmad[1] = sigmad[1] + msg->pose.position.y; 
//     sigmad[2] = sigmad[2] + msg->pose.position.z; 
//     sigmad[3] = sigmad[3] + msg->pose.orientation.x; 
//     sigmad[4] = sigmad[4] + msg->pose.orientation.y; 
//     sigmad[5] = sigmad[5] + msg->pose.orientation.z; 


// /*    Eigen::MatrixXd R_current(3,3); 
//     Eul2R(R_current,sigmad.segment(3,3)); 

// */

//     Eigen::Vector3d displ_angles; 
//     displ_angles << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z; 
//     Eigen::MatrixXd R_d(3,3); 
//     Eul2R(R_d,displ_angles); 
//     sigmad.segment(6,3) = R_d*sigmad.segment(6,3); 
//     cout << "CENTROID CALLBACK: Planning from sigma = " << endl << sigma_.transpose() << endl << " to sigmad = " << endl << sigmad.transpose() << endl; 
//     coop_planner_.plan(sigma_, sigmad, current_time_); 
//   }

//   // Enable the cooperative motion
//   coop_motion_ = true; 

// }

void DualArmDressingController::coopCentroidFormCallback(const CoopCentroidFormationDelta::ConstPtr& msg){
  
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  if (!coop_motion_ready_ && left_arm_data.conf_initialized_ && right_arm_data.conf_initialized_ && !left_arm_data.learning_q_conf_requested_ && !right_arm_data.learning_q_conf_requested_){ 
    cout << "-------------------------------------------   INIT CALLBACK ---------------------------------------"<< endl; 
    coop_planner_.init(sigma_, current_time_); 
    coop_motion_ready_ = true; 
  }
  // Data in the message is considered as a displacement
  if (coop_motion_ready_){
    Eigen::Matrix<double, 12, 1> sigmad = sigma_; 
    sigmad[0] = sigmad[0] + msg->position_dx; 
    sigmad[1] = sigmad[1] + msg->position_dy; 
    sigmad[2] = sigmad[2] + msg->position_dz; 
    sigmad[3] = sigmad[3] + msg->orientation_droll; 
    sigmad[4] = sigmad[4] + msg->orientation_dpitch; 
    sigmad[5] = sigmad[5] + msg->orientation_dyaw; 

    Eigen::Vector3d displ_angles; 
    displ_angles << msg->orientation_droll, msg->orientation_dpitch, msg->orientation_dyaw; 
    Eigen::MatrixXd R_d(3,3); 
    Eul2R(R_d,displ_angles); 
    sigmad.segment(6,3) = R_d*sigmad.segment(6,3); 

    double distance_increment = msg->formation_ddistance; 
    double current_distance = sigma_.segment(6,3).norm(); 
    double desired_distance = current_distance + distance_increment; 
    Eigen::Vector3d desired_formation = desired_distance * sigmad.segment(6,3)/current_distance; 
    sigmad.segment(6,3) = desired_formation; 

    cout << "CENTROID CALLBACK: Planning from sigma = " << endl << sigma_.transpose() << endl << " to sigmad = " << endl << sigmad.transpose() << endl; 
    coop_planner_.plan(sigma_, sigmad, current_time_); 
  }

  // Enable the cooperative motion
  coop_motion_ = true; 

}


// void DualArmDressingController::coopFormationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
//   auto& left_arm_data = arms_data_.at(left_arm_id_);
//   auto& right_arm_data = arms_data_.at(right_arm_id_);
//   if (!coop_motion_ready_ && left_arm_data.conf_initialized_ && right_arm_data.conf_initialized_){ 
//     coop_planner_.init(sigma_, current_time_); 
//     coop_motion_ready_ = true; 
//   }
//   // Data in the message is considered as a displacement
//   if (coop_motion_ready_){
//     Eigen::Matrix<double, 12, 1> sigmad = sigma_; 

//     sigmad[6] = sigmad[6] + msg->pose.position.x; 
//     sigmad[7] = sigmad[7] + msg->pose.position.y; 
//     sigmad[8] = sigmad[8] + msg->pose.position.z; 
//     sigmad[9] = sigmad[9] + msg->pose.orientation.x; 
//     sigmad[10] = sigmad[10] + msg->pose.orientation.y; 
//     sigmad[11] = sigmad[11] + msg->pose.orientation.z; 
//     cout << "CENTROID CALLBACK: Planning from sigma = " << endl << sigma_.transpose() << endl << " to sigmad = " << endl << sigmad.transpose() << endl; 
//     coop_planner_.plan(sigma_, sigmad, current_time_); 
//   }

//   // Enable the cooperative motion
//   coop_motion_ = true; 

// }





// void DualArmDressingController::coopDistanceFormationCallback(const std_msgs::Float64::ConstPtr& msg){
//   double distance_increment = msg->data; 
//   auto& left_arm_data = arms_data_.at(left_arm_id_);
//   auto& right_arm_data = arms_data_.at(right_arm_id_);
//   if (!coop_motion_ready_ && left_arm_data.conf_initialized_ && right_arm_data.conf_initialized_){ 
//     coop_planner_.init(sigma_, current_time_); 
//     coop_motion_ready_ = true; 
//   }
//   // Data in the message is considered as a displacement
//   if (coop_motion_ready_){
//     Eigen::Matrix<double, 12, 1> sigmad = sigma_; 
//     // Change the position of the formation
//     double current_distance = sigma_.segment(6,3).norm(); 
//     double desired_distance = current_distance + distance_increment; 
//     Eigen::Vector3d desired_formation = desired_distance * sigma_.segment(6,3)/current_distance; 
//     sigmad.segment(6,3) = desired_formation; 
//     /*sigmad[6] = sigmad[6] + msg->pose.position.x; 
//     sigmad[7] = sigmad[7] + msg->pose.position.y; 
//     sigmad[8] = sigmad[8] + msg->pose.position.z; */
//     cout << "CENTROID CALLBACK: Planning from sigma = " << endl << sigma_.transpose() << endl << " to sigmad = " << endl << sigmad.transpose() << endl; 
//     coop_planner_.plan(sigma_, sigmad, current_time_); 
//   }

//   // Enable the cooperative motion
//   coop_motion_ = true; 

// }

void DualArmDressingController::requestLearningPoseCallback(const std_msgs::Bool::ConstPtr& msg){

  if(msg->data){
    cout << "PLANNING q MOTION FOR ROBOT learning poses "  <<endl; 
    // Get current configuration 
    //
    for(int robot_id=0; robot_id<2;robot_id++){
      
      
      std::string robot_arm_id_;

      if (robot_id ==0)
        robot_arm_id_ = left_arm_id_;

      if (robot_id ==1)
        robot_arm_id_ = right_arm_id_;

      cout << "PLANNING  " << robot_arm_id_   <<endl; 
      auto& arm_data = arms_data_.at(robot_arm_id_);

    franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
      // get jacobian
      std::array<double, 42> jacobian_array = arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
      // convert to eigen
      //Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      //Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
      Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
      //Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

      // set target point to current state
      //arm_data.position_d_ = initial_transform.translation();
      //arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
      //arm_data.position_d_target_ = initial_transform.translation();
      //arm_data.orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

      // set nullspace target configuration to initial q
      //arm_data.q_d_nullspace_ = q_initial;

      // Init pose   
      // Roll-pitch-yaw -> R(r, p, y) = R_z(y)R_y(p)R_x(r)  -> yaw = euler(0); pitch = euler(1); roll = euler(2)
      //Eigen::Vector3d eul; 
      //R2Eul(initial_transform.linear(), eul); 
      
      //arm_data.eul_d_ = eul; 
      //arm_data.previous_eul_ = eul; 
      //arm_data.previous_eul_ik_ = eul; 
      //arm_data.init_x_.head(3) = arm_data.position_d_; 
      //arm_data.init_x_.tail(3) = eul; 
      //arm_data.xtilde_.setZero(); 
      arm_data.initial_q_ = q_initial; 
      //arm_data.qd_ = q_initial; 
      //arm_data.dqd_.setZero(); 
      //arm_data.ddqd_.setZero(); 
      //arm_data.pos_ = arm_data.position_d_;
      //cout << "Init ZYX: "<< eul.transpose() <<endl;
      // Init analytic jacobian
      //analyticalJacobian(arm_data.previous_Ja_, jacobian, eul); 
      //arm_data.previous_Ja_ik_ = arm_data.previous_Ja_; 
      // init planners
      //arm_data.planner_.init(arm_data.position_d_, eul, current_time_); 

      //arm_data.planner_q_.initQ(arm_data.initial_q_, current_time_); 
      arm_data.planner_q_.initQ(arm_data.initial_q_,arm_data.dqd_, current_time_); 
      //arm_data.planner_q_.planQ(arm_data.initial_q_,arm_data.learning_pose_des_q_, current_time_); 
      arm_data.planner_q_.planQ(arm_data.initial_q_,arm_data.dqd_, arm_data.initial_des_q_, current_time_); 

      //Eigen::VectorXd x_w(6); 
      //transformCartesianPose(arm_data.init_x_, x_w, arm_data.w_T_0, eul); 
      //x_coop_.segment(arm_data.ind_robot_*6, 6) = x_w;

      arm_data.learning_q_conf_requested_ = true; 

    
    }

  }
}




// trying to get franka to init pose
void DualArmDressingController::requestInitPoseCallback(const std_msgs::Bool::ConstPtr& msg){

  if(msg->data){
    cout << "PLANNING q MOTION FOR ROBOT initial poses "  <<endl;
    // Get current configuration
    for(int robot_id=0; robot_id<2;robot_id++){
      std::string robot_arm_id_;
      if (robot_id ==0)
        robot_arm_id_ = left_arm_id_;

      if (robot_id ==1)
        robot_arm_id_ = right_arm_id_;

      cout << "PLANNING  " << robot_arm_id_ <<endl;
      auto& arm_data = arms_data_.at(robot_arm_id_);
      franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
      // get jacobian
      std::array<double, 42> jacobian_array = arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
      // convert to eigen
      Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
      arm_data.initial_q_ = q_initial;

      //arm_data.planner_q_.initQ(arm_data.initial_q_, current_time_);
      arm_data.planner_q_.initQ(arm_data.initial_q_,arm_data.dqd_, current_time_); 
      //arm_data.planner_q_.planQ(arm_data.initial_q_,arm_data.initial_des_q_, current_time_);
      arm_data.planner_q_.planQ(arm_data.initial_q_,arm_data.dqd_, arm_data.initial_des_q_, current_time_); 
      arm_data.initial_q_conf_requested_ = true;
    }

  }
}


void DualArmDressingController::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const string& robot_id){
  cout << "PLANNING MOTION FOR ROBOT " <<   robot_id <<endl; 
  // Get current configuration 
  auto& arm_data = arms_data_.at(robot_id);
  franka::RobotState robot_state = arm_data.state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d curr_pos(transform.translation());  
  Eigen::Quaterniond curr_quat(transform.linear());
  Eigen::Vector3d eul_angles; // = curr_quat.toRotationMatrix().eulerAngles(eul_seq_[0], eul_seq_[1], eul_seq_[2]);  
  R2Eul(transform.linear(), eul_angles); 
  getContinuousEulerAngles(eul_angles, arm_data.previous_eul_ ); 

  // Define desired configuration in link0 frame 
  Eigen::Vector3d p_w_d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z); 
  Eigen::Quaterniond o_w_d( msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z); 
  Eigen::Affine3d A_w_d; 
  Eigen::Matrix3d R = o_w_d.normalized().toRotationMatrix(); 
  A_w_d.linear() = R; 
  A_w_d.translation() = p_w_d;  
  Eigen::Affine3d A_0_w = arm_data.f0_T_w; 
  Eigen::Affine3d A_0_d = A_0_w*A_w_d; 
  Eigen::Vector3d p_0_d = A_0_d.translation();
  Eigen::Matrix3d R_0_d = A_0_d.linear();

  cout << "CALLBACK, des quat (world frame) " << o_w_d.x() << " " << o_w_d.y() << " " << o_w_d.z() << " " <<o_w_d.w()<<endl; 
  cout << "CALLBACK, des R (world frame) " << endl; 
  cout << R << endl; 
  
  Eigen::Vector3d eul_angles_d; 
  R2Eul(R_0_d, eul_angles_d); 
  getContinuousEulerAngles(eul_angles_d, eul_angles ); 
  //arm_data.planner_.plan(curr_pos, eul_angles, p_0_d, eul_angles_d, current_time_); 
  arm_data.dx_ = arm_data.previous_Ja_ *arm_data.dq_filtered_;
  arm_data.planner_.plan(curr_pos, arm_data.dx_.head(3), eul_angles,arm_data.dx_.tail(3), p_0_d, eul_angles_d, current_time_); 

  if (coop_motion_){
    // If the robots are in cooperative motion, the cartesian planner of the other robot must be initialized as well
    string other_robot_id = left_arm_id_; 
    if (robot_id.compare(left_arm_id_) == 0){
      other_robot_id = right_arm_id_; 
    }
    cout << "Resetting coop motion: init MOTION FOR ROBOT " <<   other_robot_id <<endl; 
    auto& other_arm_data = arms_data_.at(other_robot_id);
    //other_arm_data.planner_.init( other_arm_data.pos_, other_arm_data.previous_eul_, current_time_); 
    other_arm_data.dx_ = other_arm_data.previous_Ja_ *other_arm_data.dq_filtered_;
    other_arm_data.planner_.init(other_arm_data.position_d_,other_arm_data.dx_.head(3), other_arm_data.previous_eul_,other_arm_data.dx_.tail(3), current_time_); 
    coop_motion_ = false; 
  }
  cout << robot_id << " Desired position: "<<p_0_d.transpose() << " Current position: " << curr_pos.transpose()<<endl; 
  cout << robot_id << " Desired orientation: "<<eul_angles_d.transpose()<< " Current orientation: "<< eul_angles.transpose()<< endl; 

}

void DualArmDressingController::analyticalJacobian(Eigen::Matrix<double, 6, 7>& Ja, const Eigen::Matrix<double, 6, 7>& J, const Eigen::Vector3d& eul_angles){
    //cout << "J: "<< J <<endl; 
    Eigen::Matrix<double, 6, 6> Ta; 
    //Ta.setIdentity(); 
    transfAnalyticalJacobian(Ta, eul_angles); 
    Ja = Ta.inverse()*J; 
    //cout << "Ja: "<< Ja <<endl; 
 }

 void DualArmDressingController::transfAnalyticalJacobian(Eigen::Matrix<double, 6, 6>& Ta, const Eigen::Vector3d& ZYX){
   if (eul_seq_[0]!= 2 || eul_seq_[1]!= 1 || eul_seq_[2]!= 0){
     ROS_WARN("ZYX euler angles must be considered"); 
   }
    Eigen::Matrix<double, 3, 3> T; 
    T << 0, -sin(ZYX[0]), cos(ZYX[0])*cos(ZYX[1]), 0, cos(ZYX[0]), sin(ZYX[0])*cos(ZYX[1]), 1, 0, -sin(ZYX[1]); 
    Ta = Eigen::Matrix<double, 6, 6>::Identity(); 
    Ta.block<3,3>(3,3) = T;  
    //cout << "Ta: " << endl << Ta <<endl; 
 }



  void DualArmDressingController::transformCartesianPose(const Eigen::VectorXd& xd_w, Eigen::VectorXd& xd_0, const Eigen::Affine3d& A_0_w, const Eigen::VectorXd& current_eul_0){
    Eigen::MatrixXd R_w(3,3); 
    Eul2R(R_w,xd_w.tail(3)); 

    Eigen::Affine3d A_w_d; 
    A_w_d.linear() = R_w; 
    A_w_d.translation() = xd_w.head(3); 

    Eigen::Affine3d A_0_d = A_0_w*A_w_d; 
    xd_0.head(3) = A_0_d.translation();
    Eigen::Vector3d eul_0_d; 
    R2Eul(A_0_d.linear(), eul_0_d); 
    getContinuousEulerAngles(eul_0_d, current_eul_0 ); 
    xd_0.tail(3) = eul_0_d; 
  }

  void DualArmDressingController::getContinuousEulerAngles(Eigen::Matrix<double, 3, 1>& eul, const Eigen::Matrix<double, 3, 1>& previous_eul ){
    double theta_mod = fmod(eul[1], 2*M_PI); 
    if (eul_seq_[0] == 2 && eul_seq_[1] == 1 && eul_seq_[2] == 0 && ( abs( theta_mod - M_PI/2) < 0.1 ||  abs( theta_mod - 3*M_PI/2) < 0.1 )){
        ROS_WARN("Close to euler representation singularity!"); 
    } 
    for (int i = 0; i< 3; i++){
      if ( abs(eul[i] - previous_eul[i]) >= M_PI){
			  eul[i] = eul[i] + sign(previous_eul[i])*2*M_PI; 
		  }
    }
  }

double DualArmDressingController::sign(double n){
	if (n > 0){
		return 1; 
	}else if (n < 0){
		return -1; 
	}else{
		return 0; 
	}	
}



// UTILS FUNCTION 

void DualArmDressingController::R2Eul(const Eigen::MatrixXd& R, Eigen::Vector3d& eul){

  // ZYX angles
  double theta = atan2(-R(2,0),sqrt( pow(R(2,1),2)+pow(R(2,2),2)));
  if ( ( abs( theta - M_PI/2) < 0.1 ||  abs( theta - 3*M_PI/2) < 0.1 )){
      ROS_WARN("Close to euler representation singularity!"); 
      if ( ( abs( theta - M_PI/2) < 0.01 ||  abs( theta - 3*M_PI/2) < 0.01 )){
        ROS_ERROR("Close to euler representation singularity!"); 
        stop_command_ = true; 
      }
  }

  eul(0)=atan2(R(1,0),R(0,0)); 
  eul(1)=theta;
  eul(2)=atan2(R(2,1),R(2,2));    
}


void DualArmDressingController::Eul2R(Eigen::MatrixXd& R, const Eigen::Vector3d& eul){

  // ZYX angles
  double c_phi = cos(eul(0)); 
  double c_theta = cos(eul(1)); 
  double c_psi = cos(eul(2)); 

  double s_phi = sin(eul(0)); 
  double s_theta = sin(eul(1)); 
  double s_psi = sin(eul(2)); 

  R << c_phi*c_theta, c_phi*s_theta*s_psi-s_phi*c_psi, c_phi*s_theta*c_psi+s_phi*s_psi, 
      s_phi*c_theta, s_phi*s_theta*s_psi+c_phi*c_psi,  s_phi*s_theta*c_psi-c_phi*s_psi, 
      -s_theta, c_theta*s_psi, c_theta*c_psi; 

}

//----------------

void DualArmDressingController::publishError(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_pub, const Eigen::VectorXd& error,const ros::Time& timestamp){
  
  if (arm_pub.trylock()) {
    arm_pub.msg_.header.stamp = ros::Time::now(); 
    arm_pub.msg_.pose.position.x = error[0]; 
    arm_pub.msg_.pose.position.y = error[1]; 
    arm_pub.msg_.pose.position.z = error[2]; 
    arm_pub.msg_.pose.orientation.x = error[3]; 
    arm_pub.msg_.pose.orientation.y = error[4]; 
    arm_pub.msg_.pose.orientation.z = error[5]; 
    arm_pub.unlockAndPublish();
  }
}

void DualArmDressingController::publishPoseStamped(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_pub, const Eigen::VectorXd& vec_to_pub,const ros::Time& timestamp){
  Eigen::VectorXd vec_container(7); 
  vec_container.setZero(); 
  vec_container.head(vec_to_pub.size()) = vec_to_pub; 
  if (arm_pub.trylock()) {
    arm_pub.msg_.header.stamp = ros::Time::now(); 
    arm_pub.msg_.pose.position.x = vec_container[0]; 
    arm_pub.msg_.pose.position.y = vec_container[1]; 
    arm_pub.msg_.pose.position.z = vec_container[2]; 
    arm_pub.msg_.pose.orientation.x = vec_container[3]; 
    arm_pub.msg_.pose.orientation.y = vec_container[4]; 
    arm_pub.msg_.pose.orientation.z = vec_container[5]; 
    arm_pub.msg_.pose.orientation.w = vec_container[6]; 
    arm_pub.unlockAndPublish();
  }
}

void DualArmDressingController:: publishTime(realtime_tools::RealtimePublisher<std_msgs::Float64>& time_pub, double time){
  if (time_pub.trylock()) {
      time_pub.msg_.data = time; 
      
      time_pub.unlockAndPublish();
    }

}
void DualArmDressingController:: publishMotionFinished(realtime_tools::RealtimePublisher<std_msgs::Bool>& pub){
  if (pub.trylock()) {

      pub.msg_.data = coop_motion_satisfied_;
      pub.unlockAndPublish();
    }

}


  void DualArmDressingController::publishTau(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_pub, const Eigen::VectorXd& tau, const ros::Time& timestamp){
    if (arm_pub.trylock()) {
      arm_pub.msg_.header.stamp = timestamp; 
      arm_pub.msg_.pose.position.x = tau[0]; 
      arm_pub.msg_.pose.position.y = tau[1]; 
      arm_pub.msg_.pose.position.z = tau[2]; 
      arm_pub.msg_.pose.orientation.x = tau[3]; 
      arm_pub.msg_.pose.orientation.y = tau[4]; 
      arm_pub.msg_.pose.orientation.z = tau[5]; 
      arm_pub.msg_.pose.orientation.w = tau[6]; 
      arm_pub.unlockAndPublish();
    }

  }

void DualArmDressingController::publishCenteringPose(const ros::Time& timestamp) {
  if (center_frame_pub_.trylock()) {

    center_frame_pub_.msg_.header.stamp = timestamp; 
    center_frame_pub_.msg_.header.frame_id = "base";
    center_frame_pub_.msg_.pose.position.x = sigma_[0]; 
    center_frame_pub_.msg_.pose.position.y = sigma_[1]; 
    center_frame_pub_.msg_.pose.position.z = sigma_[2]; 
    center_frame_pub_.msg_.pose.orientation.x = sigma_[3]; 
    center_frame_pub_.msg_.pose.orientation.y = sigma_[4]; 
    center_frame_pub_.msg_.pose.orientation.z = sigma_[5]; 

    center_frame_pub_.unlockAndPublish();
  }
}

void DualArmDressingController::publishFormationDistance(void ) {
  if (distance_pub_.trylock()) {

    double current_distance = sigma_.segment(6,3).norm(); 
    distance_pub_.msg_.data = current_distance; 
    distance_pub_.unlockAndPublish();
  }
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_example_controllers::DualArmDressingController,
    controller_interface::ControllerBase)
