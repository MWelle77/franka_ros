// KthJointVelocityEffortInterfaceController for usage in KTH RPL lab

#include <franka_example_controllers/kth_joint_velocity_effort_interface_controller.h>
#include <cmath>
#include <functional>
#include <memory>
#include <controller_interface/controller_base.h>
#include <eigen_conversions/eigen_msg.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>



namespace franka_example_controllers {

//------------------- CHECK FUNCTIONS 

inline bool KthJointVelocityEffortInterfaceController::checkTorqueLimits(const Eigen::Matrix<double, 7, 1>& tau, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(tau[i]) > perc_allowed_torque_limit_*torque_limits_[i]) {
          ROS_ERROR("%s: Close to torque %d limit: torque value=%f, torque limit=%f", arm_id.c_str(), i, tau[i],torque_limits_[i]);
          return false; 
      } 
  }  
  return true; 
}

inline  bool KthJointVelocityEffortInterfaceController::checkDerivativeTorqueLimits(const Eigen::Matrix<double, 7, 1>& dtau, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(dtau[i]) > perc_allowed_dtorque_limit_*dtorque_limits_[i]) {
          ROS_ERROR("%s: Close to derivative torque %d limit: dtorque value=%f, dtorque limit=%f", arm_id.c_str(),  i, dtau[i],dtorque_limits_[i]);
          return false; 
      } 
  }  
  return true; 
}

inline bool KthJointVelocityEffortInterfaceController::checkJointLimits(const Eigen::Matrix<double, 7, 1>& q, const string& arm_id){
   
  for(int i = 0; i < 7; i++){
      if ( q[i] < perc_allowed_q_limit_*q_min_limits_[i] || q[i] > perc_allowed_q_limit_*q_max_limits_[i] ) {
          ROS_ERROR("%s: Close to joint %d limit: joint value=%f, joint range=[%f, %f]", arm_id.c_str(), i, q[i],q_min_limits_[i], q_max_limits_[i] );
          return false; 
      } 
  }  
  return true; 
}

inline bool KthJointVelocityEffortInterfaceController::checkVelocityJointLimits(const Eigen::Matrix<double, 7, 1>& dq, const string& arm_id){
  for(int i = 0; i < 7; i++){
      if (abs(dq[i]) > perc_allowed_dq_limit_*dq_limits_[i]) {
          ROS_ERROR("%s: Close to joint velocity %d limit: dq value=%f, dq limit=%f", arm_id.c_str(), i, dq[i],dq_limits_[i]);
          return false; 
      } 
  }  
  return true; 

}

// --------------------------------------

bool KthJointVelocityEffortInterfaceController::initArm(
    hardware_interface::RobotHW* robot_hw, const std::string& arm_id, const std::vector<std::string>& joint_names, ros::NodeHandle& node_handle, const Eigen::Affine3d& w_T_0, int ind_robot) {
  CustomFrankaDataContainerKthJointVel arm_data;
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "KthJointVelocityEffortInterfaceController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "KthJointVelocityEffortInterfaceController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "KthJointVelocityEffortInterfaceController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "KthJointVelocityEffortInterfaceController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "KthJointVelocityEffortInterfaceController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "KthJointVelocityEffortInterfaceController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }

  // Desired inertia  
  double Mdpos = 1;
  double Mdor = 1;  
  if (!node_handle.getParam("Mdpos", Mdpos)) {
    ROS_ERROR_STREAM("KthJointVelocityEffortInterfaceController: Could not read parameter Mdpos");
    return false;
  }
  if (!node_handle.getParam("Mdor", Mdor)) {
    ROS_ERROR_STREAM("KthJointVelocityEffortInterfaceController: Could not read parameter Mdor");
    return false;
  }
  arm_data.Md_.setIdentity(); 
  arm_data.Md_.topLeftCorner(3, 3) << Mdpos* Eigen::Matrix3d::Identity();
  arm_data.Md_.bottomRightCorner(3, 3) << Mdor * Eigen::Matrix3d::Identity();

  // Damping and stiffness
  std::vector<double> K_list; 
  arm_data.Kd_.setIdentity();
  arm_data.Dd_.setIdentity();

  if( !node_handle.getParam("Kp", K_list) || K_list.size() != p_arm )
      ROS_ERROR("KthJointVelocityEffortInterfaceController: Could not read parameter Kp.");
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
  if (!node_handle.getParam("impedance", impedence_active)) {
    ROS_ERROR_STREAM(
        "KthJointVelocityEffortInterfaceController: Could not read parameter impedance");
    return false;
  }
  arm_data.impedance_ = impedence_active; 
  cout << arm_id << " impedance active: " <<  arm_data.impedance_  <<endl; 

  // Get static trajectory flag (if 1 impedance controller is active)
  int static_traj = 1; 
  if (!node_handle.getParam("static", static_traj) ) {
    ROS_ERROR_STREAM(
        "KthJointVelocityEffortInterfaceController: Could not read parameter static_traj  ");
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
  
  arm_data.Gamma_i_.setZero(); 
  arm_data.Gamma_i_.block<6,6>(0,ind_robot*6) = Eigen::Matrix<double, 6, 6>::Identity(); 
  arm_data.ind_robot_ = ind_robot; 
  cout << arm_id << " Gamma_i: " <<endl << arm_data.Gamma_i_ <<endl; 

  arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));

  return true;
}


bool KthJointVelocityEffortInterfaceController::waitForInitialTransform(const tf::TransformListener& listener, tf::StampedTransform& transform, const string& frame1, const string& frame2){
  try {
    if (listener.waitForTransform(frame1, frame2, ros::Time(0), ros::Duration(4.0))) {
      listener.lookupTransform(frame1, frame2, ros::Time(0), transform);
    } else {
      ROS_ERROR( "KthJointVelocityEffortInterfaceController: Failed to read transform from %s to %s. Aborting init!", frame1.c_str(), frame2.c_str());
      return false;
    }
  } catch (tf::TransformException& ex) {
    ROS_ERROR("KthJointVelocityEffortInterfaceController: %s", ex.what());
    return false;
  }
  return true; 
}


bool KthJointVelocityEffortInterfaceController::init(hardware_interface::RobotHW* robot_hw,
                                                      ros::NodeHandle& node_handle) {
  
  cout << "---------------------Start init call ----------------- "<<endl; 
  // Get arm id
  //if (!node_handle.getParam("left/arm_id", left_arm_id_)) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR_STREAM(
        "KthJointVelocityEffortInterfaceController: Could not read parameter left_arm_id_");
    return false;
  }
  
  // Get joint names
  std::vector<std::string> joint_names;
  //if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7) {
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "KthJointVelocityEffortInterfaceController: Invalid or no left_joint_names parameters provided, aborting controller init!");
    return false;
  }
 

  // ------------ Subscribers

  // Desired poses for the robots
  boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> callbackTarget =
      boost::bind(&KthJointVelocityEffortInterfaceController::targetJointVelCallback, this, _1, arm_id_);

  ros::SubscribeOptions subscribe_options;
  subscribe_options.init("panda/equilibrium_pose", 1, callbackTarget);
  subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  sub_target_pose_ = node_handle.subscribe(subscribe_options);

  // ---------- end of subscribers 

  // Get the transformation from right_O_frame to left_O_frame
  tf::StampedTransform transform;
  tf::TransformListener listener;
  tf::transformTFToEigen(transform, Ol_T_Or_); 

  Eigen::Affine3d w_T_0;// world to zero left annd world to zero right
  waitForInitialTransform(listener, transform, "world", arm_id_ + "_link0"); 
  tf::transformTFToEigen(transform, w_T_0);  
  cout << "Initial translation w_0l" << w_T_0.translation().transpose() << endl;

  int ind_robot = 0; 
  bool success = initArm(robot_hw, arm_id_, joint_names, node_handle, w_T_0, ind_robot++ );

  rate_trigger_ = franka_hw::TriggerRate(DEFAULT_PUBLISH_RATE); 
  
  // Setup publishers
  //arm_error_pub_.init(node_handle, arm_id_+"_error", 1, true);
  //arm_error_pub_.msg_.header.frame_id = arm_id_ + "_link0";
  tau_d_pub_.init(node_handle, arm_id_+"_tau", 1, true);
  gravity_pub_.init(node_handle, arm_id_+"_gravity", 1, true); 
  q_error_pub_.init(node_handle, arm_id_+"_q_error", 1, true);
  //des_traj_pub_.init(node_handle, arm_id_+"_des_traj", 1, true);  
  //der_des_traj_pub_.init(node_handle, arm_id_+"_der_des_traj", 1, true); 
  //dder_des_traj_pub_.init(node_handle, arm_id_+"_dder_des_traj", 1, true); 
  qd_ik_pub_.init(node_handle, arm_id_+"_qd_ik", 1, true);
  dqd_ik_pub_.init(node_handle, arm_id_+"_dqd_ik", 1, true);
  //x_error_ik_pub_.init(node_handle, arm_id_+"_x_error_ik", 1, true);
  //dx_error_ik_pub_.init(node_handle, arm_id_+"_dx_error_ik", 1, true);


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

  return success;
}

void KthJointVelocityEffortInterfaceController::startingArm(CustomFrankaDataContainerKthJointVel& arm_data){
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
  //arm_data.xtilde_.setZero(); 
  arm_data.initial_q_ = q_initial; 
  arm_data.qd_ = q_initial; 
  arm_data.dqd_.setZero(); 
  arm_data.ddqd_.setZero(); 
  arm_data.pos_ = arm_data.position_d_;
  cout << "Init ZYX: "<< eul.transpose() <<endl;
  // Init analytic jacobian
  analyticalJacobian(arm_data.previous_Ja_, jacobian, eul); 
  arm_data.previous_Ja_ik_ = arm_data.previous_Ja_; 
  // init planners

  arm_data.planner_vel_q_.initdQ(arm_data.initial_q_, arm_data.dqd_, arm_data.dqd_*0 , current_time_); 

  //TODO check INIT condition for controler

}

void KthJointVelocityEffortInterfaceController::starting(const ros::Time& /*time*/) {

  for (auto& arm_data : arms_data_) {
    startingArm(arm_data.second);
   }
 
  current_time_ = 0;
  cout << "---- Time initialized -----" <<endl;  
  stop_command_ = false;   
}

void KthJointVelocityEffortInterfaceController::update(const ros::Time& /*time*/,
                                                        const ros::Duration& period) {

  ros::Time starttime_upd = ros::Time::now(); 
  // Update arms
  for (auto& arm_data : arms_data_) {
    updateArm(arm_data.second, arm_data.first, period);
  }


  auto& arm_data = arms_data_.at(arm_id_);

  // Publishers
  ros::Time timestamp = ros::Time::now(); 
  if (rate_trigger_()){
    
    // Publish EE error
    //publishError(arm_error_pub_, arm_data.xtilde_,timestamp);

    // Publish desired tau
    publishTau(tau_d_pub_, arm_data.tau_d_,timestamp);
    publishTau(gravity_pub_, arm_data.gravity_,timestamp);

    // Publish q error
    publishTau(q_error_pub_, arm_data.q_error_,timestamp); 

    // Publish desired pose,  velocity and acceleration
   // publishError(des_traj_pub_, arm_data.xd_,timestamp); 
    //publishError(der_des_traj_pub_, arm_data.dxd_,timestamp);  
    //publishError(dder_des_traj_pub_, arm_data.ddxd_,timestamp);  

    // Publish desired q and dq
    publishTau(qd_ik_pub_, arm_data.qd_,timestamp); 
    publishTau(dqd_ik_pub_, arm_data.dqd_,timestamp); 

    // Publish ik error
    //publishError(x_error_ik_pub_, arm_data.xtilde_ik_,timestamp); 
    //publishError(dx_error_ik_pub_, arm_data.dxtilde_ik_,timestamp); 

    publishTime(time_pub_, current_time_); 
  }
  ros::Time endtime_upd = ros::Time::now(); 
  ros::Duration difftime = endtime_upd-starttime_upd; 
  current_time_ = current_time_ + period.toSec(); 
  //cout << difftime.toSec()<<endl; 
}

void KthJointVelocityEffortInterfaceController::updateArm(CustomFrankaDataContainerKthJointVel& arm_data, const string& arm_id, const ros::Duration& period){
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

  // joint space motion
  Eigen::VectorXd des_q(7),des_dq(7), des_ddq(7); 
  arm_data.planner_vel_q_.getPoseQ(current_time_, des_q, des_dq, des_ddq);
  arm_data.qd_ = des_q; 
  arm_data.dqd_ = des_dq; 
  arm_data.ddqd_ = des_ddq; 
  /*
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
    arm_data.planner_.init(arm_data.position_d_, eul, current_time_); 

    arm_data.conf_initialized_ = true; 
  }
  */


/*
  // Get desired trajectory
  Eigen::VectorXd xd(6), dxd(6), ddxd(6); 

  // get curretn step
  arm_data.planner_.getPose(current_time_, xd, dxd, ddxd);  

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
  Eigen::Matrix<double, 6, 6> Ta; 
  transfAnalyticalJacobian(Ta, eul_angles); 
  */
  Eigen::VectorXd tau_task(7), y(7); 

  // TO CHANGE!! non considero wrench
  F_ext_hat_0 = - 0*F_ext_hat_0; // in this way we consider the forces that the robot exerts on the environment
  /*
  //cout << "Iner:"<< inertia << endl; 
  //cout << "Norm: " << inertia.norm() << endl; 


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



  //performe the caresian space motion
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


*/
  // Integration with forward euler
  arm_data.qd_ = arm_data.qd_ + arm_data.dqd_*period.toSec(); 
  arm_data.dqd_ = arm_data.dqd_ +arm_data.ddqd_ *period.toSec(); 

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

Eigen::Matrix<double, 7, 1> KthJointVelocityEffortInterfaceController::saturateTorqueRate(
    const CustomFrankaDataContainerKthJointVel& arm_data,
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
void KthJointVelocityEffortInterfaceController::getDesiredTrajectory(bool stationary, double time, const Eigen::Matrix<double, 6, 1>& init_x, Eigen::Matrix<double, 6, 1>& xd, Eigen::Matrix<double, 6, 1>& dxd, Eigen::Matrix<double, 6, 1>& ddxd){
  if (stationary){
    getStationaryTrajectory(time, init_x, xd, dxd, ddxd); 
  }else{
    getSinTrajectory(time, init_x, xd, dxd, ddxd); 
  }
}

void KthJointVelocityEffortInterfaceController::getStationaryTrajectory(double time, const Eigen::Matrix<double, 6, 1>& init_x, Eigen::Matrix<double, 6, 1>& xd, Eigen::Matrix<double, 6, 1>& dxd, Eigen::Matrix<double, 6, 1>& ddxd){
  xd = init_x; 
  dxd.setZero(); 
  ddxd.setZero(); 
}

void KthJointVelocityEffortInterfaceController::getSinTrajectory(double time, const Eigen::Matrix<double, 6, 1>& init_x, Eigen::Matrix<double, 6, 1>& xd, Eigen::Matrix<double, 6, 1>& dxd, Eigen::Matrix<double, 6, 1>& ddxd){
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

void KthJointVelocityEffortInterfaceController::targetJointVelCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const string& robot_id){
  

  //cout << "PLANNING q MOTION FOR ROBOT learning poses "  <<endl;     
  auto& arm_data = arms_data_.at(robot_id);
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array = arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  arm_data.initial_q_ = q_initial; 

  Eigen::Matrix<double, 7, 1> des_dq_; 
  des_dq_ <<msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
  //cout << " arm_data.initial_q_ " << arm_data.initial_q_ << " arm_data.dqd_ " << arm_data.dqd_ << " des_dq_ " << des_dq_  << endl; 
  //arm_data.planner_q_.initQ(arm_data.initial_q_, current_time_); 
  arm_data.planner_vel_q_.plandQ(arm_data.initial_q_, arm_data.dqd_, des_dq_, current_time_, 0.1); 
  
    



  /*
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

  arm_data.planner_q_.plan(curr_pos, eul_angles, p_0_d, eul_angles_d, current_time_); 
  
  cout << robot_id << " Desired position: "<<p_0_d.transpose() << " Current position: " << curr_pos.transpose()<<endl; 
  cout << robot_id << " Desired orientation: "<<eul_angles_d.transpose()<< " Current orientation: "<< eul_angles.transpose()<< endl; 
  */

}

void KthJointVelocityEffortInterfaceController::analyticalJacobian(Eigen::Matrix<double, 6, 7>& Ja, const Eigen::Matrix<double, 6, 7>& J, const Eigen::Vector3d& eul_angles){
    //cout << "J: "<< J <<endl; 
    Eigen::Matrix<double, 6, 6> Ta; 
    //Ta.setIdentity(); 
    transfAnalyticalJacobian(Ta, eul_angles); 
    Ja = Ta.inverse()*J; 
    //cout << "Ja: "<< Ja <<endl; 
 }

 void KthJointVelocityEffortInterfaceController::transfAnalyticalJacobian(Eigen::Matrix<double, 6, 6>& Ta, const Eigen::Vector3d& ZYX){
   if (eul_seq_[0]!= 2 || eul_seq_[1]!= 1 || eul_seq_[2]!= 0){
     ROS_WARN("ZYX euler angles must be considered"); 
   }
    Eigen::Matrix<double, 3, 3> T; 
    T << 0, -sin(ZYX[0]), cos(ZYX[0])*cos(ZYX[1]), 0, cos(ZYX[0]), sin(ZYX[0])*cos(ZYX[1]), 1, 0, -sin(ZYX[1]); 
    Ta = Eigen::Matrix<double, 6, 6>::Identity(); 
    Ta.block<3,3>(3,3) = T;  
    //cout << "Ta: " << endl << Ta <<endl; 
 }


/*
  void KthJointVelocityEffortInterfaceController::transformCartesianPose(const Eigen::VectorXd& xd_w, Eigen::VectorXd& xd_0, const Eigen::Affine3d& A_0_w, const Eigen::VectorXd& current_eul_0){
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
*/

  void KthJointVelocityEffortInterfaceController::getContinuousEulerAngles(Eigen::Matrix<double, 3, 1>& eul, const Eigen::Matrix<double, 3, 1>& previous_eul ){
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

double KthJointVelocityEffortInterfaceController::sign(double n){
	if (n > 0){
		return 1; 
	}else if (n < 0){
		return -1; 
	}else{
		return 0; 
	}	
}



// UTILS FUNCTION 

void KthJointVelocityEffortInterfaceController::R2Eul(const Eigen::MatrixXd& R, Eigen::Vector3d& eul){

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


void KthJointVelocityEffortInterfaceController::Eul2R(Eigen::MatrixXd& R, const Eigen::Vector3d& eul){

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

void KthJointVelocityEffortInterfaceController::publishError(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_pub, const Eigen::VectorXd& error,const ros::Time& timestamp){
  
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

void KthJointVelocityEffortInterfaceController::publishPoseStamped(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_pub, const Eigen::VectorXd& vec_to_pub,const ros::Time& timestamp){
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

void KthJointVelocityEffortInterfaceController:: publishTime(realtime_tools::RealtimePublisher<std_msgs::Float64>& time_pub, double time){
  if (time_pub.trylock()) {
      time_pub.msg_.data = time; 
      
      time_pub.unlockAndPublish();
    }
}


  void KthJointVelocityEffortInterfaceController::publishTau(realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>& arm_pub, const Eigen::VectorXd& tau, const ros::Time& timestamp){
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



}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_example_controllers::KthJointVelocityEffortInterfaceController,
    controller_interface::ControllerBase)
