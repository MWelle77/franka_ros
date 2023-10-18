#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <sensor_msgs/JointState.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>

#include <franka_gripper/FGMoveGoal.h>
#include <std_msgs/Bool.h>


using franka_gripper::updateGripperState;



class FGGripperClass{
  public: 
    FGGripperClass(franka::Gripper* gripper, ros::NodeHandle* node_handle){
      gripper_ = gripper; 
      target_sub_ = node_handle->subscribe("fg_franka_gripper_goal", 1, &FGGripperClass::moveCallback, this); 
      stop_sub_ = node_handle->subscribe("fg_franka_gripper_stop", 1, &FGGripperClass::stopCallback, this); 
    }

  private: 
    double target_vel_ = 0.1; 
    double target_width_ = -1; 
    bool target_change_ = false; 
    bool gripper_stopped_ = false; 
    franka::Gripper* gripper_; 
    // Define sub
    ros::Subscriber target_sub_, stop_sub_;  
    
  void moveCallback(const franka_gripper::FGMoveGoal::ConstPtr& msg){
    target_vel_ = msg->velocity; 
    target_width_ = msg->width; 

    bool gripper_result =false;
    try{
    gripper_result = gripper_->move(target_width_, target_vel_);
  }
  catch (const franka::Exception& ex) {
      std::cout << "move: " << ex.what() << std::endl; 
    }
  
  if (gripper_result == false){
      std::cout << "fg_franka_gripper_node: Invalid gripper move!" << std::endl;  
  }
    
    std::cout << "fg_franka_gripper_node: Received target width " << target_width_ << ", target vel " << target_vel_ << std::endl; 
  }

  void stopCallback(const std_msgs::Bool::ConstPtr& msg){
    bool gripper_result =false;
    try {
      gripper_result = gripper_->stop(); 
    }
    catch (const franka::Exception& ex) {
      std::cout << "stop: " << ex.what() << std::endl; 
    }
    
    if (gripper_result == false){
      std::cout << "fg_franka_gripper_node: Invalid gripper stop!" << std::endl;  
    }else{
      std::cout << "fg_franka_gripper_node: stopping" << std::endl; 
    }
  }

}; 
int main(int argc, char** argv) {

  ros::init(argc, argv, "fg_franka_gripper_class_node");
  ros::NodeHandle node_handle("~");
  std::string robot_ip;


  if (!node_handle.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("fg_franka_gripper_node: Could not parse robot_ip parameter");
    return -1;
  }

  double default_speed(0.1);
  if (node_handle.getParam("default_speed", default_speed)) {
    ROS_INFO_STREAM("fg_franka_gripper_node: Found default_speed " << default_speed);
  }


  franka::Gripper gripper(robot_ip);


  FGGripperClass gripper_class(&gripper, &node_handle); 
  


  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("fg_franka_gripper_node: Could not find parameter publish_rate. Defaulting to "
                    << publish_rate);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("fg_franka_gripper_node: Could not parse joint_names!");
    return -1;
  }
  if (joint_names.size() != 2) {
    ROS_ERROR("fg_franka_gripper_node: Got wrong number of joint_names!");
    return -1;
  }

  bool stop_at_shutdown(false);
  if (!node_handle.getParam("stop_at_shutdown", stop_at_shutdown)) {
    ROS_INFO_STREAM("fg_franka_gripper_node: Could not find parameter stop_at_shutdown. Defaulting to "
                    << std::boolalpha << stop_at_shutdown);
  }

  franka::GripperState gripper_state;
  std::mutex gripper_state_mutex;


  std::thread read_thread([&gripper_state, &gripper, &gripper_state_mutex]() {
    ros::Rate read_rate(10);
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> _(gripper_state_mutex);
        updateGripperState(gripper, &gripper_state);
        // std::cout << "Stopping loop - Target: " << target_change << " Stop: " << gripper_stopped << std::endl; 
        // if (target_change && !gripper_stopped){
        //     std::cout << "fg_franka_gripper_node: Stopping"  << std::endl; 
        //     gripper.stop(); 
        //     gripper_stopped = true; 
        // }
      }
      read_rate.sleep();
    }
  });


  ros::Publisher gripper_state_publisher =
      node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate rate(publish_rate);
  bool gripper_result = false; 
  while (ros::ok()) {
    if (gripper_state_mutex.try_lock()) {
      sensor_msgs::JointState joint_states;
      joint_states.header.stamp = ros::Time::now();
      joint_states.name.push_back(joint_names[0]);
      joint_states.name.push_back(joint_names[1]);
      joint_states.position.push_back(gripper_state.width * 0.5);
      joint_states.position.push_back(gripper_state.width * 0.5);
      joint_states.velocity.push_back(0.0);
      joint_states.velocity.push_back(0.0);
      joint_states.effort.push_back(0.0);
      joint_states.effort.push_back(0.0);
      gripper_state_publisher.publish(joint_states);
      gripper_state_mutex.unlock();
      }
      // if (target_width>0){
      //     // Move the gripper according to the received width and velocity

      //     // 
      //     if (target_change && gripper_stopped){
      //       std::cout << "Sending " << target_width << " " << target_vel << std::endl; 
      //       gripper_result = gripper.move(target_width, target_vel);
      //       target_change = false; 
      //       gripper_stopped = false;
      //     }
      //     if (gripper_result == false){
      //         std::cout << "fg_franka_gripper_node: Invalid gripper move!" << std::endl;  
      //     }
      // }
    
    rate.sleep();
  }
  read_thread.join();
  if (stop_at_shutdown) {

    gripper.stop();
  }
  
  return 0;
}
