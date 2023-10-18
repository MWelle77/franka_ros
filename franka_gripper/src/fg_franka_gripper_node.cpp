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


using franka_gripper::updateGripperState;


//very ugly global variables
double target_vel = 0.1; 
double target_width = -1; 
bool target_change = false; 
bool gripper_stopped = false; 
ros::Subscriber target_sub;


void subscriberCallback(const franka_gripper::FGMoveGoal::ConstPtr& msg){
  target_vel = msg->velocity; 
  target_width = msg->width; 
  target_change = true; 
  gripper_stopped = false;
  std::cout << "fg_franka_gripper_node: Received target width " << target_width << ", target vel " << target_vel << std::endl; 
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "fg_franka_gripper_node");
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

  // Define sub
  target_sub = node_handle.subscribe("fg_franka_gripper_goal", 1, subscriberCallback);


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

  ros::Publisher gripper_state_publisher =
      node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);


  std::thread read_thread([&gripper_state, &gripper, &gripper_state_mutex, &joint_names, &gripper_state_publisher]() {
    ros::Rate read_rate(10);
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> _(gripper_state_mutex);
        updateGripperState(gripper, &gripper_state);
        //std::cout << "Stopping loop - Target: " << target_change << " Stop: " << gripper_stopped << std::endl; 
        if (target_change && !gripper_stopped){
            std::cout << "fg_franka_gripper_node: Stopping"  << std::endl; 
            try{
              gripper.stop(); 
              gripper_stopped = true; 
              }
              catch (const franka::Exception& ex) {
              std::cout << "Catching stop" << std::endl;

              
            }
        }
      }
      // puplish the joint states of the gripper
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

      read_rate.sleep();
    }
  });


  
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
      if (target_width>0){
          // Move the gripper according to the received width and velocity

          // 
          if (target_change && gripper_stopped){
            std::cout << "Sending " << target_width << " " << target_vel << std::endl; 
            try{
              target_change = false; 
              gripper_stopped = false;
              gripper_result = gripper.move(target_width, target_vel);
              if (gripper_result == false){
                 std::cout << "fg_franka_gripper_node: Invalid gripper move!" << std::endl;  
              }
            }catch (const franka::Exception& ex) {
              std::cout << "Catching " << std::endl; 
              target_change = true; 
              gripper_stopped = true;

              
            }

            


          }
          
      }
    
    rate.sleep();
  }
  read_thread.join();
  if (stop_at_shutdown) {

    gripper.stop();
  }
  
  return 0;
}
