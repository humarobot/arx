#include "demonstrator.hpp"
#include <chrono>
#include <iostream>
#include <thread>

Demonstrator::Demonstrator(ros::NodeHandle &nh, int hz,
                           const std::string &bag_name)
    : nh_(nh), hz_(hz), bag_name_(bag_name) {
  robotic_arm_ = std::make_shared<arx_arm>(0);
  tau_w_.setZero();
  q_r_.setZero();
  v_r_.setZero();
}

void Demonstrator::StartUp() {

  // Communicate with the robot arm at a certain frequency
  while (ros::ok()) {
    auto start = std::chrono::high_resolution_clock::now();
    //*************** Working code ***************************
    robotic_arm_->set_joints_torque(tau_w_);
    robotic_arm_->get_curr_pos(q_r_, v_r_);
    //*******************************************************
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    int sleep_time = 1000000 / hz_ - duration.count();
    if (sleep_time > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    } else {
      // Print warning
      std::cout
          << "Warning: the control loop is slower than the desired frequency!"
          << std::endl;
    }
  }
}

void Demonstrator::Record() {
  double t = 0.0;
  bag_.open(bag_name_, rosbag::bagmode::Write);
  while (ros::ok() && t < 30.0) {
    auto start = std::chrono::high_resolution_clock::now();
    //*************** Working code ***************************
    robotic_arm_->set_joints_torque(tau_w_);
    robotic_arm_->get_curr_pos(q_r_, v_r_);
    auto timestamp = ::ros::Time(t + 1e-6); // t=0.0 throws ROS exception
    std_msgs::Float64MultiArray q_msg, v_msg;
    q_msg.data.resize(6);
    v_msg.data.resize(6);
    for (int i = 0; i < 6; i++) {
      q_msg.data[i] = q_r_(i);
      v_msg.data[i] = v_r_(i);
    }
    bag_.write("joints_position", timestamp, q_msg);
    bag_.write("joints_velocity", timestamp, v_msg);
    //*******************************************************
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    int sleep_time = 1000000 / hz_ - duration.count();
    if (sleep_time > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    } else {
      // Print warning
      std::cout
          << "Warning: the control loop is slower than the desired frequency!"
          << std::endl;
      break;
    }
    t += 1.0 / hz_;
  }
  bag_.close();
  std::cout << "Record finished!" << std::endl;
}

void Demonstrator::Replay() {
  int sleep_time = 1000000 / hz_;
  bag_.open(bag_name_, rosbag::bagmode::Read);
  rosbag::View view_pos(bag_, rosbag::TopicQuery("joints_position"));
  rosbag::View view_vel(bag_, rosbag::TopicQuery("joints_velocity"));

  rosbag::View::iterator it_pos = view_pos.begin();
  rosbag::View::iterator it_vel = view_vel.begin();
  //skip first 10 data
  for(int i=0;i<10;i++){
    ++it_pos;
    ++it_vel;
  }

  while (it_pos != view_pos.end() && it_vel != view_vel.end() && ros::ok()) {
    const rosbag::MessageInstance &ins_pos = *it_pos;
    const rosbag::MessageInstance &ins_vel = *it_vel;
    std_msgs::Float64MultiArray::ConstPtr msg_pos =
        ins_pos.instantiate<std_msgs::Float64MultiArray>();
    std_msgs::Float64MultiArray::ConstPtr msg_vel =
        ins_vel.instantiate<std_msgs::Float64MultiArray>();

    // Do something with the messages...
    Eigen::VectorXd pos(6),vel(6);
    for (int i = 0; i < 6; i++) {
      pos(i) = msg_pos->data[i];
      vel(i) = msg_vel->data[i];
    }
    // Print pos,vel
    std::cout << "pos = " << pos.transpose() << std::endl;
    std::cout << "vel = " << vel.transpose() << std::endl;
    robotic_arm_->set_joints_pos_vel(pos,vel);
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    ++it_pos;
    ++it_vel;
  }
}