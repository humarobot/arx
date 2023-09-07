#include "demonstrator.hpp"
#include <chrono>
#include <thread>
#include <iostream>

Demonstrator::Demonstrator(ros::NodeHandle& nh):nh_(nh)
{
  robotic_arm_ = std::make_shared<arx_arm>(0);
  tau_w_.setZero();
}

void Demonstrator::StartUp(int hz)
{
  
  //Communicate with the robot arm at a certain frequency
  while(ros::ok()) {
    auto start = std::chrono::high_resolution_clock::now();
    // 在这里插入您的代码
    robotic_arm_->set_joints_torque(tau_w_);
    robotic_arm_->get_curr_pos(q_r_, v_r_);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    int sleep_time = 1000000 / hz - duration.count();
    if (sleep_time > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    } else{
      // Print warning
      std::cout<<"Warning: the control loop is slower than the desired frequency!"<<std::endl;
    }
  }

}

void Demonstrator::Record()
{

}