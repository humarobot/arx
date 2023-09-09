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

  // Load the urdf model
  pinocchio::urdf::buildModel(URDF_FILE, model_);
  // Create data required by the algorithms
  data_ = pinocchio::Data(model_);
  link_id_ = model_.getFrameId("link4");
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

void Demonstrator::ReplayImpedence(){
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;

  Eigen::VectorXd tau_zero(6);
  tau_zero.setZero();
  robotic_arm_->set_joints_torque(tau_zero);

  int sleep_time = 1000000 / hz_;
  bag_.open(bag_name_, rosbag::bagmode::Read);
  rosbag::View view_pos(bag_, rosbag::TopicQuery("joints_position"));
  rosbag::View view_vel(bag_, rosbag::TopicQuery("joints_velocity"));

  rosbag::View::iterator it_pos = view_pos.begin();
  rosbag::View::iterator it_vel = view_vel.begin();

  Eigen::Matrix<double, 3, 3> Kp = Eigen::Matrix<double, 3, 3>::Identity();
  Kp.diagonal().head<3>().array() = 250.0;
  Eigen::Matrix<double, 3, 3> Kd = Eigen::Matrix<double, 3, 3>::Identity();
  Kd.diagonal().head<3>().array() = 8.;
  vector_t q(6), v(6), a(6);
  pinocchio::Data data_d_(model_);

  //skip first 10 data
  for(int i=0;i<10;i++){
    ++it_pos;
    ++it_vel;
  }
  while (it_pos != view_pos.end() && it_vel != view_vel.end() && ros::ok()) {
    robotic_arm_->get_curr_pos(q,v);

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

    pinocchio::forwardKinematics(model_, data_, q.head(3), v.head(3));
    pinocchio::forwardKinematics(model_,data_d_,pos.head(3),vel.head(3));
    pinocchio::computeJointJacobians(model_, data_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::crba(model_, data_, q.head(3));
    data_.M.triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model_, data_, q.head(3), v.head(3));
    Eigen::Matrix<double, 6, 3> jac;
    pinocchio::getFrameJacobian(model_, data_, link_id_, pinocchio::WORLD, jac);
    Vector6d err;
    auto oMf_measure = data_.oMf[link_id_];
    auto oMf_desired = data_d_.oMf[link_id_];
    Eigen::Matrix3d rot_err;

    err.head(3) = oMf_measure.translation()-oMf_desired.translation();
    Eigen::Matrix<double, 3, 3> j_v;
    Eigen::Matrix3d skew_matrix;
    Eigen::Vector3d vec;
    vec = oMf_measure.translation();
    skew_matrix << 0.0, -vec(2), vec(1),
               vec(2), 0.0, -vec(0),
               -vec(1), vec(0), 0.0;
    j_v = jac.topRows(3)- skew_matrix*(jac.bottomRows(3));
    Eigen::VectorXd tau(6);
    Eigen::Vector3d ext_force,ext_torque;
    ext_force = Kp * err.head(3) + Kd * (j_v * v.head(3));
    tau.head(3) = data_.nle - j_v.transpose() * (ext_force);
    tau.tail(3)<<0.0,0.0,0.0;
    robotic_arm_->set_head_3_torque(tau.head(3));
    robotic_arm_->set_tail_3_pos(pos.tail(3));

    std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    ++it_pos;
    ++it_vel;
  }

}