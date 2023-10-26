#pragma once
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "typeAlias.hpp"

enum class RobotType { real, sim };

// Define a robot arm joint struct to store joint information
struct Joint {
  std::string name;
  double position;
  double velocity;
  double acceleration;
  double effort;
  // Define a joint constructor
  Joint() {
    name = "";
    position = 0.0;
    velocity = 0.0;
    acceleration = 0.0;
    effort = 0.0;
  }
};

// Define a robot arm struct to store robot arm information
struct RobotArm {
  std::vector<Joint> joints;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Matrix3d rotation;
  Vector6d q, v, tau;
  // Define a robot arm constructor
  RobotArm() {
    // Add 6 joints to the robot arm
    joints.resize(6);
    position = Eigen::Vector3d::Zero();
    orientation = Eigen::Quaterniond::Identity();
    rotation = Eigen::Matrix3d::Identity();
    q.setZero();
    v.setZero();
    tau.setZero();
  }
};

// Define a robot communicator to communicate with robot and planner
class Communicator {
public:
  Communicator(const ros::NodeHandle &nh,
               const RobotType type = RobotType::sim);
  virtual ~Communicator() = default;
  void SendRecvOnce(const Vector6d &, const Vector6d &, const Vector6d &);
  RobotArm GetArmStateNow() const { return arm_state_now_; }
  void PrintJointState();

private:
  void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  Vector6d CalculateTorque(const Vector6d &, const Vector6d &, const Vector6d &);

  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher joint1_pub_;
  ros::Publisher joint2_pub_;
  ros::Publisher joint3_pub_;
  ros::Publisher joint4_pub_;
  ros::Publisher joint5_pub_;
  ros::Publisher joint6_pub_;
  const RobotType type_;
  RobotArm arm_state_now_, arm_state_last_;
};
