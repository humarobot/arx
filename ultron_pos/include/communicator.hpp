#pragma once
#include "pinocchio/algorithm/kinematics.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "typeAlias.hpp"
#include <thread>
#include "App/arm_control.h"
#include <chrono>
#include "lion_msg/armTraj.h"
#include <mutex>
#include "trajectoryLoader.hpp"
#include "std_msgs/String.h"

enum class RobotType { real, simGazebo, simMujoco };

// Define a robot arm joint struct to store joint information
struct Joint {
  std::string name;
  double position;
  double velocity;
  double acceleration;
  double effort;
};

// Define a robot arm struct to store robot arm information
struct RobotArm {
  std::vector<Joint> joints{6, Joint{}};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Vector6d q{Vector6d::Zero()}, v{Vector6d::Zero()}, tau{Vector6d::Zero()};
};

// Define a robot communicator to communicate with robot and planner
class Communicator {
 public:
  Communicator(const ros::NodeHandle &nh,TrajectoryLoader&, const RobotType type = RobotType::simGazebo);
  virtual ~Communicator() = default;
  void SendRecvOnce(const Vector6d &, const Vector6d &, const Vector6d &);
  RobotArm GetArmStateNow() const { return arm_state_now_; }
  pinocchio::SE3 GetEETarget() const { return oMdes_; }
  bool HasNewTarget() const { return hasNewTarget_; }
  bool HasNewTraj() const { return hasNewTraj_; }
  void ResetNewTarget() { hasNewTarget_ = false; }
  void ResetNewTraj() { hasNewTraj_ = false; }
  void PrintJointState();
  std::vector<Vector6d> GetQTraj() const { return qTraj_; }
  void PublishEEPose(const pinocchio::SE3 &oMee);

  std::mutex arm_state_mtx_;
  std::mutex ee_target_mtx_;

  int execPriority_{0};
  bool atInitPosi_{false};

 private:
  void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void EETargetCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void ArmTrajCallback(const lion_msg::armTraj::ConstPtr &msg);
  void ExecuteCallback(const std_msgs::Bool::ConstPtr &msg);
  Vector6d CalculateTorque(const Vector6d &, const Vector6d &, const Vector6d &);
  void JointsPosVelCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void LoadTrajCallback(const std_msgs::String::ConstPtr &msg);
  

  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber ee_target_sub_;
  ros::Subscriber arm_traj_sub_;
  ros::Subscriber execute_sub_;
  ros::Publisher joint1_pub_;
  ros::Publisher joint2_pub_;
  ros::Publisher joint3_pub_;
  ros::Publisher joint4_pub_;
  ros::Publisher joint5_pub_;
  ros::Publisher joint6_pub_;
  // For publishing current end-effector pose
  ros::Publisher ee_pose_pub_;
  // For mujoco
  ros::Publisher jointsTorque_pub_;
  ros::Subscriber jointsPosVel_sub_;

  TrajectoryLoader& traj_loader_;
  ros::Subscriber load_traj_sub_;

  const RobotType type_;
  RobotArm arm_state_last_{}, arm_state_now_{};
  pinocchio::SE3 oMdes_{Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.1, 0.0, 0.16)};
  bool hasNewTarget_{true};
  arx_arm arx_real{0};

  // Arm trajectory related variables
  std::vector<Vector6d> qTraj_, vTraj_;
  int numKnots_{0};
  double totalTime_{0.0};
  bool hasNewTraj_{false};
};
