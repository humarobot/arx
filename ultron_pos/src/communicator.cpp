#include "communicator.hpp"

Communicator::Communicator(const ros::NodeHandle &nh, const RobotType type)
    : nh_(nh), type_(type) {
  joint1_pub_ = nh_.advertise<std_msgs::Float64>(
      "/ultron/joint1_torque_controller/command", 1);
  joint2_pub_ = nh_.advertise<std_msgs::Float64>(
      "/ultron/joint2_torque_controller/command", 1);
  joint3_pub_ = nh_.advertise<std_msgs::Float64>(
      "/ultron/joint3_torque_controller/command", 1);
  joint4_pub_ = nh_.advertise<std_msgs::Float64>(
      "/ultron/joint4_torque_controller/command", 1);
  joint5_pub_ = nh_.advertise<std_msgs::Float64>(
      "/ultron/joint5_torque_controller/command", 1);
  joint6_pub_ = nh_.advertise<std_msgs::Float64>(
      "/ultron/joint6_torque_controller/command", 1);
  if (type_ == RobotType::sim) {
    joint_state_sub_ = nh_.subscribe("ultron/joint_states", 10,
                                     &Communicator::JointStateCallback, this);
  }
}

void Communicator::JointStateCallback(
    const sensor_msgs::JointState::ConstPtr &msg) {
  // Clear the ultron_arm_now
  arm_state_now_.joints.clear();
  // Push back the joint information to ultron_arm_now
  for (int i = 0; i < msg->name.size(); i++) {
    Joint joint;
    joint.name = msg->name[i];
    joint.position = msg->position[i];
    joint.velocity = msg->velocity[i];
    joint.effort = msg->effort[i];
    arm_state_now_.q(i) = joint.position;
    arm_state_now_.v(i) = joint.velocity;
    arm_state_now_.tau(i) = joint.effort;
    arm_state_now_.joints.push_back(joint);
  }
}

void Communicator::PrintJointState() {
  for (int i = 0; i < arm_state_now_.joints.size(); i++) {
    std::cout << "Joint " << i << " name: " << arm_state_now_.joints[i].name
              << std::endl;
    std::cout << "Joint " << i
              << " position: " << arm_state_now_.joints[i].position
              << std::endl;
    std::cout << "Joint " << i
              << " velocity: " << arm_state_now_.joints[i].velocity
              << std::endl;
    std::cout << "Joint " << i << " effort: " << arm_state_now_.joints[i].effort
              << std::endl;
  }
}

Vector6d Communicator::CalculateTorque(const Vector6d &qd, const Vector6d &vd, const Vector6d &tau){
  Vector6d tau_cmd;

  auto pd = [&](double kp, double kd, double q, double v, double qd, double vd, double ff) {
    return kp * (qd - q) + kd * (vd - v)+ ff;
  };
  for(int i = 0; i < 6; i++)
    tau_cmd(i) = pd(10, 2, arm_state_now_.joints[i].position, arm_state_now_.joints[i].velocity, qd(i), vd(i), tau(i));
  return tau_cmd;
}

void Communicator::SendRecvOnce(const Vector6d &qd, const Vector6d &vd, const Vector6d &tau) {
  if (type_ == RobotType::real) {

  } else if (type_ == RobotType::sim) {
    Vector6d tau_cmd = CalculateTorque(qd, vd, tau);
    std_msgs::Float64 joint1_msg;
    std_msgs::Float64 joint2_msg;
    std_msgs::Float64 joint3_msg;
    std_msgs::Float64 joint4_msg;
    std_msgs::Float64 joint5_msg;
    std_msgs::Float64 joint6_msg;
    joint1_msg.data = tau_cmd(0);
    joint2_msg.data = tau_cmd(1);
    joint3_msg.data = tau_cmd(2);
    joint4_msg.data = tau_cmd(3);
    joint5_msg.data = tau_cmd(4);
    joint6_msg.data = tau_cmd(5);
    joint1_pub_.publish(joint1_msg);
    joint2_pub_.publish(joint2_msg);
    joint3_pub_.publish(joint3_msg);
    joint4_pub_.publish(joint4_msg);
    joint5_pub_.publish(joint5_msg);
    joint6_pub_.publish(joint6_msg);
  }
}