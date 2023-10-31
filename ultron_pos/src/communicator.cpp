#include "communicator.hpp"

Communicator::Communicator(const ros::NodeHandle &nh, const RobotType type) : nh_(nh), type_(type) {
  joint1_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint1_torque_controller/command", 1);
  joint2_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint2_torque_controller/command", 1);
  joint3_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint3_torque_controller/command", 1);
  joint4_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint4_torque_controller/command", 1);
  joint5_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint5_torque_controller/command", 1);
  joint6_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint6_torque_controller/command", 1);
  if (type_ == RobotType::sim) {
    joint_state_sub_ = nh_.subscribe("ultron/joint_states", 10, &Communicator::JointStateCallback, this);
  }else if(type_ == RobotType::real){
    VectorXd q, v;
    for(int i=0;i<10;i++)
    {
      arx_real.set_joints_torque(Vector6d::Zero());
      arx_real.get_curr_pos(q, v);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::lock_guard<std::mutex> lock(arm_state_mtx_);
    // Clear the ultron_arm_now
    arm_state_now_.joints.clear();
    // Push back the joint information to ultron_arm_now
    for (int i = 0; i < q.size(); i++) {
      Joint joint;
      joint.position = q[i];
      joint.velocity = v[i];
      arm_state_now_.q(i) = joint.position;
      arm_state_now_.v(i) = joint.velocity;
      arm_state_now_.joints.push_back(joint);
    }
  }
  ee_target_sub_ = nh_.subscribe("ultron/ee_target", 10, &Communicator::EETargetCallback, this);
}

void Communicator::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(arm_state_mtx_);
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

void Communicator::EETargetCallback(const geometry_msgs::Pose::ConstPtr &msg) {
  hasNewTarget = true;
  std::lock_guard<std::mutex> lock(ee_target_mtx_);
  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::Vector3d t(msg->position.x, msg->position.y, msg->position.z);
  oMdes_.translation() = t;
  oMdes_.rotation() = q;
}

void Communicator::PrintJointState() {
  for (int i = 0; i < arm_state_now_.joints.size(); i++) {
    std::cout << "Joint " << i << " name: " << arm_state_now_.joints[i].name << std::endl;
    std::cout << "Joint " << i << " position: " << arm_state_now_.joints[i].position << std::endl;
    std::cout << "Joint " << i << " velocity: " << arm_state_now_.joints[i].velocity << std::endl;
    std::cout << "Joint " << i << " effort: " << arm_state_now_.joints[i].effort << std::endl;
  }
}

Vector6d Communicator::CalculateTorque(const Vector6d &qd, const Vector6d &vd, const Vector6d &tau) {
  Vector6d tau_cmd;
  auto pd = [&](double kp, double kd, double q, double v, double qd, double vd, double ff) {
    return kp * (qd - q) + kd * (vd - v) + ff;
  };
  std::lock_guard<std::mutex> lock(arm_state_mtx_);
  for (int i = 0; i < 3; i++)
    tau_cmd(i) = pd(100, 1., arm_state_now_.joints[i].position, arm_state_now_.joints[i].velocity, qd(i), vd(i), tau(i));
  tau_cmd(3) = pd(35, 10., arm_state_now_.joints[3].position, arm_state_now_.joints[3].velocity, qd(3), vd(3), tau(3));
  tau_cmd(4) = pd(15, 5, arm_state_now_.joints[4].position, arm_state_now_.joints[4].velocity, qd(4), vd(4), tau(4));
  tau_cmd(5) = pd(15, 5, arm_state_now_.joints[5].position, arm_state_now_.joints[5].velocity, qd(5), vd(5), tau(5));
  return tau_cmd;
}

void Communicator::SendRecvOnce(const Vector6d &qd, const Vector6d &vd, const Vector6d &tau) {
  if (type_ == RobotType::real) {
    Vector6d tau_cmd = CalculateTorque(qd, vd, tau);
    VectorXd q, v;
    // arx_real.set_joints_torque(tau_cmd);
    // arx_real.set_joints_pos(qd);
    // arx_real.set_joints_pos_vel(qd,vd);
    arx_real.set_joints_pos_vel_tau(qd,vd,tau);
    arx_real.get_curr_pos(q, v);
    std::lock_guard<std::mutex> lock(arm_state_mtx_);
    // Clear the ultron_arm_now
    arm_state_now_.joints.clear();
    // Push back the joint information to ultron_arm_now
    for (int i = 0; i < q.size(); i++) {
      Joint joint;
      joint.position = q[i];
      joint.velocity = v[i];
      arm_state_now_.q(i) = joint.position;
      arm_state_now_.v(i) = joint.velocity;
      arm_state_now_.tau(i) = joint.effort;
      arm_state_now_.joints.push_back(joint);
    }
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