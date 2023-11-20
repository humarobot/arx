#include "communicator.hpp"

Communicator::Communicator(const ros::NodeHandle &nh, const RobotType type) : nh_(nh), type_(type) {
  if (type_ == RobotType::simGazebo) {
    joint1_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint1_torque_controller/command", 1);
    joint2_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint2_torque_controller/command", 1);
    joint3_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint3_torque_controller/command", 1);
    joint4_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint4_torque_controller/command", 1);
    joint5_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint5_torque_controller/command", 1);
    joint6_pub_ = nh_.advertise<std_msgs::Float64>("/ultron/joint6_torque_controller/command", 1);
    joint_state_sub_ = nh_.subscribe("ultron/joint_states", 10, &Communicator::JointStateCallback, this);
  } else if (type_ == RobotType::real) {
    VectorXd q, v;
    for (int i = 0; i < 10; i++) {
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
  } else if (type_ == RobotType::simMujoco){
    jointsPosVel_sub_ = nh_.subscribe("/ultron_mj/jointsPosVel", 10, &Communicator::JointsPosVelCallback, this);
    jointsTorque_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ultron_mj/jointsTorque", 1);
  }
  execute_sub_ = nh_.subscribe("/execute_traj", 10, &Communicator::ExecuteCallback, this);
  // ee_target_sub_ = nh_.subscribe("ultron/ee_target", 10, &Communicator::EETargetCallback, this);
  // arm_traj_sub_ = nh_.subscribe("/arm_trajectory_topic", 10, &Communicator::ArmTrajCallback, this);
  std::cout << "Communicator init done" << std::endl;
}

void Communicator::JointsPosVelCallback(const std_msgs::Float64MultiArrayConstPtr &msg){
  Vector6d q, v;
  for (int i = 0; i < 6; i++){
    q(i) = msg->data[i];
    v(i) = msg->data[i+6];
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

void Communicator::ExecuteCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    std::cout << "ExecuteTrajectoryCallback" << std::endl;
    execPriority_++;
  }
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
  hasNewTarget_ = true;
  std::lock_guard<std::mutex> lock(ee_target_mtx_);
  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::Vector3d t(msg->position.x, msg->position.y, msg->position.z);
  oMdes_.translation() = t;
  oMdes_.rotation() = q;
}

void Communicator::ArmTrajCallback(const lion_msg::armTraj::ConstPtr &msg) {
  std::cout << "ArmTrajCallback" << std::endl;
  totalTime_ = msg->totalTime;
  numKnots_ = msg->numKnots;
  qTraj_.clear();
  vTraj_.clear();
  assert(numKnots_ == msg->position.size());
  for (auto pos : msg->position) {
    Vector6d q;
    q(0) = pos.q1;
    q(1) = pos.q2;
    q(2) = pos.q3;
    q(3) = pos.q4;
    q(4) = pos.q5;
    q(5) = pos.q6;
    qTraj_.push_back(q);
  }

  // for (auto vel : msg->velocity) {
  //   Vector6d v;
  //   v(0) = vel.q1;
  //   v(1) = vel.q2;
  //   v(2) = vel.q3;
  //   v(3) = vel.q4;
  //   v(4) = vel.q5;
  //   v(5) = vel.q6;
  //   vTraj_.push_back(v);
  // }

  hasNewTraj_ = true;
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
    tau_cmd(i) =
        pd(100, 0.1, arm_state_now_.joints[i].position, arm_state_now_.joints[i].velocity, qd(i), vd(i), tau(i));
  tau_cmd(3) = pd(35, 0.1, arm_state_now_.joints[3].position, arm_state_now_.joints[3].velocity, qd(3), vd(3), tau(3));
  tau_cmd(4) = pd(15, 0., arm_state_now_.joints[4].position, arm_state_now_.joints[4].velocity, qd(4), vd(4), tau(4));
  tau_cmd(5) = pd(15, 0., arm_state_now_.joints[5].position, arm_state_now_.joints[5].velocity, qd(5), vd(5), tau(5));
  return tau_cmd;
}

void Communicator::SendRecvOnce(const Vector6d &qd, const Vector6d &vd, const Vector6d &tau) {
  if (type_ == RobotType::real) {
    Vector6d tau_cmd = CalculateTorque(qd, vd, tau);
    VectorXd q, v;
    // arx_real.set_joints_torque(tau_cmd);
    // arx_real.set_joints_pos(qd);
    // arx_real.set_joints_pos_vel(qd,vd);
    arx_real.set_joints_pos_vel_tau(qd, vd, tau);
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
  } else if (type_ == RobotType::simGazebo) {
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
  } else if (type_ == RobotType::simMujoco){
    
    Vector6d tau_cmd = CalculateTorque(qd, vd, tau);
    // Vector6d tau_cmd = tau;
    //print qd
    // std::cout<<"tau_cmd:"<<std::endl;
    // std::cout<<tau_cmd.transpose()<<std::endl;
    std_msgs::Float64MultiArray tau_cmd_msg;
    tau_cmd_msg.data.resize(6);
    for (int i = 0; i < 6; i++){
      tau_cmd_msg.data[i] = tau_cmd(i);
    }
    jointsTorque_pub_.publish(tau_cmd_msg);

  }
}