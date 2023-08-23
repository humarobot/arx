
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "App/arm_control.h"

/**
* Robot work space
* face down: x: 0.05~0.4, y: -0.4~0.4, z: -0.2~0.2
*/

/** Dynamic-size vector type. */
typedef Eigen::Matrix<double, 6, 1> Vector6d;
using vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
bool real_robot = false;

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
  // Define a robot arm constructor
  RobotArm() {
    // Add 6 joints to the robot arm
    joints.resize(6);
    position = Eigen::Vector3d::Zero();
    orientation = Eigen::Quaterniond::Identity();
    rotation = Eigen::Matrix3d::Identity();
  }
};

RobotArm ultron_arm_now, ultron_arm_last;
// Define update rate
int update_rate = 500;
double update_period = 1.0 / update_rate;
vector_t q(6), v(6), a(6);
pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(),
                     Eigen::Vector3d(0.08, 0., 0.16));
pinocchio::SE3 oMdes_last(Eigen::Matrix3d::Identity(),
                     Eigen::Vector3d(0.08, 0., 0.16));
bool new_target = false;


void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  // Clear the ultron_arm_now
  ultron_arm_now.joints.clear();
  // Push back the joint information to ultron_arm_now
  for (int i = 0; i < msg->name.size(); i++) {
    Joint joint;
    joint.name = msg->name[i];
    joint.position = msg->position[i];
    joint.velocity = msg->velocity[i];
    joint.effort = msg->effort[i];
    ultron_arm_now.joints.push_back(joint);
  }
  // Compute joint acceleration and using low pass filter to smooth the
  // acceleration
  for (int i = 0; i < ultron_arm_now.joints.size(); i++) {
    ultron_arm_now.joints[i].acceleration =
        (ultron_arm_now.joints[i].velocity -
         ultron_arm_last.joints[i].velocity) /
        update_period;
    ultron_arm_now.joints[i].acceleration =
        0.05 * ultron_arm_now.joints[i].acceleration +
        0.95 * ultron_arm_last.joints[i].acceleration;
  }
  // Deep copy the ultron_arm_now to ultron_arm_last
  ultron_arm_last = ultron_arm_now;
  // Store joints info to q,v,a
  q.resize(ultron_arm_now.joints.size());
  v.resize(ultron_arm_now.joints.size());
  a.resize(ultron_arm_now.joints.size());
  for (int i = 0; i < ultron_arm_now.joints.size(); i++) {
    q(i) = ultron_arm_now.joints[i].position;
    v(i) = ultron_arm_now.joints[i].velocity;
    a(i) = ultron_arm_now.joints[i].acceleration;
  }
}

void poseCallback(const geometry_msgs::Pose::ConstPtr &msg) {
  ROS_INFO("End effector target position: [%f,%f,%f]", msg->position.x,
           msg->position.y, msg->position.z);
  ROS_INFO("End effector target orientation: [%f,%f,%f]", msg->orientation.x,
           msg->orientation.y, msg->orientation.z, msg->orientation.w);
  new_target = true;

  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x,
                       msg->orientation.y, msg->orientation.z);
  // quaternion to rotation matrix
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  Eigen::Vector3d t(msg->position.x, msg->position.y, msg->position.z);
  oMdes = pinocchio::SE3(R, t);
}

int main(int argc, char **argv) {
  // Init ros node
  ros::init(argc, argv, "ultron");
  ros::NodeHandle nh;
  arx_arm arx_real(0);
  ros::Rate loop_rate(500);
  // Init a joint state topic subscriber
  ros::Subscriber joint_state_sub =
      nh.subscribe("ultron/joint_states", 10, jointStateCallback);
  // Init a pose topic subscriber
  ros::Subscriber pose_sub = nh.subscribe("ultron/pose", 100, poseCallback);
  // Init joints effort topic publisher
  ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint1_torque_controller/command", 1);
  ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint2_torque_controller/command", 1);
  ros::Publisher joint3_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint3_torque_controller/command", 1);
  ros::Publisher joint4_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint4_torque_controller/command", 1);
  ros::Publisher joint5_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint5_torque_controller/command", 1);
  ros::Publisher joint6_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint6_torque_controller/command", 1);
  // Init a joints acceleration topic publisher
  ros::Publisher joint_acceleration_pub =
      nh.advertise<std_msgs::Float64MultiArray>(
          "ultron/joint_states_acceleration", 10);

  using namespace pinocchio;
  const std::string urdf_filename = URDF_FILE;
  // Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;
  // Create data required by the algorithms
  Data data(model);
  const int JOINT_ID = 6;
  vector_t x(6);
  double index = 0;
  double period,last_time,now_time,start_time;
  last_time = ros::Time::now().toSec();
  start_time = ros::Time::now().toSec();
  Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Identity();
  Kp.diagonal().head<3>().array() = 50.0;
  Kp.diagonal().tail<3>().array() = 5.0;
  Eigen::Matrix<double, 6, 6> Kd = Eigen::Matrix<double, 6, 6>::Identity();
  Kd.diagonal().head<3>().array() = 0.5;
  Kd.diagonal().tail<3>().array() = 0.1;

  while (ros::ok()) {
    if (real_robot) {
      arx_real.get_curr_pos(q, v);
    }
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model, data, q, v);
    Eigen::Matrix<double, 6, 6> jac;
    pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::WORLD, jac);
    pinocchio::SE3 X_des;
    Vector6d err;
    if(new_target){
      // lerp to target
      double s = index/500.0;
      Eigen::Vector3d p_start = oMdes_last.translation();
      Eigen::Vector3d p_end = oMdes.translation();
      Eigen::Matrix3d R_start = oMdes_last.rotation();
      Eigen::Matrix3d R_end = oMdes.rotation();
      Eigen::Vector3d p = p_start + s*(p_end - p_start);
      Eigen::Matrix3d R = R_start * pinocchio::exp3(pinocchio::log3(R_start.transpose()*R_end) * s);
      X_des = pinocchio::SE3(R,p);
      
      // auto se3_err = pinocchio::log6(oMdes.actInv(oMdes_last))*index/5000.0;
      // X_des = oMdes_last*pinocchio::exp6(se3_err);
      // const pinocchio::SE3 dMi = X_des.actInv(data.oMi[JOINT_ID]);
      // err = pinocchio::log6(dMi).toVector();
      auto oMi_measure = data.oMi[JOINT_ID];
      Eigen::Matrix3d rot_err = oMi_measure.rotation() * X_des.rotation().transpose();
      Eigen::AngleAxisd angle_axis(rot_err);
      Eigen::Vector3d angle = angle_axis.angle() * angle_axis.axis();
      err.head(3) = oMi_measure.translation()-X_des.translation();
      err.tail(3) = angle;

      index++;
      if(index==500){
        index = 0;
        oMdes_last = oMdes;
        new_target = false;
      } 
    }else{
      auto oMi_measure = data.oMi[JOINT_ID];
      Eigen::Matrix3d rot_err = oMi_measure.rotation() * oMdes.rotation().transpose();
      Eigen::AngleAxisd angle_axis(rot_err);
      Eigen::Vector3d angle = angle_axis.angle() * angle_axis.axis();
      // const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
      // err = pinocchio::log6(dMi).toVector();
      err.head(3) = oMi_measure.translation()-oMdes.translation();
      err.tail(3) = angle;
    }
    
    now_time = ros::Time::now().toSec();
    // print now_time
    // std::cout << "now_time: " << now_time << std::endl;
    period = now_time - last_time;
    last_time = now_time;
    // std::cout << "period: " << period << std::endl;

    Eigen::VectorXd tau =
        data.nle - jac.transpose() * (Kp * err + Kd * (jac * v));
    // Eigen::VectorXd tau = data.nle;
    // Publish the joints effort
    if (real_robot) {
      arx_real.set_joints_torque(tau);
    } else {
      std_msgs::Float64 joint1_msg;
      joint1_msg.data = tau(0);
      joint1_pub.publish(joint1_msg);
      std_msgs::Float64 joint2_msg;
      joint2_msg.data = tau(1);
      joint2_pub.publish(joint2_msg);
      std_msgs::Float64 joint3_msg;
      joint3_msg.data = tau(2);
      joint3_pub.publish(joint3_msg);
      std_msgs::Float64 joint4_msg;
      joint4_msg.data = tau(3);
      joint4_pub.publish(joint4_msg);
      std_msgs::Float64 joint5_msg;
      joint5_msg.data = tau(4);
      joint5_pub.publish(joint5_msg);
      std_msgs::Float64 joint6_msg;
      joint6_msg.data = tau(5);
      joint6_pub.publish(joint6_msg);
    }

    // Publish the joints acceleration
    std_msgs::Float64MultiArray joint_acceleration_msg;
    for (int i = 0; i < 6; i++) {
      joint_acceleration_msg.data.push_back(
          ultron_arm_now.joints[i].acceleration);
    }
    joint_acceleration_pub.publish(joint_acceleration_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}