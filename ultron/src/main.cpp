#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "iostream"
#include "Eigen/Core"

pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(),
                           Eigen::Vector3d(0.3, 0.1, 0.3));
bool new_target = true;
int joint_index = 0;

void poseCallback(const geometry_msgs::Pose::ConstPtr &msg) {
  ROS_INFO("End effector target position: [%f,%f,%f]", msg->position.x,
           msg->position.y, msg->position.z);
  ROS_INFO("End effector target orientation: [%f,%f,%f]", msg->orientation.x,
           msg->orientation.y, msg->orientation.z, msg->orientation.w);
  new_target = true;
  
  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x,
                       msg->orientation.y, msg->orientation.z);
  //quaternion to rotation matrix
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  Eigen::Vector3d t(msg->position.x, msg->position.y, msg->position.z);
  oMdes = pinocchio::SE3(R, t);

}

int main(int argc, char **argv) {
  // Init ros node
  ros::init(argc, argv, "ultron");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  // Init a pose topic subscriber
  ros::Subscriber pose_sub = nh.subscribe("ultron/pose", 100, poseCallback);
  // Init a joint topic publisher
  ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint1_position_controller/command", 1);
  ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint2_position_controller/command", 1);
  ros::Publisher joint3_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint3_position_controller/command", 1);
  ros::Publisher joint4_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint4_position_controller/command", 1);
  ros::Publisher joint5_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint5_position_controller/command", 1);
  ros::Publisher joint6_pub = nh.advertise<std_msgs::Float64>(
      "/ultron/joint6_position_controller/command", 1);

  using namespace pinocchio;
  const std::string urdf_filename =
      "/home/lqk/Ultron-motor/src/ultron/urdf/ultron.urdf";
  // Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;
  // Create data required by the algorithms
  Data data(model);
  const int JOINT_ID = 6;
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  Eigen::VectorXd q = pinocchio::neutral(model);
  std::cout << "q: " << q.transpose() << std::endl;
  Eigen::VectorXd q_last = q;

  // Main loop
  while (ros::ok()) {
    if (new_target == true) {
      q_last = q;
      bool success = false;
      typedef Eigen::Matrix<double, 6, 1> Vector6d;
      Vector6d err;
      Eigen::VectorXd v(model.nv);
      for (int i = 0;; i++) {
        pinocchio::forwardKinematics(model, data, q);
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
        err = pinocchio::log6(dMi).toVector();
        if (err.norm() < eps) {
          success = true;
          break;
        }
        if (i >= IT_MAX) {
          success = false;
          break;
        }
        pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(model, q, v * DT);
        if (!(i % 10))
          std::cout << i << ": error = " << err.transpose() << std::endl;
      }

      if (success) {
        std::cout << "Convergence achieved!" << std::endl;
      } else {
        std::cout << "\nWarning: the iterative algorithm has not reached "
                     "convergence to the desired precision"
                  << std::endl;
      }
      joint_index=0;
      new_target = false;
    }
    // Publish joint topic
    if(joint_index<100){
      std_msgs::Float64 joint1_msg;
      std_msgs::Float64 joint2_msg;
      std_msgs::Float64 joint3_msg;
      std_msgs::Float64 joint4_msg;
      std_msgs::Float64 joint5_msg;
      std_msgs::Float64 joint6_msg;
      joint1_msg.data = (q[0]-q_last[0])/100.0*joint_index+q_last[0];
      joint2_msg.data = (q[1]-q_last[1])/100.0*joint_index+q_last[1];
      joint3_msg.data = (q[2]-q_last[2])/100.0*joint_index+q_last[2];
      joint4_msg.data = (q[3]-q_last[3])/100.0*joint_index+q_last[3];
      joint5_msg.data = (q[4]-q_last[4])/100.0*joint_index+q_last[4];
      joint6_msg.data = (q[5]-q_last[5])/100.0*joint_index+q_last[5];
      joint1_pub.publish(joint1_msg);
      joint2_pub.publish(joint2_msg);
      joint3_pub.publish(joint3_msg);
      joint4_pub.publish(joint4_msg);
      joint5_pub.publish(joint5_msg);
      joint6_pub.publish(joint6_msg);
      joint_index++;
    }
    
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}