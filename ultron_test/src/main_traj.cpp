#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "App/arm_control.h"
#include "Eigen/Core"
#include "geometry_msgs/Pose.h"
#include "iostream"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath>

/**
 * Robot work space
 * face down: x: 0.05~0.4, y: -0.4~0.4, z: -0.2~0.2
 */

pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(),
                     Eigen::Vector3d(0.3, 0.0, 0.01));
bool new_target = true;
bool real_robot = false;
int joint_index = 0;

int main(int argc, char **argv) {
  // Init ros node
  ros::init(argc, argv, "ultron");
  ros::NodeHandle nh;
  ros::Rate loop_rate(500);
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

  arx_arm arx_real(0);
  using namespace pinocchio;
  const std::string urdf_filename = URDF_FILE;
  // std::cout<<URDF_FILE<<std::endl;
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
  Eigen::VectorXd joints_upper_limit(6);
  Eigen::VectorXd joints_lower_limit(6);
  joints_upper_limit << 1.57, 3.14, 3.14, 1.5, 1.5, 10.0;
  joints_lower_limit << -1.57, 0.0, 0.0, -1.5, -1.5, -10.0;

  const int NUM_POINTS = 500;
  const double PI = 3.14159265358979323846;

  double x, y, t;
  double points[NUM_POINTS][2];

  for (int i = 0; i < NUM_POINTS; i++) {
    t = 2 * PI * i / NUM_POINTS;
    x = 16 * pow(sin(t), 3);
    y = 13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t);
    points[i][0] = x * 0.011;
    points[i][1] = y * 0.011+0.3;
  }
  int point_index = 0;
  // Main loop
  while (ros::ok()) {
    // if (real_robot == true) {
    //   arx_real.get_curr_pos();
    // }

    if (point_index<NUM_POINTS) {
      oMdes.translation() = Eigen::Vector3d(0.3,points[point_index][0], points[point_index][1]);
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
        if (!(i % 30))
          std::cout << i << ": error = " << err.transpose() << std::endl;
      }

      if (success) {
        // if q is within the joint limits
        if ((q.array() < joints_upper_limit.array()).all() &&
            (q.array() > joints_lower_limit.array()).all()) {
          ROS_INFO("\033[32m Convergence achieved! \033[0m");
          ROS_INFO("\033[32m Joint position: [%f,%f,%f,%f,%f,%f] \033[0m", q(0),
                   q(1), q(2), q(3), q(4), q(5));
        } else {
          // set q to the joint limits
          for (int i = 0; i < 6; i++) {
            if (q(i) > joints_upper_limit(i)) {
              q(i) = joints_upper_limit(i);
            } else if (q(i) < joints_lower_limit(i)) {
              q(i) = joints_lower_limit(i);
            }
          }
          ROS_INFO("\033[33m Joint position out of range! \033[0m");
          ROS_INFO("\033[33m Joint position: [%f,%f,%f,%f,%f,%f] \033[0m", q(0),
                   q(1), q(2), q(3), q(4), q(5));
        }
      } else {
        ROS_INFO("\033[31m Warning: the iterative algorithm has not reached "
                 "convergence to the desired precision \033[0m");
      }
      joint_index = 0;
      new_target = false;
    } else{
      point_index = 0;
    }
    // Publish joint topic

    if (real_robot) {
      auto q_cmd = (q - q_last) / 500.0 * joint_index + q_last;
      arx_real.set_joints_pos(q_cmd);
    } else {
      std_msgs::Float64 joint1_msg;
      std_msgs::Float64 joint2_msg;
      std_msgs::Float64 joint3_msg;
      std_msgs::Float64 joint4_msg;
      std_msgs::Float64 joint5_msg;
      std_msgs::Float64 joint6_msg;
      joint1_msg.data = q[0];
      joint2_msg.data = q[1];
      joint3_msg.data = q[2];
      joint4_msg.data = q[3];
      joint5_msg.data = q[4];
      joint6_msg.data = q[5];
      joint1_pub.publish(joint1_msg);
      joint2_pub.publish(joint2_msg);
      joint3_pub.publish(joint3_msg);
      joint4_pub.publish(joint4_msg);
      joint5_pub.publish(joint5_msg);
      joint6_pub.publish(joint6_msg);
    }
    point_index++;

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}