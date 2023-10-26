#include "pinocchio/algorithm/rnea.hpp"
#include "inverseKinematics.hpp"
#include "robotPinocchioModel.hpp"
#include "communicator.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ultron");
  ros::NodeHandle nh;
  ros::Rate loop_rate(500);
  Vector6d q, v, tau;
  q << 0, 0.7, 0.7, 0.3, 0.3, 0.3;
  v.setZero();
  tau.setZero();

  Communicator communicator(nh, RobotType::sim);
  RobotPinocchioModel robot_pino(std::string{URDF_FILE});
  InverseKinematics ik(robot_pino);
  pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(),
                       Eigen::Vector3d(0.3, 0.0, 0.3));
  Vector6d q_target = pinocchio::neutral(robot_pino.Model());
  ik.Compute(oMdes, q_target);
  std::cout<<"q_target: "<<q_target.transpose()<<std::endl;

  while (ros::ok()) {
    auto arm_state = communicator.GetArmStateNow();
    pinocchio::nonLinearEffects(robot_pino.Model(), robot_pino.Data(), arm_state.q, arm_state.v);
    tau = robot_pino.Data().nle;
    communicator.SendRecvOnce(q_target, v, tau);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}