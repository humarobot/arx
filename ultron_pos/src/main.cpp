#include <thread>

#include "communicator.hpp"
#include "inverseKinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "robotPinocchioModel.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ultron");
  ros::NodeHandle nh;
  ros::Rate loop_rate(500);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // add a mutex to protect q,v,tau
  std::mutex qvt_mtx;
  Vector6d q, v, tau;
  q.setZero();
  v.setZero();
  tau.setZero();

  Communicator communicator(nh, RobotType::sim);
  RobotPinocchioModel robot_pino(std::string{URDF_FILE});
  InverseKinematics ik(robot_pino);

  // Create a thread to communicate with robot
  std::thread commu_thread([&]() {
    while (ros::ok()) {
      RobotArm arm_state;
      {
        std::lock_guard<std::mutex> lock(communicator.arm_state_mtx_);
        arm_state = communicator.arm_state_now_;
      }
      pinocchio::nonLinearEffects(robot_pino.Model(), robot_pino.Data(), arm_state.q, arm_state.v);
      auto tau_ff = robot_pino.Data().nle;
      {
        std::lock_guard<std::mutex> lock(qvt_mtx);
        communicator.SendRecvOnce(q, v, tau_ff);
      }
      loop_rate.sleep();
    }
  });

  // Create a thread to do the control
  std::thread control_thread([&]() {
    while (ros::ok()) {
      Vector6d q_target = pinocchio::neutral(robot_pino.Model());
      pinocchio::SE3 ee_target;
      {
        std::lock_guard<std::mutex> lock(communicator.ee_target_mtx_);
        ee_target = communicator.GetEETarget();
      }
      ik.Compute(ee_target, q_target);
      {
        std::lock_guard<std::mutex> lock(qvt_mtx);
        q = q_target;
      }
      loop_rate.sleep();
    }
  });

  commu_thread.join();
  control_thread.join();

  ros::waitForShutdown();

  return 0;
}