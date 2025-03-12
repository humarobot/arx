#include <chrono>
#include <thread>

#include "communicator.hpp"

#include "inverseKinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "robotPinocchioModel.hpp"
#include "ros/ros.h"
#include "trajectoryLoader.hpp"

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ultron");
  ros::NodeHandle nh;
  ros::Rate loop_rate(500);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  using namespace pinocchio;
  const std::string urdf_filename = URDF_FILE;
  const int JOINT_ID = 6;

  // add a mutex to protect q,v,tau
  std::mutex qvt_mtx;
  Vector6d q, v, tau;
  q.setZero();
  v.setZero();
  tau.setZero();

  Vector6d lastq;
  lastq.setZero();
  int zerocount_ = 0;

  TrajectoryLoader traj_loader;
  Communicator communicator(nh, traj_loader, RobotType::simMujoco);
  RobotPinocchioModel robot_pino(std::string{URDF_FILE});
  InverseKinematics ik(std::string{URDF_FILE});

  // Set init q,v to current robot state
  // Waiting for communicator to get current state, delay 10ms using chrono
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::unique_lock<std::mutex> lock(communicator.arm_state_mtx_);
  auto arm_state = communicator.GetArmStateNow();
  lock.unlock();
  std::unique_lock<std::mutex> lock2(qvt_mtx);
  q = arm_state.q;
  lock2.unlock();

  double cal_time = 0.0;
  int counter_ = 0;
  Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Identity();
  Kp.diagonal().head<3>().array() = 50.0;
  Kp.diagonal().tail<3>().array() = 5.0;
  Eigen::Matrix<double, 6, 6> Kd = Eigen::Matrix<double, 6, 6>::Identity();
  Kd.diagonal().head<3>().array() = 0.5;
  Kd.diagonal().tail<3>().array() = 0.1;
  Vector6d err;
  pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.08, 0., 0.36));
  double period = 3;
  int counter = 0, counter2 = 0;
  Vector6d last_q_cmd;
  Vector6d q_cmd, v_cmd;
  Vector6d target_joint_pos = Vector6d::Zero();
  double gait_period = 0.7;
  target_joint_pos << 0.0, 0.5, 2.5, 0.0, 0.0, 0.0;

  // print q
  std::cout << "q init: " << q.transpose() << std::endl;

  // * Create a thread to communicate with robot
  std::thread commu_thread([&]() {
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(communicator.arm_state_mtx_);
        arm_state = communicator.GetArmStateNow();
      }
      // auto t1 = std::chrono::steady_clock::now();
      pinocchio::forwardKinematics(robot_pino.Model(), robot_pino.Data(), arm_state.q);
      pinocchio::computeJointJacobians(robot_pino.Model(), robot_pino.Data());
      pinocchio::updateFramePlacements(robot_pino.Model(), robot_pino.Data());
      pinocchio::crba(robot_pino.Model(), robot_pino.Data(), arm_state.q);
      pinocchio::nonLinearEffects(robot_pino.Model(), robot_pino.Data(), arm_state.q, arm_state.v);
      auto tau_ff = robot_pino.Data().nle;
      auto oMee = robot_pino.Data().oMf[robot_pino.Model().getFrameId("link6")];
      // auto t2 = std::chrono::steady_clock::now();
      // cal_time += std::chrono::duration<double>(t2 - t1).count();
      // counter_++;
      // if (counter_ == 1000) {
      //   std::cout << "Average calculation time: " << cal_time / counter_ << std::endl;
      //   counter_ = 0;
      //   cal_time = 0.0;
      // }

      communicator.PublishEEPose(oMee);
      {
        std::lock_guard<std::mutex> lock(qvt_mtx);
        communicator.SendRecvOnce(q, v, tau_ff);
      }
      loop_rate.sleep();
    }
  });

  // * Create a thread to do the control
  std::thread control_thread([&]() {
    while (ros::ok()) {
      if (communicator.attack_step_ == 1) {
        if (counter < period * 500) {
          counter++;
          q_cmd = counter / period / 500 * target_joint_pos;
          v_cmd = target_joint_pos / period;
          last_q_cmd = q_cmd;
        }
      } else if (communicator.attack_step_ == 2) {
        if (counter >= period * 500) {
          counter2++;
          q_cmd = target_joint_pos;
          double sin_phase = sin(2 * 3.1415926 * counter2 / 500 / gait_period);
          double sin_phase2 = sin(4 * 3.1415926 * counter2 / 500 / gait_period);
          q_cmd(0) += 1.0 * sin_phase;
          q_cmd(1) += 0.5 * sin_phase2;
          last_q_cmd = q_cmd;
          v_cmd.setZero();
        }
      } else if (communicator.attack_step_ == 3) {
        counter2 = 0;
        last_q_cmd = q_cmd;
      } else if (communicator.attack_step_ == 4) {
        if (counter > 0) {
          counter--;
          q_cmd = last_q_cmd * counter / period / 500;
          v_cmd = -last_q_cmd / period;
        }
      }

      {
        std::lock_guard<std::mutex> lock(qvt_mtx);
        q = q_cmd;
        v = v_cmd;
      }

      loop_rate.sleep();
    }
  });

  commu_thread.join();
  control_thread.join();

  ros::waitForShutdown();

  return 0;
}