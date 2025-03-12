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
      // robot arm move to trajectory initial position
      // Linear interpolation
      bool success = false;
      double damp = 1e-6;
      double DT = 1e-1;
      int IT_MAX = 1000;
      Vector6d q_cmd, v_cmd;
      // q_cmd = arm_state.q;
      q_cmd = pinocchio::neutral(robot_pino.Model());
      for (int i = 0;; i++) {
        pinocchio::forwardKinematics(robot_pino.Model(), robot_pino.Data(), q_cmd);
        const pinocchio::SE3 dMi = communicator.GetEETarget().actInv(robot_pino.Data().oMi[JOINT_ID]);
        err = pinocchio::log6(dMi).toVector();
        if (err.norm() < 1e-4) {
          success = true;
          break;
        }
        if (i >= 1000) {
          success = false;
          break;
        }
        Eigen::Matrix<double, 6, 6> J;
        Eigen::VectorXd delta(6);
        pinocchio::computeJointJacobian(robot_pino.Model(), robot_pino.Data(), q_cmd, JOINT_ID, J);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        delta.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        q_cmd = pinocchio::integrate(robot_pino.Model(), q_cmd, delta * DT);
        // if (!(i % 50)) std::cout << i << ": error = " << err.transpose() << std::endl;
      }
      // std::cout << "oMi_measure.translation():" << oMi_measure.translation().transpose() << std::endl;
      // std::cout << "oMdes.translation():" << oMdes.translation().transpose() << std::endl;
      // std::cout << "err" << err.transpose() << std::endl;
      // std::cout << "tau_cmd:" << tau_cmd.transpose() << std::endl;

      {
        std::lock_guard<std::mutex> lock(qvt_mtx);
        if(!communicator.HasZeroFlag()) {
          zerocount_ = 0;
          lastq = arm_state.q;
          
          // protect huge change, joint3, joint4, joint5
          if((q_cmd - arm_state.q).norm() > 1.0 || q_cmd[2] > 2.5 || q_cmd[3] > 1.2 || q_cmd[3] < -1.2 || q_cmd[4] > 1.5 || q_cmd[4] < -1.5) 
            q = arm_state.q;
          else
            q = q_cmd;

        }
        else {
          // move slowly to zero
          if(zerocount_ < 1500)
            ++zerocount_;
          q = lastq * (1.0 - (double)zerocount_ / 1500.0);
        }
        // v = v_cmd;
        // tau = tau_cmd;
      }
      loop_rate.sleep();
    }
  });

  commu_thread.join();
  control_thread.join();

  ros::waitForShutdown();

  return 0;
}