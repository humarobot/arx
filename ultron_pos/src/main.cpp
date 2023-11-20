#include <chrono>
#include <thread>

#include "HermiteSpline.hpp"
#include "communicator.hpp"
#include "interpolation.hpp"
#include "inverseKinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "robotPinocchioModel.hpp"
#include "ros/ros.h"
#include "trajectoryLoader.hpp"
#include "trapezoidalSlerp.hpp"

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

  TrajectoryLoader traj_loader;
  Communicator communicator(nh, RobotType::simMujoco);
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

  // print q
  std::cout << "q init: " << q.transpose() << std::endl;

  // * Create a thread to communicate with robot
  std::thread commu_thread([&]() {
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(communicator.arm_state_mtx_);
        arm_state = communicator.GetArmStateNow();
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

  // * Create a thread to do the control
  std::thread control_thread([&]() {
    while (ros::ok()) {
      if (communicator.execPriority_ == 1 && !communicator.atInitPosi_) {
        // robot arm move to trajectory initial position
        // Linear interpolation
        double t = 0.0;
        double t_max = 2.0;
        double dt = 0.002;
        Vector6d q_cmd, v_cmd;
        auto q_traj = traj_loader.GetArmStateTrajectory();
        Vector6d q_target = q_traj.col(0);
        Interpolation<Trapezoidal> interpolator{Vector6d::Constant(2.0), t_max, q, q_target};
        if (interpolator.isFeasible()) {
          while (t < t_max) {
            q_cmd = interpolator.getPosition(t);
            v_cmd = interpolator.getVelocity(t);

            {
              std::lock_guard<std::mutex> lock(qvt_mtx);
              q = q_cmd;
              v = v_cmd;
            }
            loop_rate.sleep();
            t += dt;
          }
          communicator.atInitPosi_ = true;
        } else {
          std::cout << "Error: Trajectory is not feasible" << std::endl;
          ros::shutdown();
        }

        
      } else if (communicator.execPriority_ == 2) {
        // execute trajectory
        auto q_traj = traj_loader.GetArmStateTrajectory();
        auto v_traj = traj_loader.GetArmVelTrajectory();
        for (int i = 0; i < q_traj.cols(); i++) {
          {
            std::lock_guard<std::mutex> lock(qvt_mtx);
            q = q_traj.col(i);
            v = v_traj.col(i);
          }
          ros::Duration(0.05).sleep();
        }
        communicator.execPriority_ = 0;
        communicator.atInitPosi_ = false;
      }

      loop_rate.sleep();
    }
  });

  commu_thread.join();
  control_thread.join();

  ros::waitForShutdown();

  return 0;
}