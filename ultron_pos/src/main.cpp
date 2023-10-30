#include <thread>

#include "communicator.hpp"
#include "interpolation.hpp"
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
  InverseKinematics ik(std::string{URDF_FILE});

  // * Create a thread to communicate with robot
  std::thread commu_thread([&]() {
    while (ros::ok()) {
      RobotArm arm_state;
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
      Vector6d q_target = pinocchio::neutral(robot_pino.Model());

      // Get the target from communicator
      if (communicator.HasNewTarget()) {
        std::cout << "Processing new target" << std::endl;
        pinocchio::SE3 ee_target;
        {
          std::lock_guard<std::mutex> lock(communicator.ee_target_mtx_);
          ee_target = communicator.GetEETarget();
        }
        std::cout << "Translation:" << ee_target.translation().transpose() << std::endl;
        std::cout << "Rotation:" << std::endl << ee_target.rotation() << std::endl;
        ik.Compute(ee_target, q_target);
        communicator.ResetNewTarget();

        // Linear interpolation
        double t = 0.0;
        double t_max = 2.0;
        double dt = 0.002;
        Vector6d q_cmd, v_cmd;
        auto q_last = ik.GetLastQ();
        Interpolation<Trapezoidal> interpolator{Vector6d::Constant(2.0), t_max, q_last, q_target};
        std::cout << "is feasible: " << interpolator.isFeasible() << std::endl;
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
          t = 0.0;
          ik.SetLastQ(q_target);
        }
      }

      loop_rate.sleep();
    }
  });

  commu_thread.join();
  control_thread.join();

  ros::waitForShutdown();

  return 0;
}