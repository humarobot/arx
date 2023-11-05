#include <chrono>
#include <thread>

#include "HermiteSpline.hpp"
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
    // Define a spline
    std::vector<KnotPoint> knots_fw(3);
    knots_fw[0].position << 0.1, 0, 0.16;
    knots_fw[0].velocity << 0, 0, 0;
    knots_fw[1].position << 0.3, 0, 0.3;
    knots_fw[1].velocity << 0., 0, 0;
    knots_fw[2].position << 0.4, 0., 0.2;
    knots_fw[2].velocity << 0, 0, 0;

    std::vector<KnotPoint> knots_bw(3);
    knots_bw[0].position << 0.4, 0., 0.2;
    knots_bw[0].velocity << 0, 0, 0;
    knots_bw[1].position << 0.3, 0, 0.3;
    knots_bw[1].velocity << -0, 0, 0;
    knots_bw[2].position << 0.1, 0, 0.16;
    knots_bw[2].velocity << 0, 0, 0;

    double t_max{3.0};
    HermiteSpline hermite_spline_fw{knots_fw, t_max};
    HermiteSpline hermite_spline_bw{knots_bw, t_max};

    Vector6d q_target = pinocchio::neutral(robot_pino.Model());
    Vector6d v_target = Vector6d::Zero();
    double t{0.0};
    while (ros::ok()) {
      Vector3d posDes = hermite_spline_fw.getPosition(t);
      Vector3d velDes = hermite_spline_fw.getVelocity(t);
      pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), posDes);
      Vector6d V;
      V << velDes, Vector3d::Zero(); 
      ik.Compute(oMdes, q_target);
      v_target = ik.GetJointsVelocity(V);
      {
        std::lock_guard<std::mutex> lock(qvt_mtx);
        q = q_target;
        v = v_target;
      }
      t+=0.002;
      if(t>=t_max){
        t=0.0;
        std::swap(hermite_spline_fw,hermite_spline_bw);
      }
      loop_rate.sleep();
    }
  });

  commu_thread.join();
  control_thread.join();

  ros::waitForShutdown();

  return 0;
}