#pragma once 

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "robotPinocchioModel.hpp"
#include "typeAlias.hpp"

class InverseKinematics {
public: 
  InverseKinematics(RobotPinocchioModel& robot_pino);
  ~InverseKinematics() = default;
  bool Compute(const pinocchio::SE3& oMdes, Vector6d& q);
private:
  RobotPinocchioModel& robot_pino_;
};
