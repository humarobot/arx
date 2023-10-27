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
  void SetLastQ(const Vector6d& q) { q_last_ = q; }
  Vector6d GetLastQ() const { return q_last_; }

private:
  Vector6d q_last_{Vector6d::Zero()};
  RobotPinocchioModel& robot_pino_;
};
