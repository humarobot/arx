#pragma once 

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "typeAlias.hpp"

class InverseKinematics {
public: 
  InverseKinematics(std::string urdf_path);
  ~InverseKinematics() = default;
  bool Compute(const pinocchio::SE3& oMdes, Vector6d& q);
  Vector6d GetJointsVelocity(const Vector6d& V);
  void SetLastQ(const Vector6d& q) { q_last_ = q; }
  Vector6d GetLastQ() const { return q_last_; }

private:
  Vector6d q_last_{Vector6d::Zero()};
  pinocchio::Model model_;
  pinocchio::Data data_;
};
