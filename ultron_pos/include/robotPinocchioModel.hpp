#pragma once


#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <string>
#include "typeAlias.hpp"

class RobotPinocchioModel {
public:
  RobotPinocchioModel(std::string urdf_path);
  ~RobotPinocchioModel() = default;
  pinocchio::Model& Model() { return model_; }
  pinocchio::Data& Data() { return data_; }
private:
  pinocchio::Model model_;
  pinocchio::Data data_;

};