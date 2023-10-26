#include "robotPinocchioModel.hpp"

RobotPinocchioModel::RobotPinocchioModel(std::string urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);
}