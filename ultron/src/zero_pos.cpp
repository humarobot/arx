#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

int main(int argc, char** argv){
  using namespace pinocchio;
  Model model;
  pinocchio::urdf::buildModel(URDF_FILE,model);
  std::cout << "model name: " << model.name << std::endl;
  Data data(model);
  Eigen::VectorXd q = neutral(model);
  std::cout << "q: " << q.transpose() << std::endl;
  forwardKinematics(model,data,q);
  const int JOINT_ID = 6;
  std::cout<<data.oMi[JOINT_ID].translation().transpose()<<std::endl;  //0.12 0.0 0.16
}