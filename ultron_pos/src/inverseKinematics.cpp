#include "inverseKinematics.hpp"

InverseKinematics::InverseKinematics(std::string urdf_path){
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);
}

bool InverseKinematics::Compute(const pinocchio::SE3 &oMdes, Vector6d &q) {
  Vector6d err;
  const int JOINT_ID = 6;
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;
  bool success = false;
  Eigen::VectorXd v(model_.nv);
  pinocchio::Data::Matrix6x J(6, model_.nv);
  
  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model_, data_, q);
    const pinocchio::SE3 dMi = oMdes.actInv(data_.oMi[JOINT_ID]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps) {
      return true;
    }
    if (i >= IT_MAX) {
      return false;
    }
    pinocchio::computeJointJacobian(model_, data_, q, JOINT_ID, J);
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model_, q, v * DT);
    q_last_ = q;
    // if (!(i % 30))
    //   std::cout << i << ": error = " << err.transpose() << std::endl;
  }
}

Vector6d InverseKinematics::GetJointsVelocity(const Vector6d& V){
  Vector6d qd;
  pinocchio::Data::Matrix6x J(6, model_.nv);
  pinocchio::computeJointJacobian(model_, data_, q_last_, 6, J);
  pinocchio::Data::Matrix6 JJt;
  JJt.noalias() = J * J.transpose();
  qd = J.transpose() * JJt.ldlt().solve(V);
  std::cout << "qd: " << qd.transpose() << std::endl;
  return qd;
}