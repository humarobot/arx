#include "inverseKinematics.hpp"

InverseKinematics::InverseKinematics(RobotPinocchioModel &robot_pino)
    : robot_pino_(robot_pino) {}

bool InverseKinematics::Compute(const pinocchio::SE3& oMdes, Vector6d& q) {
  pinocchio::Model &model = robot_pino_.Model();
  pinocchio::Data &data = robot_pino_.Data();
  Vector6d err;
  const int JOINT_ID = 6;
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;
  bool success = false;
  Eigen::VectorXd v(model.nv);
  pinocchio::Data::Matrix6x J(6, model.nv);

  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model, data, q);
    const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= IT_MAX) {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);
    if (!(i % 30))
      std::cout << i << ": error = " << err.transpose() << std::endl;
  }
}