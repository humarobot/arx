#include <Eigen/Core>
#include <iostream>

#include "trapezoidalSlerp.hpp"

int main() {
  // int a{};
  // std::cout << "a = " << a << std::endl;
  // Eigen::Matrix<double, 6, 1> test;
  // // test.setZero();
  // std::cout << "test = " << test.transpose() << std::endl;
  // Eigen::Matrix3d mat{Eigen::Matrix3d::Identity()};
  // std::cout << mat << std::endl;

  // std::vector<double> t(5,int(9));
  // //print t
  // for(auto i: t){
  //   std::cout<<i<<std::endl;
  // }

  TrapezoidalSlerp tslerp{Quat::Identity(), Quat{1, 1, 1, 0}.normalized()};
  for (double t = 0; t <= 1.0; t += 0.1) {
    auto q = tslerp.GetQuat(t);
    std::cout << "Quaternion: [" << q.w() << ", " << q.vec().transpose() << "]" << std::endl;
  }
  for (double t = 0; t <= 1.0; t += 0.1) {
    auto w = tslerp.GetOmega(t);
    std::cout << "Omega: [" << w.transpose() << "]" << std::endl;
  }
  return 0;
}