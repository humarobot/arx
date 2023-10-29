#pragma once
#include <type_traits>
#include "typeAlias.hpp"
#include <iostream>

class Linear{};
class Trapezoidal{};

template <typename T, typename std::enable_if<std::is_same<T, Linear>::value>::type* = nullptr>
class Interpolation{
public:  
  Interpolation() = default;
  ~Interpolation() = default;
  void SetStartEnd(const Vector6d& q_start, const Vector6d& q_end, double t_max){
    std::cout<<"Linear interpolation"<<std::endl;
    q_start_ = q_start;
    q_end_ = q_end;
    t_ = t_max;
  }
  void GetQ(Vector6d& q, double t_now){
    q = q_start_ + (q_end_ - q_start_) * t_now / t_;
  }
  void GetQd(Vector6d& qd, double t_now){
    qd = (q_end_ - q_start_) / t_;
  }
  void GetQdd(Vector6d& qdd, double t_now){
    qdd.setZero();
  }
private:
  Vector6d q_start_{Vector6d::Zero()};
  Vector6d q_end_{Vector6d::Zero()};
  double t_{1.0};


};