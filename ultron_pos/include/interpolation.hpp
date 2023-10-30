#pragma once
#include <iostream>
#include <type_traits>

#include "typeAlias.hpp"

class Linear {};
class Trapezoidal {};

template <typename T>
class Interpolation;

template <>
class Interpolation<Linear> {
 public:
  Interpolation() = default;
  ~Interpolation() = default;
  void SetStartEnd(const Vector6d& q_start, const Vector6d& q_end, double t_max) {
    std::cout << "Linear interpolation" << std::endl;
    q_start_ = q_start;
    q_end_ = q_end;
    t_ = t_max;
  }
  void GetQ(Vector6d& q, double t_now) { q = q_start_ + (q_end_ - q_start_) * t_now / t_; }
  void GetQd(Vector6d& qd, double t_now) { qd = (q_end_ - q_start_) / t_; }
  void GetQdd(Vector6d& qdd, double t_now) { qdd.setZero(); }

 private:
  Vector6d q_start_{Vector6d::Zero()};
  Vector6d q_end_{Vector6d::Zero()};
  double t_{1.0};
};

template <>
class Interpolation<Trapezoidal> {
 public:
  Interpolation() = default;
  ~Interpolation() = default;
  Interpolation(const Eigen::VectorXd& max_acceleration, double total_time, const Eigen::VectorXd& start_pos,
                const Eigen::VectorXd& end_pos)
      : a(max_acceleration), T(total_time), theta_start(start_pos), theta_end(end_pos), num_joints(start_pos.size()) {
    delta_theta = theta_end - theta_start;
    Ta = Eigen::VectorXd::Zero(num_joints);
    Tc = Eigen::VectorXd::Zero(num_joints);
    v_max = Eigen::VectorXd::Zero(num_joints);
    is_feasible = true;
    computeParameters();
  }

  void computeParameters() {
    for (int i = 0; i < num_joints; ++i) {
      v_max(i) = 0.5*(a(i)*T - std::sqrt(a(i)*a(i)*T*T - 4*a(i)));

      if((a(i)*T*T) < 4.0){
        is_feasible = false;
      }
    }
    
  }

  Eigen::VectorXd getPosition(double t) const {
    Eigen::VectorXd position = Eigen::VectorXd::Zero(num_joints);
    for (int i = 0; i < num_joints; ++i) {
      if (t < (v_max(i)/a(i))) {  // 加速阶段
        auto s = 0.5 * a(i) * t * t;
        position(i) = theta_start(i) + s * delta_theta(i);
      } else if (t < (T-v_max(i)/a(i))) {  // 匀速阶段
        auto s = v_max(i)*t - v_max(i)*v_max(i)/(2*a(i));
        position(i) = theta_start(i) + s*delta_theta(i);
      } else {  // 减速阶段
        auto s =(2*a(i)*v_max(i)*T-2*v_max(i)*v_max(i)-a(i)*a(i)*(T-t)*(T-t))/(2*a(i));
        position(i) = theta_start(i) + s*delta_theta(i);
      }
    }
    return position;
  }

  Eigen::VectorXd getVelocity(double t) const {
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero(num_joints);
    for (int i = 0; i < num_joints; ++i) {
      if (t < (v_max(i)/a(i))) {  // 加速阶段
        velocity(i) = a(i) * t;
      } else if (t < (T-v_max(i)/a(i))) {  // 匀速阶段
        velocity(i) = v_max(i);
      } else {  // 减速阶段
        velocity(i) = a(i) * (T - t);
      }
    }
    return velocity;
  }

  Eigen::VectorXd getAcceleration(double t) const {
    Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(num_joints);
    for (int i = 0; i < num_joints; ++i) {
      if (t < Ta(i)) {  // 加速阶段
        acceleration(i) = a(i);
      } else if (t < Ta(i) + Tc(i)) {  // 匀速阶段
        acceleration(i) = 0;
      } else {  // 减速阶段
        acceleration(i) = -a(i);
      }
    }
    return acceleration;
  }

  bool isFeasible() const { return is_feasible; }

 private:
  Eigen::VectorXd a;            // 额定加速度
  double T;                     // 总时间
  Eigen::VectorXd theta_start;  // 起始位置
  Eigen::VectorXd theta_end;    // 终止位置
  Eigen::VectorXd delta_theta;  // 位置变化
  Eigen::VectorXd Ta;           // 加速阶段时间
  Eigen::VectorXd Tc;           // 匀速阶段时间
  Eigen::VectorXd v_max;        // 最大速度
  bool is_feasible;             // 是否可行
  int num_joints;               // 关节数量
};