#pragma once

#include "typeAlias.hpp"

class TrapezoidalSlerp {
 public:
  TrapezoidalSlerp(Quat q_start, Quat q_end) : q_start_(q_start), q_end_(q_end) {}
  TrapezoidalSlerp() = default;
  ~TrapezoidalSlerp() = default;

  Quat GetQuat(double t) {
    clamp(t, 0.0, 1.0);
    double r;
    if (t < 0.5)
      r = 2 * t * t;
    else
      r = 1 - 2 * (1 - t) * (1 - t);
    return q_start_.slerp(r, q_end_);
  }
  Vector3d GetOmega(double t) {
    clamp(t, 0.0, 1.0);
    auto delta_q = q_start_.inverse() * q_end_;
    // 提取实部和虚部
    double w = delta_q.w();
    Eigen::Vector3d v = delta_q.vec();
    // 计算旋转的角度θ
    double theta = acos(w);
    // 标准化虚部向量
    Eigen::Vector3d v_norm = v.normalized();
    // 计算θv
    Eigen::Vector3d log_q = theta * v_norm;
    Vector3d omega;
    if (t < 0.5)
      omega = 4*log_q*t;
    else
      omega = 4*log_q*(1-t);
    return omega;
  }

 private:
  Quat q_start_{Quat::Identity()};
  Quat q_end_{Quat::Identity()};
};