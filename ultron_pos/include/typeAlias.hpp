#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using VectorXd = Eigen::VectorXd;
using Quat = Eigen::Quaterniond;

template <class T>
const T& clamp(const T& v, const T& lo, const T& hi) {
    assert(!(hi < lo));
    return (v < lo) ? lo : (hi < v) ? hi : v;
}