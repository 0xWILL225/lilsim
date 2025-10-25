#pragma once
#include <Eigen/Dense>

namespace common {
struct SE2 {
  Eigen::Isometry2d T = Eigen::Isometry2d::Identity();
  SE2() = default;
  SE2(double x, double y, double yaw) { setFromXYYaw(x,y,yaw); }
  explicit SE2(const Eigen::Isometry2d& t) : T(t) {}

  inline void setFromXYYaw(double x, double y, double yaw) {
    T.setIdentity();
    const double c = std::cos(yaw), s = std::sin(yaw);
    Eigen::Matrix2d R; R << c,-s, s, c; T.linear() = R;
    T.translation() = Eigen::Vector2d(x,y);
  }
  inline double x()   const { return T.translation().x(); }
  inline double y()   const { return T.translation().y(); }
  inline double yaw() const { return std::atan2(T.linear()(1,0), T.linear()(0,0)); }

  inline SE2 inverse() const { return SE2{T.inverse()}; }
  inline SE2 operator*(const SE2& rhs) const { return SE2{T * rhs.T}; }
};
} // namespace common
