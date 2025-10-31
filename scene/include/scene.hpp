#pragma once

#include "SE2.hpp"

#include <atomic>

namespace scene {

struct CarState {

  CarState(double wheelbase_ = 2.6, double Lf_ = 1.3, double v_max_ = 15.0,
           double delta_max_ = 1.745) // ~100 degrees in radians
    : wheelbase(wheelbase_)
    , // distance between axles
    Lf(Lf_)
    , // front axle to center of gravity
    Lr(wheelbase_ - Lf_)
    , // rear axle to center of gravity
    v_max(v_max_)
    ,                     // maximum velocity (m/s)
    delta_max(delta_max_) // maximum steering angle (rad)
  {
    assert(Lf > 0.0 && Lf < wheelbase && Lr > 0.0 && Lr < wheelbase);
    assert(v_max > 0.0);
    assert(delta_max > 0.0);
  }

  double wheelbase;
  double Lf;
  double Lr;
  double v_max;     // maximum velocity
  double delta_max; // maximum steering angle magnitude

  common::SE2 pose;
  double v{0.0};
  double yaw_rate{0.0};

  double x() const {
    return pose.x();
  }
  double y() const {
    return pose.y();
  }
  double yaw() const {
    return pose.yaw();
  }
};

struct Scene {
  CarState car;
  // array of traffic cones here later
};

struct SceneDB {
  Scene buf[2];
  std::atomic<int> front{0};
  std::atomic<uint64_t> tick{0};

  void publish(const Scene& s) {
    int b = 1 - front.load(std::memory_order_acquire);
    buf[b] = s;
    front.store(b, std::memory_order_release);
    tick.fetch_add(1, std::memory_order_relaxed);
  }
  const Scene& snapshot() const {
    return buf[front.load(std::memory_order_acquire)];
  }
};

} // namespace scene
