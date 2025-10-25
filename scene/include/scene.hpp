#pragma once

#include "SE2.hpp"

#include <atomic>

namespace scene {

struct CarState {

  CarState(double wheelbase_ = 2.6, double Lf_ = 1.3)
  : wheelbase(wheelbase_), // distance between axles
    Lf(Lf_),               // front axle to center of gravity
    Lr(wheelbase_ - Lf_)   // rear axle to center of gravity
  {
    assert(Lf > 0.0 && Lf < wheelbase && Lr > 0.0 && Lr < wheelbase);
  }

  double wheelbase;
  double Lf;
  double Lr;

  common::SE2 pose;
  double v{0.0};
  double yaw_rate{0.0};

  double x() const { return pose.x(); }
  double y() const { return pose.y(); }
  double yaw() const { return pose.yaw(); }

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
