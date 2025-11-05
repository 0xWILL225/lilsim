#pragma once

#include "CarDefaults.hpp"
#include "SE2.hpp"

#include <atomic>
#include <vector>

namespace scene {

struct CarState {

  CarState(double wheelbase_ = common::CarDefaults::wheelbase,
           double Lf_ = common::CarDefaults::Lf,
           double v_max_ = common::CarDefaults::v_max,
           double delta_max_ = common::CarDefaults::delta_max)
    : wheelbase(wheelbase_)
    , Lf(Lf_)
    , Lr(wheelbase_ - Lf_)
    , v_max(v_max_)
    , delta_max(delta_max_) {
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

enum class ConeType { Blue, Yellow, Orange, BigOrange };

struct Cone {
  double x{0.0};
  double y{0.0};
  ConeType type{ConeType::Blue};

  Cone() = default;
  Cone(double x_, double y_, ConeType type_)
    : x(x_)
    , y(y_)
    , type(type_) {
  }
};

struct Scene {
  CarState car;
  std::vector<Cone> cones;
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
