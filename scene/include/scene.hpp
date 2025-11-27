#pragma once

#include "CarDefaults.hpp"
#include "SE2.hpp"

#include <atomic>
#include <vector>
#include <cstdint>

namespace scene {

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
  // Raw state values from the car model
  std::vector<double> car_state_values;
  
  // Raw input values used in this step
  std::vector<double> car_input_values;

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
