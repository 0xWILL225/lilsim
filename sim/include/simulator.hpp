#pragma once
#include <atomic>
#include <thread>

#include "scene.hpp"

namespace sim {

struct CarInput { 
  double delta{0.0}; // steering angle
  double ax{0.0};    // longitudinal acceleration
};

class Simulator {
public:
  explicit Simulator(scene::SceneDB& db) : db_(db) {}
  ~Simulator() { stop(); }

  void start(double dt);
  void stop();
  void setInput(CarInput const& u) { input_.store(u, std::memory_order_relaxed); }

private:
  void loop(double dt);

  scene::SceneDB& db_;
  std::atomic<bool> running_{false};
  std::thread th_;
  std::atomic<CarInput> input_;
  scene::Scene state_; // reuse Scene as holder for car state
};

} // namespace sim
