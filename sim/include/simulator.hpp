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
  explicit Simulator(scene::SceneDB& db)
    : m_db(db) {
  }
  ~Simulator() {
    stop();
  }

  void start(double dt);
  void stop();
  void setInput(CarInput const& u) {
    m_input.store(u, std::memory_order_relaxed);
  }
  void pause() {
    m_paused.store(true, std::memory_order_relaxed);
  }
  void resume() {
    m_paused.store(false, std::memory_order_relaxed);
  }
  bool isPaused() const {
    return m_paused.load(std::memory_order_relaxed);
  }
  void step(uint64_t numTicks) {
    m_stepTarget.store(numTicks, std::memory_order_relaxed);
  }
  double getDt() const {
    return m_dt;
  }
  uint64_t getTicksRemaining() const {
    return m_stepTarget.load(std::memory_order_relaxed);
  }
  void reset(double wheelbase, double v_max, double delta_max, double dt) {
    m_resetRequested.store(true, std::memory_order_relaxed);
    m_newParams.wheelbase = wheelbase;
    m_newParams.v_max = v_max;
    m_newParams.delta_max = delta_max;
    m_newParams.dt = dt;
  }

private:
  void loop(double dt);

  struct ResetParams {
    double wheelbase{2.6};
    double v_max{15.0};
    double delta_max{1.745};
    double dt{1.0 / 200.0};
  };

  scene::SceneDB& m_db;
  std::atomic<bool> m_running{false};
  std::atomic<bool> m_paused{false};
  std::atomic<bool> m_resetRequested{false};
  std::atomic<uint64_t> m_stepTarget{0}; // 0 = run continuously, >0 = step mode
  std::thread m_thread;
  std::atomic<CarInput> m_input;
  scene::Scene m_state; // reuse Scene as holder for car state
  double m_dt{0.0};     // timestep, set by start()
  ResetParams m_newParams;
};

} // namespace sim
