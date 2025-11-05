#pragma once
#include <atomic>
#include <thread>
#include <optional>

#include "CarDefaults.hpp"
#include "scene.hpp"
#include "SE2.hpp"

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
  void setCones(const std::vector<scene::Cone>& cones) {
    m_conesUpdateRequested.store(true, std::memory_order_relaxed);
    m_newCones = cones;
  }
  void setStartPose(const common::SE2& pose) {
    m_startPoseUpdateRequested.store(true, std::memory_order_relaxed);
    m_newStartPose = pose;
  }

private:
  void loop(double dt);

  struct ResetParams {
    double wheelbase{common::CarDefaults::wheelbase};
    double v_max{common::CarDefaults::v_max};
    double delta_max{common::CarDefaults::delta_max};
    double dt{common::CarDefaults::dt};
  };

  scene::SceneDB& m_db;
  std::atomic<bool> m_running{false};
  std::atomic<bool> m_paused{false};
  std::atomic<bool> m_resetRequested{false};
  std::atomic<bool> m_conesUpdateRequested{false};
  std::atomic<bool> m_startPoseUpdateRequested{false};
  std::atomic<uint64_t> m_stepTarget{0}; // 0 = run continuously, >0 = step mode
  std::thread m_thread;
  std::atomic<CarInput> m_input;
  scene::Scene m_state; // reuse Scene as holder for car state
  double m_dt{0.0};     // timestep, set by start()
  ResetParams m_newParams;
  std::vector<scene::Cone> m_newCones;
  common::SE2 m_newStartPose{0.0, 0.0, 0.0};
  common::SE2 m_startPose{0.0, 0.0, 0.0}; // Current starting pose
};

} // namespace sim
