#include <chrono>
#include <cmath>

#include "simulator.hpp"

namespace sim {

void Simulator::start(double dt) {
  if (m_running.exchange(true))
    return;
  m_dt = dt;
  m_thread = std::thread(&Simulator::loop, this, dt);
}

void Simulator::stop() {
  if (!m_running.exchange(false))
    return;
  if (m_thread.joinable())
    m_thread.join();
}

void Simulator::loop(double dt) {
  using clock = std::chrono::steady_clock;
  auto next = clock::now();

  // Lambda to initialize/reset state
  auto resetState = [&]() {
    m_state.car.pose = m_startPose;
    m_state.car.v = 0.0;
    m_state.car.yaw_rate = 0.0;

    m_db.publish(m_state);
  };

  resetState();

  while (m_running.load(std::memory_order_relaxed)) {
    // Check for start pose update request
    if (m_startPoseUpdateRequested.exchange(false, std::memory_order_relaxed)) {
      m_startPose = m_newStartPose;
    }
    
    // Check for cone update request
    if (m_conesUpdateRequested.exchange(false, std::memory_order_relaxed)) {
      m_state.cones = m_newCones;
      m_db.publish(m_state);
    }
    
    // Check for reset request
    if (m_resetRequested.exchange(false, std::memory_order_relaxed)) {
      // Apply new parameters
      m_state.car =
        scene::CarState(m_newParams.wheelbase, m_newParams.wheelbase / 2.0,
                        m_newParams.v_max, m_newParams.delta_max);
      dt = m_newParams.dt;
      m_dt = dt;
      // Reset state and tick counter
      resetState();
      m_db.tick.store(0, std::memory_order_relaxed);
      m_paused.store(true, std::memory_order_relaxed);
      next = clock::now();
      continue;
    }
    // Check if paused
    if (m_paused.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      next = clock::now(); // Reset timing when resuming
      continue;
    }

    // Check step mode
    uint64_t stepsRemaining = m_stepTarget.load(std::memory_order_relaxed);
    if (stepsRemaining > 0) {
      // Step mode: execute one tick then decrement
      m_stepTarget.fetch_sub(1, std::memory_order_relaxed);
      if (stepsRemaining == 1) {
        // Last step completed, pause
        m_paused.store(true, std::memory_order_relaxed);
      }
    }

    CarInput u = m_input.load(std::memory_order_relaxed);

    // Clamp steering angle to limits
    u.delta =
      std::clamp(u.delta, -m_state.car.delta_max, m_state.car.delta_max);

    // simple kinematic bicycle (minimal)
    const double yaw = m_state.car.pose.yaw();
    const double vx = m_state.car.v;

    const double dx = vx * std::cos(yaw);
    const double dy = vx * std::sin(yaw);
    const double dyaw = (vx / m_state.car.wheelbase) * std::tan(u.delta);
    const double dv = u.ax;

    double x = m_state.car.pose.x() + dt * dx;
    double y = m_state.car.pose.y() + dt * dy;
    double ynew = yaw + dt * dyaw;
    double v = m_state.car.v + dt * dv;

    // Clamp velocity to [0, v_max]
    v = std::clamp(v, 0.0, m_state.car.v_max);

    // update state
    m_state.car.pose.setFromXYYaw(x, y,
                                  std::atan2(std::sin(ynew), std::cos(ynew)));
    m_state.car.v = v;
    m_state.car.yaw_rate = dyaw;

    m_db.publish(m_state);

    next += std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(dt));
    std::this_thread::sleep_until(next);
  }
}

} // namespace sim
