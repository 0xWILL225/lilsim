#include <chrono>
#include <cmath>

#include "simulator.hpp"

namespace sim {

void Simulator::start(double dt) {
  if(running_.exchange(true)) return;
  th_ = std::thread(&Simulator::loop, this, dt);
}

void Simulator::stop() {
  if(!running_.exchange(false)) return;
  if(th_.joinable()) th_.join();
}

void Simulator::loop(double dt) {
  using clock = std::chrono::steady_clock;
  auto next = clock::now();

  state_.car.pose = common::SE2(0,0,0);
  state_.car.v = 0.0;

  while(running_.load(std::memory_order_relaxed)) {
    CarInput u = input_.load(std::memory_order_relaxed);

    // simple kinematic bicycle (minimal)
    const double yaw = state_.car.pose.yaw();
    const double beta = std::atan( (state_.car.Lr / state_.car.wheelbase) * std::tan(u.delta)); // body slip angle
    const double vx = state_.car.v;

    const double dx = vx * std::cos(yaw + beta);
    const double dy = vx * std::sin(yaw + beta);
    const double dyaw = (vx / state_.car.wheelbase) * std::tan(u.delta);
    const double dv = u.ax;

    double x = state_.car.pose.x() + dt * dx;
    double y = state_.car.pose.y() + dt * dy;
    double ynew = yaw + dt * dyaw;
    double v = state_.car.v + dt * dv;

    // update state
    state_.car.pose.setFromXYYaw(x, y, std::atan2(std::sin(ynew), std::cos(ynew)));
    state_.car.v = v;
    state_.car.yaw_rate = dyaw;

    db_.publish(state_);

    next += std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt));
    std::this_thread::sleep_until(next);
  }
}

} // namespace sim
