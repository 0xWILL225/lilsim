#pragma once

namespace common {

/**
 * @brief Default car parameters used throughout the simulation.
 *
 * These defaults are used in CarState construction and simulator reset.
 * Modify here to change defaults across the entire codebase.
 */
struct CarDefaults {
  static constexpr double wheelbase = 2.97;      // meters, distance between axles
  static constexpr double trackwidth = 1.6;      // meters, width between left and right wheels (assumed same front and rear, for now)
  static constexpr double Lf = 1.0;              // meters, front axle to center of gravity
  static constexpr double v_max = 30.0;          // m/s, maximum velocity
  static constexpr double ax_max = 20.0;         // m/s^2, maximum longitudinal acceleration
  static constexpr double Fx_max_front = 1200.0; // N, maximum front axle force
  static constexpr double Fx_max_rear = 1200.0;  // N, maximum rear axle force
  static constexpr double delta_max = 0.436;     // radians, ~25 degrees, max steering angle
  static constexpr double steer_rate_max = 6.0;  // radians/s, max steering rate
  static constexpr double dt = 1.0 / 200.0;      // seconds, default timestep (200 Hz)
  static constexpr double mass = 180.0;          // kg, mass of the car
};

} // namespace common

