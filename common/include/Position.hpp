#pragma once

namespace common {

/**
 * @brief Simple 2D position (x, y)
 */
struct Position {
  double x{0.0};
  double y{0.0};

  Position() = default;
  Position(double x_, double y_)
    : x(x_)
    , y(y_) {
  }
};

} // namespace common

