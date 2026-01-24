#pragma once

#include <cstddef>
#include <limits>

namespace common {

/** @brief Sentinel value for invalid/unset indices. */
inline constexpr std::size_t kNullIndex = std::numeric_limits<std::size_t>::max();

}  // namespace common
