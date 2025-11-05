#pragma once

#include "scene.hpp"
#include "SE2.hpp"
#include <string>
#include <vector>
#include <optional>

namespace scene {

/**
 * @brief Track data loaded from CSV.
 */
struct TrackData {
  std::vector<Cone> cones;
  std::optional<common::SE2> startPose; // Starting pose if car_start is present
  std::vector<common::SE2> midpoints;   // Midpoint positions (for midline visualization)
};

/**
 * @brief Utility to load track data from CSV files.
 *
 * CSV format: tag,x,y,yaw
 * Supported tags: blue, yellow, orange, big_orange, car_start, midpoint
 */
class TrackLoader {
public:
  /**
   * @brief Load track data from a CSV file.
   *
   * @param filepath Path to the CSV file
   * @param outTrackData TrackData to populate with cones and starting pose
   * @return true if successful, false on error
   */
  static bool loadFromCSV(const std::string& filepath, TrackData& outTrackData);

  /**
   * @brief Parse cone type from tag string.
   */
  static ConeType parseConeType(const std::string& tag);
};

} // namespace scene

