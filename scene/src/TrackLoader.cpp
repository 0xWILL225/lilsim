#include "TrackLoader.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace scene {

ConeType TrackLoader::parseConeType(const std::string& tag) {
  if (tag == "blue") {
    return ConeType::Blue;
  } else if (tag == "yellow") {
    return ConeType::Yellow;
  } else if (tag == "orange") {
    return ConeType::Orange;
  } else if (tag == "big_orange") {
    return ConeType::BigOrange;
  }
  // Default to blue if unknown
  return ConeType::Blue;
}

bool TrackLoader::loadFromCSV(const std::string& filepath, TrackData& outTrackData) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  outTrackData.cones.clear();
  outTrackData.startPose.reset();
  outTrackData.midpoints.clear();

  std::string line;
  bool firstLine = true;

  while (std::getline(file, line)) {
    // Skip header line
    if (firstLine) {
      firstLine = false;
      continue;
    }

    // Skip empty lines
    if (line.empty()) {
      continue;
    }

    // Parse CSV: tag,x,y,yaw
    std::istringstream ss(line);
    std::string tag, xStr, yStr, yawStr;

    if (!std::getline(ss, tag, ',')) {
      continue;
    }
    if (!std::getline(ss, xStr, ',')) {
      continue;
    }
    if (!std::getline(ss, yStr, ',')) {
      continue;
    }
    if (!std::getline(ss, yawStr, ',')) {
      continue;
    }

    // Trim whitespace
    tag.erase(std::remove_if(tag.begin(), tag.end(), ::isspace), tag.end());

    // Parse coordinates
    try {
      double x = std::stod(xStr);
      double y = std::stod(yStr);
      double yaw = std::stod(yawStr);

      if (tag == "car_start") {
        // Store starting pose
        outTrackData.startPose = common::SE2(x, y, yaw);
      } else if (tag == "midpoint") {
        // Store midpoints for midline visualization
        outTrackData.midpoints.emplace_back(x, y, yaw);
      } else {
        // It's a cone
        ConeType type = parseConeType(tag);
        outTrackData.cones.emplace_back(x, y, type);
      }
    } catch (...) {
      // Skip invalid lines
      continue;
    }
  }

  return true;
}

} // namespace scene

