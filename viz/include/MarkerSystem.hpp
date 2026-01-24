#pragma once

#include "SE2.hpp"
#include "Position.hpp"
//#include "comm/messages.pb.h"
#include <map>
#include <string>
#include <vector>
#include <optional>
#include <cstdint>

namespace viz {

namespace lilsim {
  enum FrameId { WORLD = 0, CAR = 1 };
  enum MarkerType {
      CIRCLE=0,
      RECTANGLE=1,
      TEXT=2,
      ARROW=3,
      LINE_STRIP=4,
      CIRCLE_LIST=5,
      TRIANGLE_LIST=6,
      MESH_2D=7,
      CAR_SPRITE=8
  };
}

// Use local enums instead of protobuf
using FrameId = lilsim::FrameId;
using MarkerType = lilsim::MarkerType;

/**
 * @brief Color representation with RGBA values (0-255).
 */
struct Color {
  uint8_t r{255};
  uint8_t g{255};
  uint8_t b{255};
  uint8_t a{255};

  Color() = default;
  Color(uint8_t r_, uint8_t g_, uint8_t b_, uint8_t a_ = 255)
    : r(r_)
    , g(g_)
    , b(b_)
    , a(a_) {
  }
};

/**
 * @brief Scale in X and Y dimensions.
 */
struct Scale2D {
  float x{1.0f};
  float y{1.0f};

  Scale2D() = default;
  Scale2D(float x_, float y_)
    : x(x_)
    , y(y_) {
  }
  explicit Scale2D(float uniform)
    : x(uniform)
    , y(uniform) {
  }
};

struct CarMarkerData {
  double wheelbase{1.0};
  double track_width{1.0};
  bool has_wheel_fl_angle{false};
  double wheel_fl_angle{0.0};
  bool has_wheel_fr_angle{false};
  double wheel_fr_angle{0.0};
  double opacity{1.0};
  double tint_opacity{0.0};
};

/**
 * @brief A single marker for visualization.
 */
struct Marker {
  MarkerType type{lilsim::CIRCLE};
  common::SE2 pose{0.0, 0.0, 0.0};
  Color color;
  Scale2D scale{1.0f, 1.0f};
  std::optional<double> ttl_sec;  // Time to live in simulation seconds
  FrameId frame_id{lilsim::WORLD};  // Reference frame
  
  // Type-specific data
  std::string text;                      // For Text markers
  std::vector<common::Position> points;  // For line/point/triangle-based markers
  std::vector<Color> colors;             // Per-vertex colors (for LineStrip, TriangleList, CircleList)
  std::optional<CarMarkerData> car;      // For CAR_SPRITE
  
  // Internal state
  double creation_time{0.0};  // Simulation time when created
  bool visible{true};         // Per-marker visibility
};

/**
 * @brief Key for uniquely identifying a marker.
 */
struct MarkerKey {
  std::string ns;
  int id;

  bool operator<(const MarkerKey& other) const {
    if (ns != other.ns)
      return ns < other.ns;
    return id < other.id;
  }
};

/**
 * @brief System for managing and rendering markers (RViz-like API).
 * 
 * Markers are identified by (namespace, id) pairs and can have optional TTL.
 * Supports various 2D primitive types for visualization.
 */
class MarkerSystem {
public:
  MarkerSystem() = default;

  /**
   * @brief Add or update a marker.
   * 
   * If a marker with the same (namespace, id) exists, it will be replaced.
   * 
   * @param ns Namespace for grouping markers
   * @param id Unique ID within the namespace
   * @param marker Marker data
   * @param simulation_time Current simulation time for TTL tracking
   */
  void addMarker(const std::string& ns, int id, const Marker& marker,
                 double simulation_time);

  /**
   * @brief Delete a specific marker.
   */
  void deleteMarker(const std::string& ns, int id);

  /**
   * @brief Delete all markers in a namespace.
   */
  void deleteNamespace(const std::string& ns);

  /**
   * @brief Clear all markers.
   */
  void clearAll();

  /**
   * @brief Update marker system (remove expired TTL markers).
   * 
   * @param simulation_time Current simulation time
   */
  void update(double simulation_time);

  /**
   * @brief Get all markers (for rendering).
   */
  const std::map<MarkerKey, Marker>& getMarkers() const {
    return m_markers;
  }

  /**
   * @brief Get all unique namespaces.
   */
  std::vector<std::string> getNamespaces() const;

  /**
   * @brief Get all marker IDs in a namespace.
   */
  std::vector<int> getMarkerIds(const std::string& ns) const;

  /**
   * @brief Get namespace visibility state.
   */
  bool isNamespaceVisible(const std::string& ns) const;

  /**
   * @brief Set namespace visibility (affects all markers in namespace).
   */
  void setNamespaceVisible(const std::string& ns, bool visible);

  /**
   * @brief Get per-marker visibility state.
   */
  bool isMarkerVisible(const std::string& ns, int id) const;

  /**
   * @brief Set per-marker visibility.
   */
  void setMarkerVisible(const std::string& ns, int id, bool visible);

  /**
   * @brief Replace namespace visibility table with a snapshot (used for config load).
   */
  void applyNamespaceVisibilitySnapshot(const std::map<std::string, bool>& snapshot);

  /**
   * @brief Retrieve the current namespace visibility table for persistence.
   */
  std::map<std::string, bool> snapshotNamespaceVisibility() const;

private:
  std::map<MarkerKey, Marker> m_markers;
  std::map<std::string, bool> m_namespaceVisibility;  // Default: visible
};

} // namespace viz
