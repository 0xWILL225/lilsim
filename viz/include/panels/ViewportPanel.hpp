#pragma once

#include "scene.hpp"
#include <GLFW/glfw3.h>

namespace viz {

class MarkerSystem;  // Forward declaration

/**
 * @brief Renders the 2D simulation scene with camera controls.
 *
 * Features:
 * - Grid background (1m x 1m cells)
 * - Car rendering as triangle
 * - Cone rendering with color coding
 * - Two camera modes: follow car or free-moving
 * - Zoom support
 */
class ViewportPanel {
public:
  enum class CameraMode { Free, CarFollow };

  ViewportPanel(scene::SceneDB& sceneDB, const MarkerSystem* markerSystem = nullptr,
                const bool* showCar = nullptr, const bool* showCones = nullptr)
    : m_sceneDB(sceneDB)
    , m_markerSystem(markerSystem)
    , m_showCar(showCar)
    , m_showCones(showCones) {
  }

  /**
   * @brief Draws the viewport with scene rendering.
   *
   * @param x X position of viewport in window
   * @param y Y position of viewport in window
   * @param width Width of viewport in pixels
   * @param height Height of viewport in pixels
   */
  void draw(float x, float y, float width, float height);

  /**
   * @brief Handles keyboard and mouse input for camera control.
   *
   * @param window GLFW window handle
   */
  void handleInput(GLFWwindow* window);

  // Camera state (public for external control)
  CameraMode cameraMode = CameraMode::CarFollow;
  float followCarZoom = 50.0f;
  float freeCameraX = 0.0f;
  float freeCameraY = 0.0f;
  float freeCameraZoom = 50.0f;

  /**
   * @brief Check if the mouse is hovering over this viewport.
   */
  bool isHovered() const {
    return m_isHovered;
  }

private:
  scene::SceneDB& m_sceneDB;
  const MarkerSystem* m_markerSystem;  // Read-only access to markers
  const bool* m_showCar;               // Read-only access to car visibility
  const bool* m_showCones;             // Read-only access to cones visibility
  
  // Mouse state for panning
  bool m_mouseLeftPressed = false;
  float m_lastMouseX = 0.0f;
  float m_lastMouseY = 0.0f;
  bool m_freeCameraInitialized = false;
  bool m_isHovered = false;
};

} // namespace viz



