#pragma once

#include "scene.hpp"
#include "TextureManager.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <optional>

namespace viz {

class MarkerSystem;

class ViewportPanel {
public:
  enum class CameraMode { Free, CarFollow };

  struct RenderState {
      double x{0}, y{0}, yaw{0}; // Car pose
      double v{0};               // Car velocity
      double wheelbase{2.8};     // For rendering size
      double track_width{1.6};   // For rendering size
      double sim_time{0.0};
      
      // Optional HUD states
      std::optional<double> ax;
      std::optional<double> steering_wheel_angle;
      std::optional<double> steering_wheel_rate;
      std::optional<double> wheel_fl_angle;
      std::optional<double> wheel_fr_angle;
      
      const std::vector<scene::Cone>* cones{nullptr};
  };

  ViewportPanel(const MarkerSystem* markerSystem = nullptr,
                const bool* showCar = nullptr, const bool* showCones = nullptr)
    : m_markerSystem(markerSystem)
    , m_showCar(showCar)
    , m_showCones(showCones) {
  }

  void draw(float x, float y, float width, float height, const RenderState& state);

  void handleInput(GLFWwindow* window, const RenderState& state); 

  // Camera state
  CameraMode cameraMode = CameraMode::CarFollow;
  float followCarZoom = 50.0f;
  float freeCameraX = 0.0f;
  float freeCameraY = 0.0f;
  float freeCameraZoom = 50.0f;

  bool isHovered() const {
    return m_isHovered;
  }

private:
  const MarkerSystem* m_markerSystem;
  const bool* m_showCar;
  const bool* m_showCones;
  
  bool m_mouseLeftPressed = false;
  float m_lastMouseX = 0.0f;
  float m_lastMouseY = 0.0f;
  bool m_freeCameraInitialized = false;
  bool m_isHovered = false;

  struct CarVisualAssets {
      const TextureManager::TextureData* chassis{nullptr};
      const TextureManager::TextureData* overlay{nullptr};
      const TextureManager::TextureData* tire{nullptr};
      const TextureManager::TextureData* tsalRed{nullptr};
      double blueXNorm{0.5};
      double blueYNorm{0.5};
      double redXNorm{0.5};
      double redYNorm{0.0};
      double greenXNorm{0.5};
      double greenYNorm{0.0};
      double wheelbaseNorm{1.0};
      double trackWidthNorm{1.0};
      int baseWidth{1};
      int baseHeight{1};
      int tireWidth{1};
      int tireHeight{1};
  };

  CarVisualAssets m_carVisual;
  bool m_carVisualLoaded{false};

  bool ensureCarVisualAssets();
  bool loadReferencePoints(const std::string& assetPath);
};

} // namespace viz
