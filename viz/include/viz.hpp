#pragma once

#define GLFW_INCLUDE_NONE

// Platform-specific native window access
#if defined(__linux__)
#define GLFW_EXPOSE_NATIVE_X11
#elif defined(__APPLE__)
#define GLFW_EXPOSE_NATIVE_COCOA
#elif defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#endif

#include <GLFW/glfw3.h>
#include <webgpu/webgpu.h>

#include "CarDefaults.hpp"
#include "MarkerSystem.hpp"
#include "panels/SidePanel.hpp"
#include "panels/ViewportPanel.hpp"
#include "scene.hpp"
#include "simulator.hpp"

namespace comm {
  class MarkerSubscriber;
}

namespace viz {

/**
 * @brief Main visualization application class that manages the rendering window
 * and WebGPU context.
 *
 * This class encapsulates all visualization-related functionality including:
 * - GLFW window management
 * - WebGPU device initialization and configuration
 * - ImGui rendering setup
 * - Main rendering loop execution
 *
 * The application supports Linux (X11), macOS (Metal), and Windows platforms.
 */
class Application {
public:
  Application(scene::SceneDB& db, sim::Simulator& sim);
  ~Application(); // Defined in .cpp to handle unique_ptr with forward-declared type

  bool initialize();
  void terminate();
  void mainLoop();
  bool isRunning();
  void onResize(int newWidth, int newHeight);

  // Public for callback access
  ViewportPanel m_viewportPanel;
  SidePanel m_rightPanel;
  SidePanel m_leftPanel;
  MarkerSystem m_markerSystem;
  
  // Simulated object visibility
  bool m_showCar{true};
  bool m_showCones{true};

  // Communication
  std::unique_ptr<comm::MarkerSubscriber> m_markerSub;

private:
  // References to scene and simulator
  scene::SceneDB& m_sceneDB;
  sim::Simulator& m_simulator;

  GLFWwindow* m_window = nullptr;
  int m_width = 800;
  int m_height = 600;

  WGPUInstance m_instance = nullptr;
  WGPUSurface m_surface = nullptr;
  WGPUAdapter m_adapter = nullptr;
  WGPUDevice m_device = nullptr;
  WGPUQueue m_queue = nullptr;
  WGPUTextureFormat m_surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  float m_clearColor[4] = {0.45f, 0.55f, 0.60f, 1.00f};

  // Input state for car control
  bool m_keyW = false, m_keyA = false, m_keyS = false, m_keyD = false;

  // UI state for simulation parameters
  float m_uiWheelbase = static_cast<float>(common::CarDefaults::wheelbase);
  float m_uiVMax = static_cast<float>(common::CarDefaults::v_max);
  float m_uiDeltaMax = static_cast<float>(common::CarDefaults::delta_max);
  float m_uiDt = static_cast<float>(common::CarDefaults::dt);
  int m_stepN = 10;

  // Track loading state
  char m_trackDirBuffer[512] = "";
  std::vector<std::string> m_availableTracks;
  int m_selectedTrackIndex = -1;

  bool initGLFW();
  bool initWebGPU();
  bool initImGui();
  void terminateImGui();
  void terminateWebGPU();
  void terminateGLFW();
  void handleInput();
  void render2D();
  void setupPanels();
  void scanTrackDirectory();
};

} // namespace viz