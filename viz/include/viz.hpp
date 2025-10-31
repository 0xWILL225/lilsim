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

#include "scene.hpp"
#include "simulator.hpp"

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
  Application(scene::SceneDB& db, sim::Simulator& sim)
    : m_sceneDB(db)
    , m_simulator(sim) {
  }

  bool initialize();
  void terminate();
  void mainLoop();
  bool isRunning();
  void onResize(int newWidth, int newHeight);

  // Camera state (public for callback/input access)
  enum class CameraMode { Free, CarFollow };
  CameraMode cameraMode = CameraMode::CarFollow;

  // Car-follow camera state
  float followCarZoom = 50.0f; // pixels per meter

  // Free camera state
  float freeCameraX = 0.0f;
  float freeCameraY = 0.0f;
  float freeCameraZoom = 50.0f;

  // Panel state
  bool panelCollapsed = false;
  float panelWidth = 300.0f;
  bool panelResizing = false;

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

  // Input state
  bool m_keyW = false, m_keyA = false, m_keyS = false, m_keyD = false;
  bool m_mouseLeftPressed = false;
  float m_lastMouseX = 0.0f, m_lastMouseY = 0.0f;

  bool initGLFW();
  bool initWebGPU();
  bool initImGui();
  void terminateImGui();
  void terminateWebGPU();
  void terminateGLFW();
  void handleInput();
  void render2D();
};

} // namespace viz