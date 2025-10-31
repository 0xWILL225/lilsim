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
    : sceneDB(db)
    , simulator(sim) {
  }

  bool initialize();
  void terminate();
  void mainLoop();
  bool isRunning();
  void onResize(int newWidth, int newHeight);

  // Camera state (public for callback access)
  float zoom = 50.0f; // pixels per meter

private:
  // References to scene and simulator
  scene::SceneDB& sceneDB;
  sim::Simulator& simulator;

  GLFWwindow* window = nullptr;
  int width = 800;
  int height = 600;

  WGPUInstance instance = nullptr;
  WGPUSurface surface = nullptr;
  WGPUAdapter adapter = nullptr;
  WGPUDevice device = nullptr;
  WGPUQueue queue = nullptr;
  WGPUTextureFormat surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  float clearColor[4] = {0.45f, 0.55f, 0.60f, 1.00f};

  // Input state
  bool keyW = false, keyA = false, keyS = false, keyD = false;

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