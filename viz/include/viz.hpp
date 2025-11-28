#pragma once

#define GLFW_INCLUDE_NONE

#if defined(__linux__)
#define GLFW_EXPOSE_NATIVE_X11
#elif defined(__APPLE__)
#define GLFW_EXPOSE_NATIVE_COCOA
#elif defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#endif

#include <GLFW/glfw3.h>
#include <webgpu/webgpu.h>
#include <vector>
#include <chrono>

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

class Application {
public:
  Application(scene::SceneDB& db, sim::Simulator& sim);
  ~Application(); 

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
  
  bool m_showCar{true};
  bool m_showCones{true};

  // Communication
  // std::unique_ptr<comm::MarkerSubscriber> m_markerSub;
  std::chrono::steady_clock::time_point m_lastConnectionProbe;

private:
  scene::SceneDB& m_sceneDB;
  sim::Simulator& m_simulator;

  GLFWwindow* m_window = nullptr;
  int m_width = 1400;
  int m_height = 800;

  WGPUInstance m_instance = nullptr;
  WGPUSurface m_surface = nullptr;
  WGPUAdapter m_adapter = nullptr;
  WGPUDevice m_device = nullptr;
  WGPUQueue m_queue = nullptr;
  WGPUTextureFormat m_surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  float m_clearColor[4] = {0.45f, 0.55f, 0.60f, 1.00f};

  double m_targetFrameTime = 1.0 / 60.0;
  double m_lastFrameTime = 0.0;

  bool m_keyW = false, m_keyA = false, m_keyS = false, m_keyD = false;

  std::vector<double> m_uiParamValues;
  std::vector<int32_t> m_uiSettingValues;
  
  // Cache indices
  int m_inputIdxWheelAngle = -1;
  int m_inputIdxWheelRate = -1;
  int m_inputIdxAx = -1;
  
  int m_stateIdxX = -1;
  int m_stateIdxY = -1;
  int m_stateIdxYaw = -1;
  int m_stateIdxV = -1;
  int m_stateIdxAx = -1;
  int m_stateIdxSteerWheelAngle = -1;
  int m_stateIdxSteerWheelRate = -1;
  int m_stateIdxWheelFL = -1;
  int m_stateIdxWheelFR = -1;
  
  int m_paramIdxWheelbase = -1;
  int m_paramIdxTrackWidth = -1;
  int m_settingIdxSteeringMode = -1;
  
  int m_stepN = 10;

  char m_trackDirBuffer[512] = "";
  char m_paramFileBuffer[512] = "";
  std::vector<std::string> m_availableTracks;
  int m_selectedTrackIndex = -1;
  
  std::vector<sim::Simulator::ModelInfo> m_availableModels;
  int m_selectedModelIndex = -1;

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
  void refreshAvailableModels();
  void onModelChanged(); 
  void syncParamProfileBuffer();
};

} // namespace viz
