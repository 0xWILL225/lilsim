#include "viz.hpp"
#include "KeyBindings.hpp"
#include "TrackLoader.hpp"
#include "TextureManager.hpp"
#include "models/cars/include/base.h"
#include "imgui_toggle/imgui_toggle.h"

#include <GLFW/glfw3native.h>
#include <spdlog/spdlog.h>
#include <ImGuiFileDialog.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>
#include <cfloat>

#ifdef Success
#undef Success
#endif
#ifdef None
#undef None
#endif

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_wgpu.h>

#define WGPU_STR(s) WGPUStringView { s, WGPU_STRLEN }

namespace {
constexpr ImVec2 kFileDialogMinSize{700.0f, 420.0f};
constexpr ImVec2 kFileDialogMaxSize{FLT_MAX, FLT_MAX};
}

namespace viz {

Application::Application(scene::SceneDB& db, sim::Simulator& sim)
  : m_sceneDB(db)
  , m_simulator(sim)
  , m_rightPanel("Admin Panel", SidePanel::Side::Right, 300.0f)
  , m_leftPanel("Display", SidePanel::Side::Left, 300.0f)
  , m_viewportPanel(nullptr, &m_showCar, &m_showCones) 
  , m_lastConnectionProbe(std::chrono::steady_clock::now()) {
  
  std::filesystem::path exePath = std::filesystem::current_path();
  std::string defaultTrackPath = (exePath / "tracks").string();
  strncpy(m_trackDirBuffer, defaultTrackPath.c_str(), sizeof(m_trackDirBuffer) - 1);
  m_trackDirBuffer[sizeof(m_trackDirBuffer) - 1] = '\0';
  
  m_viewportPanel = ViewportPanel(&m_markerSystem, &m_showCar, &m_showCones);

  setupPanels();
  
  scanTrackDirectory();
  refreshAvailableModels();
  syncParamProfileBuffer();
  if (initZmqInterface()) {
    requestMetadataSnapshot();
  }
  m_markerSub = std::make_unique<comm::MarkerSubscriber>();
  if (!m_markerSub->start()) {
    spdlog::warn("[viz] Failed to start marker subscriber; remote markers disabled.");
    m_markerSub.reset();
  }
}

Application::~Application() {
  if (m_markerSub) {
    m_markerSub->stop();
  }
}

static void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (app) app->onResize(width, height);
}

static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  (void)xoffset;
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (app) {
    if (!app->m_viewportPanel.isHovered()) return;
    float zoomDelta = (yoffset > 0) ? 1.1f : 0.9f;
    if (app->m_viewportPanel.cameraMode == ViewportPanel::CameraMode::Free) {
      app->m_viewportPanel.freeCameraZoom *= zoomDelta;
      app->m_viewportPanel.freeCameraZoom = std::clamp(app->m_viewportPanel.freeCameraZoom, 5.0f, 500.0f);
    } else {
      app->m_viewportPanel.followCarZoom *= zoomDelta;
      app->m_viewportPanel.followCarZoom = std::clamp(app->m_viewportPanel.followCarZoom, 5.0f, 500.0f);
    }
  }
}

static void onAdapterRequestEnded(WGPURequestAdapterStatus status,
                                  WGPUAdapter m_adapter, WGPUStringView message,
                                  void* userdata1, void* userdata2) {
  (void)userdata2; (void)message;
  if (status == WGPURequestAdapterStatus_Success) *(WGPUAdapter*)userdata1 = m_adapter;
}

static void onDeviceRequestEnded(WGPURequestDeviceStatus status,
                                 WGPUDevice m_device, WGPUStringView message,
                                 void* userdata1, void* userdata2) {
  (void)userdata2; (void)message;
  if (status == WGPURequestDeviceStatus_Success) *(WGPUDevice*)userdata1 = m_device;
}

bool Application::initGLFW() {
  if (!glfwInit()) return false;
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  m_window = glfwCreateWindow(m_width, m_height, "lilsim", nullptr, nullptr);
  if (!m_window) return false;
  glfwSetWindowUserPointer(m_window, this);
  glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);
  glfwSetScrollCallback(m_window, scrollCallback);
  return true;
}

bool Application::initWebGPU() {
  WGPUInstanceDescriptor instanceDesc = {};
  m_instance = wgpuCreateInstance(&instanceDesc);
  if (!m_instance) return false;

#if defined(__linux__)
  WGPUSurfaceSourceXlibWindow fromXlib = {};
  fromXlib.chain.sType = WGPUSType_SurfaceSourceXlibWindow;
  fromXlib.display = glfwGetX11Display();
  fromXlib.window = glfwGetX11Window(m_window);
  WGPUSurfaceDescriptor surfaceDesc = {};
  surfaceDesc.nextInChain = reinterpret_cast<WGPUChainedStruct const*>(&fromXlib);
  m_surface = wgpuInstanceCreateSurface(m_instance, &surfaceDesc);
#elif defined(__APPLE__)
#elif defined(_WIN32)
#endif

  if (!m_surface) return false;

  WGPURequestAdapterOptions adapterOpts = {};
  adapterOpts.compatibleSurface = m_surface;
  adapterOpts.powerPreference = WGPUPowerPreference_HighPerformance;
  WGPURequestAdapterCallbackInfo cbInfo = {};
  cbInfo.mode = WGPUCallbackMode_AllowSpontaneous;
  cbInfo.callback = onAdapterRequestEnded;
  cbInfo.userdata1 = &m_adapter;
  wgpuInstanceRequestAdapter(m_instance, &adapterOpts, cbInfo);

  while (!m_adapter) {
    wgpuInstanceProcessEvents(m_instance);
    glfwPollEvents();
  }

  WGPUDeviceDescriptor deviceDesc = {};
  deviceDesc.label = WGPU_STR("Main Device");
  WGPURequestDeviceCallbackInfo devCbInfo = {};
  devCbInfo.mode = WGPUCallbackMode_AllowSpontaneous;
  devCbInfo.callback = onDeviceRequestEnded;
  devCbInfo.userdata1 = &m_device;
  wgpuAdapterRequestDevice(m_adapter, &deviceDesc, devCbInfo);

  while (!m_device) {
    wgpuInstanceProcessEvents(m_instance);
    glfwPollEvents();
  }

  m_queue = wgpuDeviceGetQueue(m_device);

  WGPUSurfaceConfiguration config = {};
  config.device = m_device;
  config.format = m_surfaceFormat;
  config.usage = WGPUTextureUsage_RenderAttachment;
  config.width = m_width;
  config.height = m_height;
  config.presentMode = WGPUPresentMode_Immediate;
  config.alphaMode = WGPUCompositeAlphaMode_Auto;
  wgpuSurfaceConfigure(m_surface, &config);

  TextureManager::getInstance().initialize(m_device, m_queue);
  return true;
}

bool Application::initImGui() {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOther(m_window, true);
  ImGui_ImplWGPU_InitInfo init_info = {};
  init_info.Device = m_device;
  init_info.NumFramesInFlight = 3;
  init_info.RenderTargetFormat = m_surfaceFormat;
  init_info.DepthStencilFormat = WGPUTextureFormat_Undefined;
  ImGui_ImplWGPU_Init(&init_info);
  return true;
}

bool Application::initialize() {
  if (!initGLFW()) return false;
  if (!initWebGPU()) return false;
  if (!initImGui()) return false;
  return true;
}

void Application::terminate() {
  TextureManager::getInstance().cleanup();
  terminateImGui();
  terminateWebGPU();
  terminateGLFW();
}

void Application::terminateImGui() {
  ImGui_ImplWGPU_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

void Application::terminateWebGPU() {
  if (m_queue) wgpuQueueRelease(m_queue);
  if (m_device) wgpuDeviceRelease(m_device);
  if (m_adapter) wgpuAdapterRelease(m_adapter);
  if (m_surface) wgpuSurfaceRelease(m_surface);
  if (m_instance) wgpuInstanceRelease(m_instance);
}

void Application::terminateGLFW() {
  if (m_window) glfwDestroyWindow(m_window);
  glfwTerminate();
}

bool Application::isRunning() {
  return m_window && !glfwWindowShouldClose(m_window);
}

void Application::onResize(int newWidth, int newHeight) {
  if (newWidth <= 0 || newHeight <= 0) return;
  m_width = newWidth;
  m_height = newHeight;
  if (m_surface && m_device) {
    WGPUSurfaceConfiguration config = {};
    config.device = m_device;
    config.format = m_surfaceFormat;
    config.usage = WGPUTextureUsage_RenderAttachment;
    config.width = m_width;
    config.height = m_height;
    config.presentMode = WGPUPresentMode_Immediate;
    config.alphaMode = WGPUCompositeAlphaMode_Auto;
    wgpuSurfaceConfigure(m_surface, &config);
  }
}

void Application::refreshAvailableModels() {
    m_availableModels = m_simulator.getAvailableModels();
    
    // Sync selected index with current loaded model
    std::string current = m_simulator.getCurrentModelName();
    m_selectedModelIndex = -1;
    for(size_t i=0; i<m_availableModels.size(); ++i) {
        if(m_availableModels[i].name == current) {
            m_selectedModelIndex = (int)i;
            break;
        }
    }
}

void Application::onModelChanged() {
    refreshAvailableModels(); // Sync dropdown
    const auto* desc = m_simulator.getCurrentModelDescriptor();
    if (!desc) return;
    
    m_uiParamValues.assign(desc->param_values, desc->param_values + desc->num_params);
    m_uiSettingValues.assign(desc->setting_values, desc->setting_values + desc->num_settings);
    
    // Cache inputs
    m_inputIdxWheelAngle = -1;
    m_inputIdxWheelRate = -1;
    m_inputIdxAx = -1;
    for(size_t i=0; i<desc->num_inputs; ++i) {
        std::string name = desc->input_names[i];
        if (name == "steering_wheel_angle_input") m_inputIdxWheelAngle = (int)i;
        else if (name == "steering_wheel_rate_input") m_inputIdxWheelRate = (int)i;
        else if (name == "ax" || name == "ax_input") m_inputIdxAx = (int)i;
    }
    
    // Cache states
    m_stateIdxX = -1; m_stateIdxY = -1; m_stateIdxYaw = -1; m_stateIdxV = -1;
    m_stateIdxAx = -1; m_stateIdxSteerWheelAngle = -1; m_stateIdxSteerWheelRate = -1;
    m_stateIdxWheelFL = -1; m_stateIdxWheelFR = -1;
    
    for(size_t i=0; i<desc->num_states; ++i) {
        std::string name = desc->state_names[i];
        if (name == "x") m_stateIdxX = (int)i;
        else if (name == "y") m_stateIdxY = (int)i;
        else if (name == "yaw") m_stateIdxYaw = (int)i;
        else if (name == "v") m_stateIdxV = (int)i;
        else if (name == "ax") m_stateIdxAx = (int)i;
        else if (name == "steering_wheel_angle") m_stateIdxSteerWheelAngle = (int)i;
        else if (name == "steering_wheel_rate") m_stateIdxSteerWheelRate = (int)i;
        else if (name == "wheel_fl_angle") m_stateIdxWheelFL = (int)i;
        else if (name == "wheel_fr_angle") m_stateIdxWheelFR = (int)i;
    }
    
    // Cache params
    m_paramIdxWheelbase = -1;
    m_paramIdxTrackWidth = -1;
    for(size_t i=0; i<desc->num_params; ++i) {
        std::string name = desc->param_names[i];
        if (name == "wheelbase") m_paramIdxWheelbase = (int)i;
        else if (name == "track_width") m_paramIdxTrackWidth = (int)i;
    }

    // Cache steering mode setting index
    m_settingIdxSteeringMode = -1;
    for(size_t i=0; i<desc->num_settings; ++i) {
        std::string name = desc->setting_names[i];
        if (name == "steering_input_mode") {
            m_settingIdxSteeringMode = (int)i;
            break;
        }
    }

  syncParamProfileBuffer();
}

void Application::handleInput() {
  pollMetadataUpdates();
  pollMarkerMessages();
  m_guiControlSource = !m_simulator.isExternalControlEnabled();
  if (m_simulator.checkAndClearModelChanged()) {
      onModelChanged();
  }

  if (m_selectedModelIndex == -1 && !m_availableModels.empty()) {
      onModelChanged();
  }

  const scene::Scene& scene = m_sceneDB.snapshot();
  double simTime = m_sceneDB.tick.load(std::memory_order_relaxed) * m_simulator.getDt();
  ViewportPanel::RenderState renderState;
  renderState.sim_time = simTime;
  if (m_stateIdxX >= 0 && (size_t)m_stateIdxX < scene.car_state_values.size()) renderState.x = scene.car_state_values[m_stateIdxX];
  if (m_stateIdxY >= 0 && (size_t)m_stateIdxY < scene.car_state_values.size()) renderState.y = scene.car_state_values[m_stateIdxY];
  if (m_stateIdxYaw >= 0 && (size_t)m_stateIdxYaw < scene.car_state_values.size()) renderState.yaw = scene.car_state_values[m_stateIdxYaw];
  
  m_viewportPanel.handleInput(m_window, renderState);

  static bool pauseKeyWasPressed = false;
  bool pauseKeyPressed = glfwGetKey(m_window, gKeyBindings.pauseSimulation) == GLFW_PRESS;
  if (pauseKeyPressed && !pauseKeyWasPressed) {
    if (m_simulator.isPaused()) m_simulator.resume();
    else m_simulator.pause();
  }
  pauseKeyWasPressed = pauseKeyPressed;

  static bool resetKeyWasPressed = false;
  bool resetKeyPressed = glfwGetKey(m_window, gKeyBindings.resetSimulation) == GLFW_PRESS;
  if (resetKeyPressed && !resetKeyWasPressed) {
    m_simulator.reset();
  }
  resetKeyWasPressed = resetKeyPressed;

  if (m_guiControlSource) {
    const auto* desc = m_simulator.getCurrentModelDescriptor();
        if (desc) {
        std::vector<double> input(desc->num_inputs, 0.0);
        
        bool keyW = glfwGetKey(m_window, gKeyBindings.carAccelerate) == GLFW_PRESS;
        bool keyS = glfwGetKey(m_window, gKeyBindings.carBrake) == GLFW_PRESS;
        bool keyA = glfwGetKey(m_window, gKeyBindings.carSteerLeft) == GLFW_PRESS;
        bool keyD = glfwGetKey(m_window, gKeyBindings.carSteerRight) == GLFW_PRESS;
        
        bool steeringRateMode = false;
        if (m_settingIdxSteeringMode >= 0 && desc->setting_values) {
            steeringRateMode = desc->setting_values[m_settingIdxSteeringMode] == 1;
        }
        
        if (m_inputIdxAx >= 0) {
            double maxAx = desc->input_max[m_inputIdxAx];
            double minAx = desc->input_min[m_inputIdxAx];
            if (keyW) input[m_inputIdxAx] = maxAx;
            else if (keyS) input[m_inputIdxAx] = minAx;
        }
        
        if (steeringRateMode) {
            if (m_inputIdxWheelRate >= 0) {
                double maxRate = desc->input_max[m_inputIdxWheelRate];
                double minRate = desc->input_min[m_inputIdxWheelRate];
                if (keyA) input[m_inputIdxWheelRate] = maxRate;
                else if (keyD) input[m_inputIdxWheelRate] = minRate;
            }
        } else {
            if (m_inputIdxWheelAngle >= 0) {
                double maxAngle = desc->input_max[m_inputIdxWheelAngle];
                double minAngle = desc->input_min[m_inputIdxWheelAngle];
                if (keyA) input[m_inputIdxWheelAngle] = maxAngle;
                else if (keyD) input[m_inputIdxWheelAngle] = minAngle;
            }
        }
        
        m_simulator.setInput(input);
    }
  }
}

void Application::setupPanels() {
  m_leftPanel.addSection("Simulated Objects", [this]() {
    ImGui::Checkbox("Car", &m_showCar);
    ImGui::Checkbox("Cones", &m_showCones);
  });
  
  m_leftPanel.addSection("Markers", [this]() { });
  
  m_rightPanel.addSection("Simulation Control", [this]() {
    bool isPaused = m_simulator.isPaused();
    
    if (isPaused) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.6f, 0.0f, 1.0f));
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.0f, 0.0f, 1.0f));
    }
    
    if (ImGui::Button(isPaused ? "Resume" : "Pause", ImVec2(-1, 30))) {
      sendAdminCommand(isPaused ? ::lilsim::AdminCommandType::RUN
                                : ::lilsim::AdminCommandType::PAUSE);
    }
    
    ImGui::PopStyleColor();
    
    ImGui::Text("Step size (N):");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    ImGui::InputInt("##StepN", &m_stepN, 0, 0); 
    m_stepN = std::max(1, m_stepN);

    float buttonWidth = ImGui::GetContentRegionAvail().x / 2.0f - 4.0f;
    if (ImGui::Button("Step N", ImVec2(buttonWidth, 30))) {
      m_simulator.step(static_cast<uint64_t>(std::max(1, m_stepN)));
    }
    ImGui::SameLine();
    if (ImGui::Button("Step 1", ImVec2(-1, 30))) {
      m_simulator.step(1);
    }

    if (ImGui::Button("Reset", ImVec2(-1, 30))) {
      sendAdminCommand(::lilsim::AdminCommandType::RESET);
    }
    
    ImGui::Separator();
    double activeDt = m_simulator.getDt();
    double requestedDt = m_simulator.getRequestedDt();
    double dtInput = requestedDt;
    ImGui::TextUnformatted("Timestep dt (s)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputDouble("##SimDtInput", &dtInput, 0.001, 0.01, "%.4f")) {
        dtInput = std::clamp(dtInput, 0.001, 1.0);
        m_simulator.requestDt(dtInput);
    }
    ImGui::SameLine();
    if (std::abs(dtInput - activeDt) > 1e-6) {
        ImGui::TextUnformatted("(pending, applies on reset)");
    } else {
        ImGui::TextUnformatted("(active)");
    }

    double runSpeed = m_simulator.getRunSpeed();
    ImGui::TextUnformatted("Run Speed (0.1x - 3x)");
    ImGui::SetNextItemWidth(-1);
    float minSpeed = 0.1f;
    float maxSpeed = 3.0f;
    float runSpeedFloat = static_cast<float>(runSpeed);
    if (ImGui::SliderFloat("##SimRunSpeed", &runSpeedFloat, minSpeed, maxSpeed, "%.1fx", ImGuiSliderFlags_AlwaysClamp)) {
        runSpeedFloat = std::round(runSpeedFloat * 10.0f) / 10.0f;
        m_simulator.setRunSpeed(static_cast<double>(runSpeedFloat));
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Input Source");
    ImGuiToggleConfig toggleCfg;
    toggleCfg.Flags = ImGuiToggleFlags_Animated | ImGuiToggleFlags_Bordered | ImGuiToggleFlags_Shadowed;
    toggleCfg.Width = 44.0f;
    toggleCfg.Height = 20.0f;

    bool guiToggle = m_guiControlSource;
    ImGui::PushID("InputSourceToggle");
    if (ImGui::Toggle("##InputSource", &guiToggle, toggleCfg)) {
        bool targetGui = guiToggle;
        bool targetSync = targetGui ? false : m_simulator.isSyncMode();
        bool external = !targetGui;
        if (sendSetModeCommand(targetSync, external)) {
            m_guiControlSource = targetGui;
        } else {
            guiToggle = !targetGui;
        }
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(guiToggle ? "GUI" : "ZeroMQ Client");
    ImGui::PopID();

    ImGui::TextUnformatted("Control Mode");
    bool controlDisabled = guiToggle;
    bool asyncState = controlDisabled ? true : !m_simulator.isSyncMode();
    bool asyncToggle = asyncState;

    ImGui::PushID("ControlModeToggle");
    if (controlDisabled) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Toggle("##ControlMode", &asyncToggle, toggleCfg) && !controlDisabled) {
        if (!sendSetModeCommand(!asyncToggle, !m_guiControlSource)) {
            asyncToggle = asyncState;
        }
    }
    if (controlDisabled) {
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(asyncToggle ? "Asynchronous" : "Synchronous");

    ImGui::PopID();
  });

  m_rightPanel.addSection("Model Selection", [this]() {
      std::string currentName = "None";
      if (m_selectedModelIndex >= 0 && m_selectedModelIndex < (int)m_availableModels.size()) {
          currentName = m_availableModels[m_selectedModelIndex].name;
      } else {
          // If no selection but sim has model, show it
          std::string simModel = m_simulator.getCurrentModelName();
          if (!simModel.empty()) currentName = simModel;
      }

      if (ImGui::BeginCombo("Model", currentName.c_str())) {
          for (int i = 0; i < (int)m_availableModels.size(); ++i) {
              bool isSelected = (m_selectedModelIndex == i);
              if (ImGui::Selectable(m_availableModels[i].name.c_str(), isSelected)) {
                  m_selectedModelIndex = i;
                  if (m_simulator.loadModel(m_availableModels[i].path)) {
                      onModelChanged();
                  }
              }
              if (isSelected) ImGui::SetItemDefaultFocus();
          }
          ImGui::EndCombo();
      }
      
      if (ImGui::Button("Refresh Models")) {
          refreshAvailableModels();
      }

      ImGui::Separator();
      ImGui::Text("Parameter Profile");
      const ImGuiStyle& style = ImGui::GetStyle();
      const float browseWidth = ImGui::CalcTextSize("Browse").x + style.FramePadding.x * 2.0f + 10.0f;
      float firstRowWidth = ImGui::GetContentRegionAvail().x;
      float inputWidth = std::max(50.0f, firstRowWidth - browseWidth - style.ItemSpacing.x);
      ImGui::SetNextItemWidth(inputWidth);
      ImGui::InputText("##paramProfilePath", m_paramFileBuffer, sizeof(m_paramFileBuffer));
      ImGui::SameLine();
      if (ImGui::Button("Browse##paramProfile", ImVec2(browseWidth, 0.0f))) {
          IGFD::FileDialogConfig config;
          std::filesystem::path initial = m_paramFileBuffer[0] ? std::filesystem::path(m_paramFileBuffer).parent_path()
                                                               : std::filesystem::current_path();
          config.path = initial.empty() ? "." : initial.string();
          config.flags = ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_DontShowHiddenFiles;
          ImGuiFileDialog::Instance()->OpenDialog("ParamProfileDialog", "Select Parameter File", ".yaml,.yml", config);
      }

      ImGui::Spacing();
      float secondRowWidth = ImGui::GetContentRegionAvail().x;
      float halfWidth = (secondRowWidth - style.ItemSpacing.x) * 0.5f;
      if (ImGui::Button("Load Profile", ImVec2(halfWidth, 0.0f))) {
          if (sendProfileCommand(::lilsim::AdminCommandType::LOAD_PARAM_PROFILE, std::string(m_paramFileBuffer))) {
              syncParamProfileBuffer();
          }
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear##paramProfile", ImVec2(halfWidth, 0.0f))) {
          if (sendProfileCommand(::lilsim::AdminCommandType::CLEAR_PARAM_PROFILE)) {
              syncParamProfileBuffer();
          }
      }
      ImGui::TextUnformatted("Profile applies on Reset.");

      if (ImGuiFileDialog::Instance()->Display("ParamProfileDialog", ImGuiWindowFlags_None, kFileDialogMinSize, kFileDialogMaxSize)) {
          if (ImGuiFileDialog::Instance()->IsOk()) {
              std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
              if (!filePathName.empty()) {
                  std::strncpy(m_paramFileBuffer, filePathName.c_str(), sizeof(m_paramFileBuffer) - 1);
                  m_paramFileBuffer[sizeof(m_paramFileBuffer) - 1] = '\0';
                  if (sendProfileCommand(::lilsim::AdminCommandType::LOAD_PARAM_PROFILE, filePathName)) {
                      syncParamProfileBuffer();
                  }
              }
          }
          ImGuiFileDialog::Instance()->Close();
      }
  });

  m_rightPanel.addSection("Parameters", [this]() {
      const auto* desc = m_simulator.getCurrentModelDescriptor();
      if (!desc) return;
      
      if (m_simulator.checkAndClearModelChanged()) {
          onModelChanged();
      }

      std::vector<double> pendingSnapshot;
      if (m_simulator.consumePendingParamSnapshot(pendingSnapshot)) {
          m_uiParamValues = pendingSnapshot;
      }
      
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 4));
      
      for (size_t i=0; i<desc->num_params; ++i) {
          if (m_uiParamValues.size() <= i) break; 
          double v = m_uiParamValues[i];
          double previous = v;
          double min = desc->param_min[i];
          double max = desc->param_max[i];
          const char* name = desc->param_names[i];
          
          ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
          ImGui::BeginChild((std::string("##param_") + name).c_str(), ImVec2(0, 50), true);
          ImGui::Text("%s", name);
          ImGui::SetNextItemWidth(-1);
          
          if (ImGui::InputDouble((std::string("##input_") + name).c_str(), &v, 0.0, 0.0, "%.3f")) {
              v = std::clamp(v, min, max);
              if (stageParamUpdate(i, v)) {
                  m_uiParamValues[i] = v;
              } else {
                  m_uiParamValues[i] = previous;
              }
          }
          ImGui::EndChild();
          ImGui::PopStyleColor();
      }
      
      ImGui::PopStyleVar();
  });
  
  m_rightPanel.addSection("Settings", [this]() {
      const auto* desc = m_simulator.getCurrentModelDescriptor();
      if (!desc) return;
      
      std::vector<int32_t> pendingSettings;
      if (m_simulator.consumePendingSettingSnapshot(pendingSettings)) {
          m_uiSettingValues = pendingSettings;
      }

      for (size_t i=0; i<desc->num_settings; ++i) {
          if (m_uiSettingValues.size() <= i) break;
          int v = m_uiSettingValues[i];
          int prev = v;
          const char* name = desc->setting_names[i];
          
          std::vector<const char*> options;
          for(size_t k=0; k<desc->num_setting_options; ++k) {
              if (desc->setting_option_setting_index[k] == (int32_t)i) {
                  options.push_back(desc->setting_option_names[k]);
              }
          }
          
          if (!options.empty()) {
              ImGui::SetNextItemWidth(-1);
              if (ImGui::Combo(name, &v, options.data(), (int)options.size())) {
                  if (stageSettingUpdate(i, v)) {
                      m_uiSettingValues[i] = v;
                  } else {
                      m_uiSettingValues[i] = prev;
                  }
              }
          }
      }
  });

  m_rightPanel.addSection("Track Loading", [this]() {
      ImGui::Text("Track Directory:");
      ImGui::SetNextItemWidth(-80);
      ImGui::InputText("##trackdir", m_trackDirBuffer, sizeof(m_trackDirBuffer));
      ImGui::SameLine();
      if (ImGui::Button("Browse...")) {
          IGFD::FileDialogConfig config;
          config.path = m_trackDirBuffer[0] ? m_trackDirBuffer : ".";
          config.flags = ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_DisableCreateDirectoryButton |
                         ImGuiFileDialogFlags_OptionalFileName;
          ImGuiFileDialog::Instance()->OpenDialog("TrackDirDialog", "Select Track Directory", nullptr, config);
      }

      if (ImGuiFileDialog::Instance()->Display("TrackDirDialog", ImGuiWindowFlags_None, kFileDialogMinSize, kFileDialogMaxSize)) {
          if (ImGuiFileDialog::Instance()->IsOk()) {
              std::string selectedPath = ImGuiFileDialog::Instance()->GetCurrentPath();
              if (!selectedPath.empty()) {
                  std::strncpy(m_trackDirBuffer, selectedPath.c_str(), sizeof(m_trackDirBuffer) - 1);
                  m_trackDirBuffer[sizeof(m_trackDirBuffer) - 1] = '\0';
                  scanTrackDirectory();
              }
          }
          ImGuiFileDialog::Instance()->Close();
      }
      
      ImGui::Text("Tracks:");
      ImGui::BeginChild("##tracklist", ImVec2(0, 150), true);
      for(size_t i=0; i<m_availableTracks.size(); ++i) {
          bool isSelected = (m_selectedTrackIndex == (int)i);
          if(ImGui::Selectable(m_availableTracks[i].c_str(), isSelected, ImGuiSelectableFlags_AllowDoubleClick)) {
              m_selectedTrackIndex = (int)i;
              if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                  std::string filepath = std::string(m_trackDirBuffer) + "/" + m_availableTracks[i] + ".csv";
                  sendSetTrackCommand(filepath);
              }
          }
      }
      ImGui::EndChild();
      
      if (ImGui::Button("Load Selected Track", ImVec2(-1, 30))) {
          if (m_selectedTrackIndex >= 0 && m_selectedTrackIndex < (int)m_availableTracks.size()) {
              std::string filepath = std::string(m_trackDirBuffer) + "/" + m_availableTracks[m_selectedTrackIndex] + ".csv";
              sendSetTrackCommand(filepath);
          }
      }
  });
  
  m_rightPanel.addSection("Status", [this]() {
      const scene::Scene& scene = m_sceneDB.snapshot();
      uint64_t tick = m_sceneDB.tick.load();
      double simTime = tick * m_simulator.getDt();
      ImGui::Text("Tick: %lu", (unsigned long)tick);
      ImGui::Text("Sim Time: %.3f s", simTime);
      
      if (m_stateIdxX >= 0 && m_stateIdxY >= 0 && (size_t)m_stateIdxY < scene.car_state_values.size()) {
          double x = scene.car_state_values[m_stateIdxX];
          double y = scene.car_state_values[m_stateIdxY];
          ImGui::Text("Pos: (%.2f, %.2f)", x, y);
      }
      if (m_stateIdxV >= 0 && (size_t)m_stateIdxV < scene.car_state_values.size()) {
          ImGui::Text("V: %.2f m/s", scene.car_state_values[m_stateIdxV]);
      }
  });
}

void Application::render2D() {
  const scene::Scene& scene = m_sceneDB.snapshot();
  uint64_t tick = m_sceneDB.tick.load(std::memory_order_relaxed);
  double simTime = tick * m_simulator.getDt();
  m_markerSystem.update(simTime);
  
  m_leftPanel.draw(m_width, m_height);
  m_rightPanel.draw(m_width, m_height);
  
  ViewportPanel::RenderState rs;
  rs.sim_time = simTime;
  const auto* desc = m_simulator.getCurrentModelDescriptor();
  
  if (desc && !scene.car_state_values.empty()) {
      if (m_stateIdxX >= 0 && (size_t)m_stateIdxX < scene.car_state_values.size()) rs.x = scene.car_state_values[m_stateIdxX];
      if (m_stateIdxY >= 0 && (size_t)m_stateIdxY < scene.car_state_values.size()) rs.y = scene.car_state_values[m_stateIdxY];
      if (m_stateIdxYaw >= 0 && (size_t)m_stateIdxYaw < scene.car_state_values.size()) rs.yaw = scene.car_state_values[m_stateIdxYaw];
      
      if (m_paramIdxWheelbase >= 0 &&
          desc->param_values &&
          (size_t)m_paramIdxWheelbase < desc->num_params) {
          rs.wheelbase = desc->param_values[m_paramIdxWheelbase];
      }
      if (m_paramIdxTrackWidth >= 0 &&
          desc->param_values &&
          (size_t)m_paramIdxTrackWidth < desc->num_params) {
          rs.track_width = desc->param_values[m_paramIdxTrackWidth];
      }
      
      if (m_stateIdxAx >= 0 && (size_t)m_stateIdxAx < scene.car_state_values.size()) {
          rs.ax = scene.car_state_values[m_stateIdxAx];
      }
      
      if (m_stateIdxSteerWheelAngle >= 0 && (size_t)m_stateIdxSteerWheelAngle < scene.car_state_values.size()) {
          rs.steering_wheel_angle = scene.car_state_values[m_stateIdxSteerWheelAngle];
      }
      
      if (m_stateIdxSteerWheelRate >= 0 && (size_t)m_stateIdxSteerWheelRate < scene.car_state_values.size()) {
          rs.steering_wheel_rate = scene.car_state_values[m_stateIdxSteerWheelRate];
      }

      if (m_stateIdxWheelFL >= 0 && (size_t)m_stateIdxWheelFL < scene.car_state_values.size()) {
          rs.wheel_fl_angle = scene.car_state_values[m_stateIdxWheelFL];
      }

      if (m_stateIdxWheelFR >= 0 && (size_t)m_stateIdxWheelFR < scene.car_state_values.size()) {
          rs.wheel_fr_angle = scene.car_state_values[m_stateIdxWheelFR];
      }
  }
  
  rs.cones = &scene.cones;
  
  float leftW = m_leftPanel.getWidth();
  float rightW = m_rightPanel.getWidth();
m_viewportPanel.draw(leftW, 0, m_width - leftW - rightW, m_height, rs);
}

/** @brief Initialize GUI ZMQ sockets and connect to simulator inproc endpoints. */
bool Application::initZmqInterface() {
  m_simulator.setCommEnable(true);
  m_zmqContext = m_simulator.getCommContext();
  if (!m_zmqContext) {
    spdlog::error("[viz] Failed to acquire ZMQ context from simulator");
    return false;
  }
  try {
    m_adminSocket = std::make_unique<zmq::socket_t>(*m_zmqContext, zmq::socket_type::req);
    m_adminSocket->connect(comm::endpoints::ADMIN_REP_INPROC);

    m_controlPub = std::make_unique<zmq::socket_t>(*m_zmqContext, zmq::socket_type::pub);
    m_controlPub->connect(comm::endpoints::CONTROL_ASYNC_SUB_INPROC);

    m_metadataSub = std::make_unique<zmq::socket_t>(*m_zmqContext, zmq::socket_type::sub);
    m_metadataSub->connect(comm::endpoints::METADATA_PUB_INPROC);
    m_metadataSub->set(zmq::sockopt::subscribe, "");
  } catch (const zmq::error_t& err) {
    spdlog::error("[viz] Failed to initialize ZMQ sockets: {}", err.what());
    m_adminSocket.reset();
    m_controlPub.reset();
    m_metadataSub.reset();
    return false;
  }
  return true;
}

/** @brief Send an AdminCommand via the inproc REQ/REP pair. */
bool Application::sendAdminCommand(::lilsim::AdminCommandType type,
                                   const std::function<void(::lilsim::AdminCommand&)>& builder,
                                   ::lilsim::AdminReply* out) {
  if (!m_adminSocket) {
    return false;
  }
  ::lilsim::AdminCommand cmd;
  auto* header = cmd.mutable_header();
  uint64_t tick = m_sceneDB.tick.load(std::memory_order_relaxed);
  header->set_tick(tick);
  header->set_sim_time(tick * m_simulator.getDt());
  header->set_version(1);
  cmd.set_type(type);
  if (builder) builder(cmd);

  std::string payload = cmd.SerializeAsString();
  zmq::message_t request(payload.size());
  std::memcpy(request.data(), payload.data(), payload.size());

  try {
    if (!m_adminSocket->send(request, zmq::send_flags::none)) {
      spdlog::warn("[viz] Failed to send admin command {}", static_cast<int>(type));
      return false;
    }
    zmq::message_t replyMsg;
    if (!m_adminSocket->recv(replyMsg, zmq::recv_flags::none)) {
      spdlog::warn("[viz] No reply for admin command {}", static_cast<int>(type));
      return false;
    }
    ::lilsim::AdminReply reply;
    if (!reply.ParseFromArray(replyMsg.data(), static_cast<int>(replyMsg.size()))) {
      spdlog::warn("[viz] Failed to parse admin reply");
      return false;
    }
    if (out) {
      *out = reply;
    }
    if (!reply.success()) {
      spdlog::warn("[viz] Admin command {} failed: {}", static_cast<int>(type), reply.message());
    }
    return reply.success();
  } catch (const zmq::error_t& err) {
    spdlog::error("[viz] ZMQ error sending admin command: {}", err.what());
    return false;
  }
}

/** @brief Drain the metadata subscriber and cache the latest payload. */
void Application::pollMetadataUpdates() {
  if (!m_metadataSub) return;
  zmq::message_t msg;
  while (m_metadataSub->recv(msg, zmq::recv_flags::dontwait)) {
    ::lilsim::ModelMetadata metadata;
    if (metadata.ParseFromArray(msg.data(), static_cast<int>(msg.size()))) {
      handleMetadataMessage(metadata);
    }
  }
}

/** @brief Cache metadata broadcast from the simulator. */
void Application::handleMetadataMessage(const ::lilsim::ModelMetadata& msg) {
  m_metadataVersion = msg.schema_version();
  m_lastMetadata = msg;
}

/** @brief Issue a GET_METADATA admin command to fetch the current schema. */
bool Application::requestMetadataSnapshot() {
  ::lilsim::AdminReply reply;
  if (!sendAdminCommand(::lilsim::AdminCommandType::GET_METADATA, {}, &reply)) {
    return false;
  }
  if (reply.has_metadata()) {
    handleMetadataMessage(reply.metadata());
    return true;
  }
  return false;
}

/** @brief Stage a parameter override by index. */
bool Application::stageParamUpdate(size_t index, double value) {
  return sendAdminCommand(::lilsim::AdminCommandType::SET_PARAMS,
                          [&](::lilsim::AdminCommand& cmd) {
                            auto* update = cmd.add_param_updates();
                            update->set_index(static_cast<uint32_t>(index));
                            update->set_value(value);
                          });
}

/** @brief Stage a setting override by index. */
bool Application::stageSettingUpdate(size_t index, int32_t value) {
  return sendAdminCommand(::lilsim::AdminCommandType::SET_SETTINGS,
                          [&](::lilsim::AdminCommand& cmd) {
                            auto* update = cmd.add_setting_updates();
                            update->set_index(static_cast<uint32_t>(index));
                            update->set_value(value);
                          });
}

/** @brief Send a SET_TRACK admin command. */
bool Application::sendSetTrackCommand(const std::string& path) {
  if (path.empty()) {
    return false;
  }
  return sendAdminCommand(::lilsim::AdminCommandType::SET_TRACK,
                          [&](::lilsim::AdminCommand& cmd) {
                            cmd.set_track_path(path);
                          });
}

/** @brief Dispatch LOAD_PARAM_PROFILE or CLEAR_PARAM_PROFILE commands. */
bool Application::sendProfileCommand(::lilsim::AdminCommandType type, const std::string& path) {
  if (type == ::lilsim::AdminCommandType::LOAD_PARAM_PROFILE && path.empty()) {
    return false;
  }
  return sendAdminCommand(type, [&](::lilsim::AdminCommand& cmd) {
    if (!path.empty()) {
      cmd.set_param_profile_path(path);
    }
  });
}

/** @brief Toggle sync/async control modes via admin command. */
/** @brief Send a SET_CONTROL_MODE admin command with sync + source toggles. */
bool Application::sendSetModeCommand(bool sync, bool externalControl) {
  return sendAdminCommand(::lilsim::AdminCommandType::SET_CONTROL_MODE,
                          [&](::lilsim::AdminCommand& cmd) {
                            cmd.set_sync_mode(sync);
                            cmd.set_control_period_ms(m_simulator.getControlPeriodMs());
                            cmd.set_use_external_control(externalControl);
                          });
}

/** @brief Publish GUI-originated control inputs. */
void Application::sendGuiControlInputs(const std::vector<double>& inputs) {
  if (!m_controlPub || inputs.empty()) {
    return;
  }
  if (m_metadataVersion == 0 && !requestMetadataSnapshot()) {
    return;
  }
  ::lilsim::ControlAsync msg;
  auto* header = msg.mutable_header();
  uint64_t tick = m_sceneDB.tick.load(std::memory_order_relaxed);
  header->set_tick(tick);
  header->set_sim_time(tick * m_simulator.getDt());
  header->set_version(1);
  msg.set_metadata_version(m_metadataVersion);
  for (double val : inputs) {
    msg.add_input_values(val);
  }
  std::string payload = msg.SerializeAsString();
  zmq::message_t m(payload.size());
  std::memcpy(m.data(), payload.data(), payload.size());
  try {
    m_controlPub->send(m, zmq::send_flags::none);
  } catch (const zmq::error_t& err) {
    spdlog::error("[viz] Failed to publish GUI control: {}", err.what());
  }
}

/** @brief Consume incoming marker messages and update the marker cache. */
void Application::pollMarkerMessages() {
  if (!m_markerSub) {
    return;
  }

  auto mapType = [](::lilsim::MarkerType type) -> MarkerType {
    switch (type) {
      case ::lilsim::MarkerType::TEXT:          return MarkerType::TEXT;
      case ::lilsim::MarkerType::ARROW:         return MarkerType::ARROW;
      case ::lilsim::MarkerType::RECTANGLE:     return MarkerType::RECTANGLE;
      case ::lilsim::MarkerType::CIRCLE:        return MarkerType::CIRCLE;
      case ::lilsim::MarkerType::LINE_STRIP:    return MarkerType::LINE_STRIP;
      case ::lilsim::MarkerType::CIRCLE_LIST:   return MarkerType::CIRCLE_LIST;
      case ::lilsim::MarkerType::TRIANGLE_LIST: return MarkerType::TRIANGLE_LIST;
      case ::lilsim::MarkerType::MESH_2D:       return MarkerType::MESH_2D;
      default:                                  return MarkerType::CIRCLE;
    }
  };

  auto mapFrame = [](::lilsim::FrameId frame) -> FrameId {
    switch (frame) {
      case ::lilsim::FrameId::CAR:   return FrameId::CAR;
      case ::lilsim::FrameId::WORLD:
      default:                       return FrameId::WORLD;
    }
  };

  while (true) {
    comm::MarkerSubscriber::PollResult result = m_markerSub->poll();
    if (result.type == comm::MarkerSubscriber::MessageType::None) {
      break;
    }
    double simTime = m_sceneDB.tick.load(std::memory_order_relaxed) * m_simulator.getDt();

    if (result.type == comm::MarkerSubscriber::MessageType::MarkerArray &&
        result.marker_array) {
      for (const auto& markerProto : result.marker_array->markers()) {
        Marker marker;
        marker.type = mapType(markerProto.type());
        marker.pose.setFromXYYaw(
            markerProto.pose().x(),
            markerProto.pose().y(),
            markerProto.pose().yaw());
        marker.color = Color(
            static_cast<uint8_t>(markerProto.color().r()),
            static_cast<uint8_t>(markerProto.color().g()),
            static_cast<uint8_t>(markerProto.color().b()),
            static_cast<uint8_t>(markerProto.color().a()));
        marker.scale = Scale2D(markerProto.scale().x(), markerProto.scale().y());
        if (markerProto.ttl_sec() > 0.0) {
          marker.ttl_sec = markerProto.ttl_sec();
        } else {
          marker.ttl_sec.reset();
        }
        marker.frame_id = mapFrame(markerProto.frame_id());
        marker.text = markerProto.text();
        marker.points.clear();
        marker.points.reserve(markerProto.points_size());
        for (const auto& pt : markerProto.points()) {
          marker.points.emplace_back(pt.x(), pt.y());
        }
        marker.colors.clear();
        marker.colors.reserve(markerProto.colors_size());
        for (const auto& col : markerProto.colors()) {
          marker.colors.emplace_back(
              static_cast<uint8_t>(col.r()),
              static_cast<uint8_t>(col.g()),
              static_cast<uint8_t>(col.b()),
              static_cast<uint8_t>(col.a()));
        }
        marker.visible = markerProto.visible();
        m_markerSystem.addMarker(markerProto.ns(), markerProto.id(), marker, simTime);
      }
    } else if (result.type == comm::MarkerSubscriber::MessageType::MarkerCommand &&
               result.marker_command) {
      const auto& cmd = *result.marker_command;
      switch (cmd.type()) {
        case ::lilsim::MarkerCommandType::DELETE_MARKER:
          m_markerSystem.deleteMarker(cmd.ns(), cmd.id());
          break;
        case ::lilsim::MarkerCommandType::DELETE_NAMESPACE:
          m_markerSystem.deleteNamespace(cmd.ns());
          break;
        case ::lilsim::MarkerCommandType::CLEAR_ALL:
          m_markerSystem.clearAll();
          break;
        default:
          break;
      }
    }
  }
}

/**
 * @brief Synchronize the parameter profile text buffer with simulator state.
 */
void Application::syncParamProfileBuffer() {
  std::string pending = m_simulator.getPendingParamProfilePath();
  std::string active = m_simulator.getActiveParamProfilePath();
  const std::string& source = pending.empty() ? active : pending;
  if (source.empty()) {
    m_paramFileBuffer[0] = '\0';
    return;
  }
  std::strncpy(m_paramFileBuffer, source.c_str(), sizeof(m_paramFileBuffer) - 1);
  m_paramFileBuffer[sizeof(m_paramFileBuffer) - 1] = '\0';
}

void Application::scanTrackDirectory() {
  m_availableTracks.clear();
  namespace fs = std::filesystem;
  try {
    if (fs::exists(m_trackDirBuffer) && fs::is_directory(m_trackDirBuffer)) {
      for (const auto& entry : fs::directory_iterator(m_trackDirBuffer)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
          m_availableTracks.push_back(entry.path().stem().string());
        }
      }
      std::sort(m_availableTracks.begin(), m_availableTracks.end());
    }
  } catch (...) {}
}

void Application::mainLoop() {
  double currentTime = glfwGetTime();
  double deltaTime = currentTime - m_lastFrameTime;
  if (deltaTime < m_targetFrameTime) {
    std::this_thread::sleep_for(std::chrono::duration<double>(m_targetFrameTime - deltaTime));
    currentTime = glfwGetTime();
  }
  m_lastFrameTime = currentTime;

  glfwPollEvents();
  handleInput();

  WGPUSurfaceTexture m_surfaceTexture = {};
  wgpuSurfaceGetCurrentTexture(m_surface, &m_surfaceTexture);

  if (m_surfaceTexture.status != WGPUSurfaceGetCurrentTextureStatus_SuccessOptimal && 
      m_surfaceTexture.status != WGPUSurfaceGetCurrentTextureStatus_SuccessSuboptimal) {
      if (m_surfaceTexture.texture) wgpuTextureRelease(m_surfaceTexture.texture);
      return; 
  }

  ImGui_ImplWGPU_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2((float)m_width, (float)m_height);
  ImGui::NewFrame();

  render2D();

  ImGui::Render();

  WGPUTextureViewDescriptor viewDesc = {};
  viewDesc.format = m_surfaceFormat;
  viewDesc.dimension = WGPUTextureViewDimension_2D;
  viewDesc.baseMipLevel = 0;
  viewDesc.mipLevelCount = 1;
  viewDesc.baseArrayLayer = 0;
  viewDesc.arrayLayerCount = 1;
  viewDesc.aspect = WGPUTextureAspect_All;
  WGPUTextureView backbuffer = wgpuTextureCreateView(m_surfaceTexture.texture, &viewDesc);

  WGPURenderPassColorAttachment colorAttachment = {};
  colorAttachment.view = backbuffer;
  colorAttachment.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  colorAttachment.loadOp = WGPULoadOp_Clear;
  colorAttachment.storeOp = WGPUStoreOp_Store;
  colorAttachment.clearValue = {m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]};

  WGPURenderPassDescriptor renderPassDesc = {};
  renderPassDesc.colorAttachmentCount = 1;
  renderPassDesc.colorAttachments = &colorAttachment;

  WGPUCommandEncoderDescriptor encoderDesc = {};
  WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(m_device, &encoderDesc);
  WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &renderPassDesc);
  ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  WGPUCommandBufferDescriptor cmdBufferDesc = {};
  WGPUCommandBuffer cmdBuffer = wgpuCommandEncoderFinish(encoder, &cmdBufferDesc);
  wgpuCommandEncoderRelease(encoder);
  wgpuQueueSubmit(m_queue, 1, &cmdBuffer);
  wgpuCommandBufferRelease(cmdBuffer);
  wgpuSurfacePresent(m_surface);
  wgpuTextureViewRelease(backbuffer);
  wgpuTextureRelease(m_surfaceTexture.texture);
}

} // namespace viz
