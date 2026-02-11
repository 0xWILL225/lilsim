#include "viz.hpp"

#include "KeyBindings.hpp"
#include "TextureManager.hpp"
#include "TrackLoader.hpp"

#include "models/cars/include/base.h"

#include "imgui_toggle/imgui_toggle.h"

#include <GLFW/glfw3native.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

#include <ImGuiFileDialog.h>

#ifdef Success
#undef Success
#endif
#ifdef None
#undef None
#endif

#include <yaml-cpp/yaml.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_wgpu.h>

#define WGPU_STR(s) \
  WGPUStringView { s, WGPU_STRLEN }

namespace {
constexpr ImVec2 kFileDialogMinSize{700.0f, 420.0f};
constexpr ImVec2 kFileDialogMaxSize{FLT_MAX, FLT_MAX};
constexpr const char* kDefaultGuiConfigName = "lilsim_gui_config.yaml";
constexpr const char* kGuiConfigPointerName = "lilsim_gui_config.path";

std::string to_lower_copy(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}
}  // namespace

namespace viz {

Application::Application(scene::SceneDB& db, sim::Simulator& sim,
                         const std::filesystem::path& install_root)
    : m_viewport_panel(nullptr, &m_show_car, &m_show_cones),
      m_right_panel("Admin Panel", SidePanel::Side::Right, 300.0f),
      m_left_panel("Display", SidePanel::Side::Left, 300.0f),
      m_show_car(true),
      m_show_cones(true),
      m_last_connection_probe(std::chrono::steady_clock::now()),
      m_sceneDB(db),
      m_simulator(sim),
      m_width(1400u),
      m_height(800u) {
  try {
    m_install_root = install_root.empty()
                         ? std::filesystem::current_path()
                         : (install_root.is_absolute()
                                ? install_root
                                : std::filesystem::absolute(install_root));
  } catch (...) {
    m_install_root = std::filesystem::current_path();
  }

  std::string default_track_path = (m_install_root / "tracks").string();
  std::strncpy(m_track_dir_buffer, default_track_path.c_str(),
               sizeof(m_track_dir_buffer) - 1);
  m_track_dir_buffer[sizeof(m_track_dir_buffer) - 1] = '\0';

  m_viewport_panel =
      ViewportPanel(&m_marker_system, &m_show_car, &m_show_cones);

  initializeGuiConfigSystem(m_install_root);
  applyBasicGuiConfig();
  applySimConfigFromGui();

  setupPanels();

  scanTrackDirectory();
  refreshAvailableModels();
  restoreModelFromConfig();

  if (initZmqInterface()) {
    requestMetadataSnapshot();
    applyProfileFromConfig();
    restoreTrackFromConfig();
    applyControlModeFromConfig();
  }
  syncParamProfileBuffer();

  // Apply staged timing/profile changes and start in paused state.
  m_simulator.reset();

  m_marker_sub = std::make_unique<comm::MarkerSubscriber>();
  if (!m_marker_sub->start()) {
    spdlog::warn(
        "[viz] Failed to start marker subscriber; remote markers disabled.");
    m_marker_sub.reset();
  }
}

Application::~Application() {
  if (m_marker_sub) {
    m_marker_sub->stop();
  }
}

static void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (app && width > 0 && height > 0) {
    app->onResize(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  }
}

static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  (void)xoffset;
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (app) {
    if (!app->m_viewport_panel.isHovered()) return;
    float zoom_delta = (yoffset > 0) ? 1.1f : 0.9f;
    if (app->m_viewport_panel.cameraMode == ViewportPanel::CameraMode::Free) {
      app->m_viewport_panel.freeCameraZoom *= zoom_delta;
      app->m_viewport_panel.freeCameraZoom =
          std::clamp(app->m_viewport_panel.freeCameraZoom, 5.0f, 500.0f);
    } else {
      app->m_viewport_panel.followCarZoom *= zoom_delta;
      app->m_viewport_panel.followCarZoom =
          std::clamp(app->m_viewport_panel.followCarZoom, 5.0f, 500.0f);
    }
  }
}

static void onAdapterRequestEnded(WGPURequestAdapterStatus status,
                                  WGPUAdapter m_adapter, WGPUStringView message,
                                  void* userdata1, void* userdata2) {
  (void)userdata2;
  (void)message;
  if (status == WGPURequestAdapterStatus_Success)
    *(WGPUAdapter*)userdata1 = m_adapter;
}

static void onDeviceRequestEnded(WGPURequestDeviceStatus status,
                                 WGPUDevice m_device, WGPUStringView message,
                                 void* userdata1, void* userdata2) {
  (void)userdata2;
  (void)message;
  if (status == WGPURequestDeviceStatus_Success)
    *(WGPUDevice*)userdata1 = m_device;
}

bool Application::initGLFW() {
  if (!glfwInit()) return false;
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  m_window =
      glfwCreateWindow(static_cast<int>(m_width), static_cast<int>(m_height),
                       "lilsim", nullptr, nullptr);
  if (!m_window) return false;
  glfwSetWindowUserPointer(m_window, this);
  glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);
  glfwSetScrollCallback(m_window, scrollCallback);
  refreshWindowTitle();
  return true;
}

bool Application::initWebGPU() {
  WGPUInstanceDescriptor instance_desc = {};
  m_instance = wgpuCreateInstance(&instance_desc);
  if (!m_instance) return false;

#if defined(__linux__)
  WGPUSurfaceSourceXlibWindow from_xlib = {};
  from_xlib.chain.sType = WGPUSType_SurfaceSourceXlibWindow;
  from_xlib.display = glfwGetX11Display();
  from_xlib.window = glfwGetX11Window(m_window);
  WGPUSurfaceDescriptor surface_desc = {};
  surface_desc.nextInChain =
      reinterpret_cast<const WGPUChainedStruct*>(&from_xlib);
  m_surface = wgpuInstanceCreateSurface(m_instance, &surface_desc);
#elif defined(__APPLE__)
#elif defined(_WIN32)
#endif

  if (!m_surface) return false;

  WGPURequestAdapterOptions adapter_opts = {};
  adapter_opts.compatibleSurface = m_surface;
  adapter_opts.powerPreference = WGPUPowerPreference_HighPerformance;
  WGPURequestAdapterCallbackInfo cb_info = {};
  cb_info.mode = WGPUCallbackMode_AllowSpontaneous;
  cb_info.callback = onAdapterRequestEnded;
  cb_info.userdata1 = &m_adapter;
  wgpuInstanceRequestAdapter(m_instance, &adapter_opts, cb_info);

  while (!m_adapter) {
    wgpuInstanceProcessEvents(m_instance);
    glfwPollEvents();
  }

  WGPUDeviceDescriptor device_desc = {};
  device_desc.label = WGPU_STR("Main Device");
  WGPURequestDeviceCallbackInfo dev_cb_info = {};
  dev_cb_info.mode = WGPUCallbackMode_AllowSpontaneous;
  dev_cb_info.callback = onDeviceRequestEnded;
  dev_cb_info.userdata1 = &m_device;
  wgpuAdapterRequestDevice(m_adapter, &device_desc, dev_cb_info);

  while (!m_device) {
    wgpuInstanceProcessEvents(m_instance);
    glfwPollEvents();
  }

  m_queue = wgpuDeviceGetQueue(m_device);

  WGPUSurfaceConfiguration config = {};
  config.device = m_device;
  config.format = m_surfaceFormat;
  config.usage = WGPUTextureUsage_RenderAttachment;
  config.width = static_cast<uint32_t>(m_width);
  config.height = static_cast<uint32_t>(m_height);
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

void Application::onResize(uint32_t new_width, uint32_t new_height) {
  if (new_width == 0 || new_height == 0) return;
  m_width = new_width;
  m_height = new_height;
  if (m_guiConfig.window_width != new_width ||
      m_guiConfig.window_height != new_height) {
    m_guiConfig.window_width = new_width;
    m_guiConfig.window_height = new_height;
    markGuiConfigDirty();
  }
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
  m_selected_model_index = common::kNullIndex;
  std::string configured_path = m_guiConfig.modelPath;
  for (size_t i = 0; i < m_availableModels.size(); ++i) {
    if (!configured_path.empty() &&
        m_availableModels[i].path == configured_path) {
      m_selected_model_index = i;
      break;
    }
    if (m_availableModels[i].name == current) {
      m_selected_model_index = i;
      break;
    }
  }
}

void Application::onModelChanged() {
  refreshAvailableModels();  // Sync dropdown
  const auto* desc = m_simulator.getCurrentModelDescriptor();
  if (!desc) return;

  m_ui_param_values.assign(desc->param_values,
                           desc->param_values + desc->num_params);
  m_ui_setting_values.assign(desc->setting_values,
                             desc->setting_values + desc->num_settings);

  // Cache inputs
  m_input_idx_wheel_angle = common::kNullIndex;
  m_input_idx_wheel_rate = common::kNullIndex;
  m_input_idx_ax = common::kNullIndex;
  for (size_t i = 0; i < desc->num_inputs; ++i) {
    std::string name = desc->input_names[i];
    if (name == "steering_wheel_angle_input")
      m_input_idx_wheel_angle = i;
    else if (name == "steering_wheel_rate_input")
      m_input_idx_wheel_rate = i;
    else if (name == "ax" || name == "ax_input")
      m_input_idx_ax = i;
  }

  // Cache states
  m_state_idx_x = common::kNullIndex;
  m_state_idx_y = common::kNullIndex;
  m_state_idx_yaw = common::kNullIndex;
  m_state_idx_v = common::kNullIndex;
  m_state_idx_ax = common::kNullIndex;
  m_state_idx_steer_wheel_angle = common::kNullIndex;
  m_state_idx_steer_wheel_rate = common::kNullIndex;
  m_state_idx_wheel_fl = common::kNullIndex;
  m_state_idx_wheel_fr = common::kNullIndex;

  for (size_t i = 0; i < desc->num_states; ++i) {
    std::string name = desc->state_names[i];
    if (name == "x")
      m_state_idx_x = i;
    else if (name == "y")
      m_state_idx_y = i;
    else if (name == "yaw")
      m_state_idx_yaw = i;
    else if (name == "v")
      m_state_idx_v = i;
    else if (name == "ax")
      m_state_idx_ax = i;
    else if (name == "steering_wheel_angle")
      m_state_idx_steer_wheel_angle = i;
    else if (name == "steering_wheel_rate")
      m_state_idx_steer_wheel_rate = i;
    else if (name == "wheel_fl_angle")
      m_state_idx_wheel_fl = i;
    else if (name == "wheel_fr_angle")
      m_state_idx_wheel_fr = i;
  }

  // Cache params
  m_param_idx_wheelbase = common::kNullIndex;
  m_param_idx_track_width = common::kNullIndex;
  for (size_t i = 0; i < desc->num_params; ++i) {
    std::string name = desc->param_names[i];
    if (name == "wheelbase")
      m_param_idx_wheelbase = i;
    else if (name == "track_width")
      m_param_idx_track_width = i;
  }

  // Cache steering mode setting index
  m_setting_idx_steering_mode = common::kNullIndex;
  for (size_t i = 0; i < desc->num_settings; ++i) {
    std::string name = desc->setting_names[i];
    if (name == "steering_input_mode") {
      m_setting_idx_steering_mode = i;
      break;
    }
  }

  syncParamProfileBuffer();
}

void Application::handleInput() {
  pollMetadataUpdates();
  pollMarkerMessages();
  m_gui_control_source = !m_simulator.isExternalControlEnabled();
  if (m_simulator.checkAndClearModelChanged()) {
    onModelChanged();
  }

  if (m_selected_model_index == common::kNullIndex &&
      !m_availableModels.empty()) {
    onModelChanged();
  }

  const scene::Scene& scene = m_sceneDB.snapshot();
  double sim_time =
      static_cast<double>(m_sceneDB.tick.load(std::memory_order_relaxed)) *
      m_simulator.getDt();
  ViewportPanel::RenderState render_state;
  render_state.sim_time = sim_time;
  if (m_state_idx_x != common::kNullIndex &&
      m_state_idx_x < scene.car_state_values.size())
    render_state.x = scene.car_state_values[m_state_idx_x];
  if (m_state_idx_y != common::kNullIndex &&
      m_state_idx_y < scene.car_state_values.size())
    render_state.y = scene.car_state_values[m_state_idx_y];
  if (m_state_idx_yaw != common::kNullIndex &&
      m_state_idx_yaw < scene.car_state_values.size())
    render_state.yaw = scene.car_state_values[m_state_idx_yaw];

  m_viewport_panel.handleInput(m_window, render_state);

  static bool pause_key_was_pressed = false;
  bool pause_key_pressed =
      glfwGetKey(m_window, gKeyBindings.pauseSimulation) == GLFW_PRESS;
  if (pause_key_pressed && !pause_key_was_pressed) {
    if (m_simulator.isPaused())
      m_simulator.resume();
    else
      m_simulator.pause();
  }
  pause_key_was_pressed = pause_key_pressed;

  static bool reset_key_was_pressed = false;
  bool reset_key_pressed =
      glfwGetKey(m_window, gKeyBindings.resetSimulation) == GLFW_PRESS;
  if (reset_key_pressed && !reset_key_was_pressed) {
    m_simulator.reset();
  }
  reset_key_was_pressed = reset_key_pressed;

  if (m_gui_control_source) {
    const auto* desc = m_simulator.getCurrentModelDescriptor();
    if (desc) {
      std::vector<double> input(desc->num_inputs, 0.0);

      bool key_w =
          glfwGetKey(m_window, gKeyBindings.carAccelerate) == GLFW_PRESS;
      bool key_s = glfwGetKey(m_window, gKeyBindings.carBrake) == GLFW_PRESS;
      bool key_a =
          glfwGetKey(m_window, gKeyBindings.carSteerLeft) == GLFW_PRESS;
      bool key_d =
          glfwGetKey(m_window, gKeyBindings.carSteerRight) == GLFW_PRESS;

      bool steering_rate_mode = false;
      if (m_setting_idx_steering_mode != common::kNullIndex &&
          desc->setting_values) {
        steering_rate_mode =
            desc->setting_values[m_setting_idx_steering_mode] == 1;
      }

      if (m_input_idx_ax != common::kNullIndex) {
        double max_ax = desc->input_max[m_input_idx_ax];
        double min_ax = desc->input_min[m_input_idx_ax];
        if (key_w)
          input[m_input_idx_ax] = max_ax;
        else if (key_s)
          input[m_input_idx_ax] = min_ax;
      }

      if (steering_rate_mode) {
        if (m_input_idx_wheel_rate != common::kNullIndex) {
          double max_rate = desc->input_max[m_input_idx_wheel_rate];
          double min_rate = desc->input_min[m_input_idx_wheel_rate];
          if (key_a)
            input[m_input_idx_wheel_rate] = max_rate;
          else if (key_d)
            input[m_input_idx_wheel_rate] = min_rate;
        }
      } else {
        if (m_input_idx_wheel_angle != common::kNullIndex) {
          double max_angle = desc->input_max[m_input_idx_wheel_angle];
          double min_angle = desc->input_min[m_input_idx_wheel_angle];
          if (key_a)
            input[m_input_idx_wheel_angle] = max_angle;
          else if (key_d)
            input[m_input_idx_wheel_angle] = min_angle;
        }
      }

      m_simulator.setInput(input);
    }
  }
}

void Application::setupPanels() {
  m_left_panel.addSection("Simulated Objects", [this]() {
    if (ImGui::Checkbox("Car", &m_show_car)) {
      m_guiConfig.showCar = m_show_car;
      markGuiConfigDirty();
    }
    if (ImGui::Checkbox("Cones", &m_show_cones)) {
      m_guiConfig.showCones = m_show_cones;
      markGuiConfigDirty();
    }
  });

  m_left_panel.addSection("Markers", [this]() {
    const auto& markers = m_marker_system.getMarkers();
    if (markers.empty()) {
      ImGui::TextUnformatted("No markers");
      return;
    }

    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginListBox("##MarkerList", ImVec2(-1, 200.0f))) {
      std::string current_ns;
      for (const auto& entry : markers) {
        const auto& key = entry.first;
        const auto& marker = entry.second;

        if (current_ns != key.ns) {
          current_ns = key.ns;
          ImGui::Separator();
          bool ns_visible = m_marker_system.isNamespaceVisible(current_ns);
          if (m_guiConfig.markerNamespaceVisibility.find(current_ns) ==
              m_guiConfig.markerNamespaceVisibility.end()) {
            m_guiConfig.markerNamespaceVisibility[current_ns] = ns_visible;
          }
          if (ImGui::Checkbox(("##ns_" + current_ns).c_str(), &ns_visible)) {
            m_marker_system.setNamespaceVisible(current_ns, ns_visible);
            m_guiConfig.markerNamespaceVisibility[current_ns] = ns_visible;
            markGuiConfigDirty();
          }
          ImGui::SameLine();
          ImGui::TextUnformatted(current_ns.c_str());
        }

        ImGui::Indent();
        bool marker_visible = marker.visible;
        std::string label =
            "##marker_" + current_ns + "_" + std::to_string(key.id);
        if (ImGui::Checkbox(label.c_str(), &marker_visible)) {
          m_marker_system.setMarkerVisible(current_ns, key.id, marker_visible);
        }
        ImGui::SameLine();
        ImGui::Text("ID %d (%s)", key.id,
                    marker.type == MarkerType::CAR_SPRITE ? "car" : "marker");
        ImGui::Unindent();
      }
      ImGui::EndListBox();
    }
  });

  m_right_panel.addSection("Simulation Control", [this]() {
    bool is_paused = m_simulator.isPaused();

    if (is_paused) {
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.6f, 0.0f, 1.0f));
    } else {
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.0f, 0.0f, 1.0f));
    }

    if (ImGui::Button(is_paused ? "Resume" : "Pause", ImVec2(-1, 30))) {
      sendAdminCommand(is_paused ? ::lilsim::AdminCommandType::RUN
                                 : ::lilsim::AdminCommandType::PAUSE);
    }

    ImGui::PopStyleColor();

    ImGui::Text("Step size (N):");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    ImGui::InputInt("##StepN", &m_stepN, 0, 0);
    m_stepN = std::max(1, m_stepN);

    float button_width = ImGui::GetContentRegionAvail().x / 2.0f - 4.0f;
    if (ImGui::Button("Step N", ImVec2(button_width, 30))) {
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
    double active_dt_ms = m_simulator.getDt() * 1000.0;
    double requested_dt_ms = m_simulator.getRequestedDt() * 1000.0;
    double dt_input = requested_dt_ms;
    ImGui::TextUnformatted("Timestep dt (ms)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputDouble("##SimDtInput", &dt_input, 0.1, 1.0, "%.2f")) {
      dt_input = std::clamp(dt_input, 1.0, 1000.0);
      m_simulator.requestDt(dt_input / 1000.0);
      if (std::abs(m_guiConfig.timestepMs - dt_input) > 1e-3) {
        m_guiConfig.timestepMs = dt_input;
        markGuiConfigDirty();
      }
    }
    ImGui::SameLine();
    if (std::abs(dt_input - active_dt_ms) > 1e-3) {
      ImGui::TextUnformatted("(pending, applies on reset)");
    } else {
      ImGui::TextUnformatted("(active)");
    }

    double run_speed = m_simulator.getRunSpeed();
    ImGui::TextUnformatted("Run Speed (0.1x - 3x)");
    ImGui::SetNextItemWidth(-1);
    float min_speed = 0.1f;
    float max_speed = 3.0f;
    float run_speed_float = static_cast<float>(run_speed);
    if (ImGui::SliderFloat("##SimRunSpeed", &run_speed_float, min_speed,
                           max_speed, "%.1fx", ImGuiSliderFlags_AlwaysClamp)) {
      run_speed_float = std::round(run_speed_float * 10.0f) / 10.0f;
      m_simulator.setRunSpeed(static_cast<double>(run_speed_float));
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Input Source");
    ImGuiToggleConfig toggle_cfg;
    toggle_cfg.Flags = ImGuiToggleFlags_Animated | ImGuiToggleFlags_Bordered |
                       ImGuiToggleFlags_Shadowed;
    toggle_cfg.Width = 44.0f;
    toggle_cfg.Height = 20.0f;

    bool gui_toggle = m_gui_control_source;
    ImGui::PushID("InputSourceToggle");
    if (ImGui::Toggle("##InputSource", &gui_toggle, toggle_cfg)) {
      bool target_gui = gui_toggle;
      bool target_sync = target_gui ? false : m_simulator.isSyncMode();

      if (sendSetModeCommand(target_sync, !target_gui)) {
        m_gui_control_source = target_gui;
        bool changed = false;
        if (m_guiConfig.guiInputSource != target_gui) {
          m_guiConfig.guiInputSource = target_gui;
          changed = true;
        }
        if (m_guiConfig.syncControlMode != target_sync) {
          m_guiConfig.syncControlMode = target_sync;
          changed = true;
        }
        if (changed) {
          markGuiConfigDirty();
        }
      } else {
        gui_toggle = !target_gui;
      }
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(gui_toggle ? "GUI" : "ZeroMQ Client");
    ImGui::PopID();

    ImGui::TextUnformatted("Control Mode");
    bool control_disabled = gui_toggle;
    bool async_state = control_disabled ? true : !m_simulator.isSyncMode();
    bool async_toggle = async_state;

    ImGui::PushID("ControlModeToggle");
    if (control_disabled) {
      ImGui::BeginDisabled();
    }
    if (ImGui::Toggle("##ControlMode", &async_toggle, toggle_cfg) &&
        !control_disabled) {
      bool requested_sync = !async_toggle;
      if (sendSetModeCommand(requested_sync, !m_gui_control_source)) {
        if (m_guiConfig.syncControlMode != requested_sync) {
          m_guiConfig.syncControlMode = requested_sync;
          markGuiConfigDirty();
        }
      } else {
        async_toggle = async_state;
      }
    }
    if (control_disabled) {
      ImGui::EndDisabled();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted(async_toggle ? "Asynchronous" : "Synchronous");

    ImGui::PopID();

    bool sync_controls_enabled = !async_toggle;
    if (!sync_controls_enabled) {
      ImGui::BeginDisabled();
    }

    ImGui::TextUnformatted("Control Period (ms)");
    double control_period_input =
        m_simulator.getRequestedControlPeriodMilliseconds();
    double active_control_period = m_simulator.getControlPeriodMilliseconds();
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputDouble("##ControlPeriodMs", &control_period_input, 0.5, 1.0,
                           "%.1f")) {
      control_period_input = std::max(1.0, control_period_input);
      m_simulator.requestControlPeriodMs(control_period_input);
      if (std::abs(m_guiConfig.controlPeriodMs - control_period_input) > 0.05) {
        m_guiConfig.controlPeriodMs = control_period_input;
        markGuiConfigDirty();
      }
    }
    ImGui::SameLine();
    if (std::abs(control_period_input - active_control_period) > 0.5) {
      ImGui::TextUnformatted("(pending, applies on reset)");
    } else {
      ImGui::TextUnformatted("(active)");
    }

    ImGui::TextUnformatted("Control Delay (ms)");
    double control_delay_input =
        m_simulator.getRequestedControlDelayMilliseconds();
    double active_control_delay = m_simulator.getControlDelayMilliseconds();
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputDouble("##ControlDelayMs", &control_delay_input, 0.5, 1.0,
                           "%.1f")) {
      control_delay_input = std::max(1.0, control_delay_input);
      m_simulator.requestControlDelayMs(control_delay_input);
      if (std::abs(m_guiConfig.controlDelayMs - control_delay_input) > 0.05) {
        m_guiConfig.controlDelayMs = control_delay_input;
        markGuiConfigDirty();
      }
    }
    ImGui::SameLine();
    if (std::abs(control_delay_input - active_control_delay) > 0.5) {
      ImGui::TextUnformatted("(pending, applies on reset)");
    } else {
      ImGui::TextUnformatted("(active)");
    }

    if (!sync_controls_enabled) {
      ImGui::EndDisabled();
    }
  });

  m_right_panel.addSection("Model Selection", [this]() {
    std::string current_name = "None";
    if (m_selected_model_index != common::kNullIndex &&
        m_selected_model_index < m_availableModels.size()) {
      current_name = m_availableModels[m_selected_model_index].name;
    } else {
      // If no selection but sim has model, show it
      std::string sim_model = m_simulator.getCurrentModelName();
      if (!sim_model.empty()) current_name = sim_model;
    }

    if (ImGui::BeginCombo("Model", current_name.c_str())) {
      for (size_t i = 0; i < m_availableModels.size(); ++i) {
        bool is_selected = (m_selected_model_index == i);
        if (ImGui::Selectable(m_availableModels[i].name.c_str(), is_selected)) {
          m_selected_model_index = i;
          if (m_simulator.loadModel(m_availableModels[i].path)) {
            onModelChanged();
            if (m_guiConfig.modelPath != m_availableModels[i].path) {
              m_guiConfig.modelPath = m_availableModels[i].path;
              markGuiConfigDirty();
            }
          }
        }
        if (is_selected) ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }

    if (ImGui::Button("Refresh Models")) {
      refreshAvailableModels();
    }

    ImGui::Separator();
    ImGui::Text("Parameter Profile");
    const ImGuiStyle& style = ImGui::GetStyle();
    const float browse_width =
        ImGui::CalcTextSize("Browse").x + style.FramePadding.x * 2.0f + 10.0f;
    float first_row_width = ImGui::GetContentRegionAvail().x;
    float input_width =
        std::max(50.0f, first_row_width - browse_width - style.ItemSpacing.x);
    ImGui::SetNextItemWidth(input_width);
    ImGui::InputText("##paramProfilePath", m_param_file_buffer,
                     sizeof(m_param_file_buffer));
    ImGui::SameLine();
    if (ImGui::Button("Browse##paramProfile", ImVec2(browse_width, 0.0f))) {
      IGFD::FileDialogConfig config;
      std::filesystem::path initial =
          m_param_file_buffer[0]
              ? std::filesystem::path(m_param_file_buffer).parent_path()
              : std::filesystem::current_path();
      config.path = initial.empty() ? "." : initial.string();
      config.flags =
          ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_DontShowHiddenFiles;
      ImGuiFileDialog::Instance()->OpenDialog(
          "ParamProfileDialog", "Select Parameter File", ".yaml,.yml", config);
    }

    ImGui::Spacing();
    float second_row_width = ImGui::GetContentRegionAvail().x;
    float half_width = (second_row_width - style.ItemSpacing.x) * 0.5f;
    if (ImGui::Button("Load Profile", ImVec2(half_width, 0.0f))) {
      if (sendProfileCommand(::lilsim::AdminCommandType::LOAD_PARAM_PROFILE,
                             std::string(m_param_file_buffer))) {
        syncParamProfileBuffer();
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear##paramProfile", ImVec2(half_width, 0.0f))) {
      if (sendProfileCommand(::lilsim::AdminCommandType::CLEAR_PARAM_PROFILE)) {
        syncParamProfileBuffer();
      }
    }
    ImGui::TextUnformatted("Profile applies on Reset.");

    if (ImGuiFileDialog::Instance()->Display(
            "ParamProfileDialog", ImGuiWindowFlags_None, kFileDialogMinSize,
            kFileDialogMaxSize)) {
      if (ImGuiFileDialog::Instance()->IsOk()) {
        std::string file_path_name =
            ImGuiFileDialog::Instance()->GetFilePathName();
        if (!file_path_name.empty()) {
          std::strncpy(m_param_file_buffer, file_path_name.c_str(),
                       sizeof(m_param_file_buffer) - 1);
          m_param_file_buffer[sizeof(m_param_file_buffer) - 1] = '\0';
          if (sendProfileCommand(::lilsim::AdminCommandType::LOAD_PARAM_PROFILE,
                                 file_path_name)) {
            syncParamProfileBuffer();
          }
        }
      }
      ImGuiFileDialog::Instance()->Close();
    }
  });

  m_right_panel.addSection("Model Parameters", [this]() {
    const auto* desc = m_simulator.getCurrentModelDescriptor();
    if (!desc) return;

    if (m_simulator.checkAndClearModelChanged()) {
      onModelChanged();
    }

    std::vector<double> pending_snapshot;
    if (m_simulator.consumePendingParamSnapshot(pending_snapshot)) {
      m_ui_param_values = pending_snapshot;
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 4));

    for (size_t i = 0; i < desc->num_params; ++i) {
      if (m_ui_param_values.size() <= i) break;
      double v = m_ui_param_values[i];
      double previous = v;
      double min = desc->param_min[i];
      double max = desc->param_max[i];
      const char* name = desc->param_names[i];

      ImGui::PushStyleColor(ImGuiCol_ChildBg,
                            ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
      ImGui::BeginChild((std::string("##param_") + name).c_str(), ImVec2(0, 50),
                        true);
      ImGui::Text("%s", name);
      ImGui::SetNextItemWidth(-1);

      if (ImGui::InputDouble((std::string("##input_") + name).c_str(), &v, 0.0,
                             0.0, "%.3f")) {
        v = std::clamp(v, min, max);
        if (stageParamUpdate(i, v)) {
          m_ui_param_values[i] = v;
        } else {
          m_ui_param_values[i] = previous;
        }
      }
      ImGui::EndChild();
      ImGui::PopStyleColor();
    }

    ImGui::PopStyleVar();
  });

  m_right_panel.addSection("Model Settings", [this]() {
    const auto* desc = m_simulator.getCurrentModelDescriptor();
    if (!desc) return;

    std::vector<uint32_t> pending_settings;
    if (m_simulator.consumePendingSettingSnapshot(pending_settings)) {
      m_ui_setting_values = pending_settings;
    }

    for (size_t i = 0; i < desc->num_settings; ++i) {
      if (m_ui_setting_values.size() <= i) break;
      int v = static_cast<int>(m_ui_setting_values[i]);
      uint32_t prev = m_ui_setting_values[i];
      const char* name = desc->setting_names[i];

      std::vector<const char*> options;
      for (size_t k = 0; k < desc->num_setting_options; ++k) {
        if (desc->setting_option_setting_index[k] == i) {
          options.push_back(desc->setting_option_names[k]);
        }
      }

      if (!options.empty()) {
        ImGui::SetNextItemWidth(-1);
        if (ImGui::Combo(name, &v, options.data(),
                         static_cast<int>(options.size()))) {
          uint32_t new_val = static_cast<uint32_t>(v);
          if (stageSettingUpdate(i, new_val)) {
            m_ui_setting_values[i] = new_val;
          } else {
            m_ui_setting_values[i] = prev;
          }
        }
      }
    }
  });

  m_right_panel.addSection("Track Loading", [this]() {
    ImGui::Text("Track Directory:");
    ImGui::SetNextItemWidth(-80);
    if (ImGui::InputText("##trackdir", m_track_dir_buffer,
                         sizeof(m_track_dir_buffer))) {
      scanTrackDirectory();
    }
    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
      IGFD::FileDialogConfig config;
      config.path = m_track_dir_buffer[0] ? m_track_dir_buffer : ".";
      config.flags = ImGuiFileDialogFlags_Modal |
                     ImGuiFileDialogFlags_DisableCreateDirectoryButton |
                     ImGuiFileDialogFlags_OptionalFileName;
      ImGuiFileDialog::Instance()->OpenDialog(
          "TrackDirDialog", "Select Track Directory", nullptr, config);
    }

    if (ImGuiFileDialog::Instance()->Display(
            "TrackDirDialog", ImGuiWindowFlags_None, kFileDialogMinSize,
            kFileDialogMaxSize)) {
      if (ImGuiFileDialog::Instance()->IsOk()) {
        std::string selected_path =
            ImGuiFileDialog::Instance()->GetCurrentPath();
        if (!selected_path.empty()) {
          std::strncpy(m_track_dir_buffer, selected_path.c_str(),
                       sizeof(m_track_dir_buffer) - 1);
          m_track_dir_buffer[sizeof(m_track_dir_buffer) - 1] = '\0';
          scanTrackDirectory();
        }
      }
      ImGuiFileDialog::Instance()->Close();
    }

    ImGui::Text("Tracks:");
    ImGui::BeginChild("##tracklist", ImVec2(0, 150), true);
    for (size_t i = 0; i < m_availableTracks.size(); ++i) {
      bool is_selected = (m_selected_track_index == i);
      if (ImGui::Selectable(m_availableTracks[i].c_str(), is_selected,
                            ImGuiSelectableFlags_AllowDoubleClick)) {
        m_selected_track_index = i;
        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
          std::string filepath = std::string(m_track_dir_buffer) + "/" +
                                 m_availableTracks[i] + ".csv";
          sendSetTrackCommand(filepath);
        }
      }
    }
    ImGui::EndChild();

    if (ImGui::Button("Load Selected Track", ImVec2(-1, 30))) {
      if (m_selected_track_index != common::kNullIndex &&
          m_selected_track_index < m_availableTracks.size()) {
        std::string filepath = std::string(m_track_dir_buffer) + "/" +
                               m_availableTracks[m_selected_track_index] +
                               ".csv";
        sendSetTrackCommand(filepath);
      }
    }
  });

  m_right_panel.addSection("Status", [this]() {
    const scene::Scene& scene = m_sceneDB.snapshot();
    uint64_t tick = m_sceneDB.tick.load();
    double sim_time = static_cast<double>(tick) * m_simulator.getDt();
    ImGui::Text("Tick: %lu", (unsigned long)tick);
    ImGui::Text("Sim Time: %.3f s", sim_time);

    if (m_state_idx_x != common::kNullIndex &&
        m_state_idx_y != common::kNullIndex &&
        m_state_idx_y < scene.car_state_values.size()) {
      double x = scene.car_state_values[m_state_idx_x];
      double y = scene.car_state_values[m_state_idx_y];
      ImGui::Text("Pos: (%.2f, %.2f)", x, y);
    }
    if (m_state_idx_v != common::kNullIndex &&
        m_state_idx_v < scene.car_state_values.size()) {
      ImGui::Text("V: %.2f m/s", scene.car_state_values[m_state_idx_v]);
    }
  });
}

void Application::render2D() {
  const scene::Scene& scene = m_sceneDB.snapshot();
  uint64_t tick = m_sceneDB.tick.load(std::memory_order_relaxed);
  double sim_time = static_cast<double>(tick) * m_simulator.getDt();
  m_marker_system.update(sim_time);

  float menu_bar_height = handleMenuBar();
  handleConfigShortcuts();

  m_left_panel.setTopMargin(menu_bar_height);
  m_right_panel.setTopMargin(menu_bar_height);
  m_left_panel.draw(static_cast<float>(m_width), static_cast<float>(m_height));
  m_right_panel.draw(static_cast<float>(m_width), static_cast<float>(m_height));

  ViewportPanel::RenderState rs;
  rs.sim_time = sim_time;
  const auto* desc = m_simulator.getCurrentModelDescriptor();

  if (desc && !scene.car_state_values.empty()) {
    if (m_state_idx_x != common::kNullIndex &&
        m_state_idx_x < scene.car_state_values.size())
      rs.x = scene.car_state_values[m_state_idx_x];
    if (m_state_idx_y != common::kNullIndex &&
        m_state_idx_y < scene.car_state_values.size())
      rs.y = scene.car_state_values[m_state_idx_y];
    if (m_state_idx_yaw != common::kNullIndex &&
        m_state_idx_yaw < scene.car_state_values.size())
      rs.yaw = scene.car_state_values[m_state_idx_yaw];

    if (m_param_idx_wheelbase != common::kNullIndex && desc->param_values &&
        m_param_idx_wheelbase < desc->num_params) {
      rs.wheelbase = desc->param_values[m_param_idx_wheelbase];
    }
    if (m_param_idx_track_width != common::kNullIndex && desc->param_values &&
        m_param_idx_track_width < desc->num_params) {
      rs.track_width = desc->param_values[m_param_idx_track_width];
    }

    if (m_state_idx_ax != common::kNullIndex &&
        m_state_idx_ax < scene.car_state_values.size()) {
      rs.ax = scene.car_state_values[m_state_idx_ax];
    }

    if (m_state_idx_steer_wheel_angle != common::kNullIndex &&
        m_state_idx_steer_wheel_angle < scene.car_state_values.size()) {
      rs.steering_wheel_angle =
          scene.car_state_values[m_state_idx_steer_wheel_angle];
    }

    if (m_state_idx_steer_wheel_rate != common::kNullIndex &&
        m_state_idx_steer_wheel_rate < scene.car_state_values.size()) {
      rs.steering_wheel_rate =
          scene.car_state_values[m_state_idx_steer_wheel_rate];
    }

    if (m_state_idx_wheel_fl != common::kNullIndex &&
        m_state_idx_wheel_fl < scene.car_state_values.size()) {
      rs.wheel_fl_angle = scene.car_state_values[m_state_idx_wheel_fl];
    }

    if (m_state_idx_wheel_fr != common::kNullIndex &&
        m_state_idx_wheel_fr < scene.car_state_values.size()) {
      rs.wheel_fr_angle = scene.car_state_values[m_state_idx_wheel_fr];
    }
  }

  rs.cones = &scene.cones;

  float left_w = m_left_panel.getWidth();
  float right_w = m_right_panel.getWidth();
  float viewport_y = menu_bar_height;
  float viewport_height =
      std::max(0.0f, static_cast<float>(m_height) - menu_bar_height);
  m_viewport_panel.draw(left_w, viewport_y,
                        static_cast<float>(m_width) - left_w - right_w,
                        viewport_height, rs);

  capturePanelLayoutState();
  processGuiConfigDialogs();
}

/** @brief Initialize GUI ZMQ sockets and connect to simulator inproc endpoints.
 */
bool Application::initZmqInterface() {
  m_simulator.setCommEnable(true);
  m_zmq_context = m_simulator.getCommContext();
  if (!m_zmq_context) {
    spdlog::error("[viz] Failed to acquire ZMQ context from simulator");
    return false;
  }
  try {
    m_admin_socket =
        std::make_unique<zmq::socket_t>(*m_zmq_context, zmq::socket_type::req);
    m_admin_socket->connect(comm::endpoints::ADMIN_REP_INPROC);

    m_control_pub =
        std::make_unique<zmq::socket_t>(*m_zmq_context, zmq::socket_type::pub);
    m_control_pub->connect(comm::endpoints::CONTROL_ASYNC_SUB_INPROC);

    m_metadata_sub =
        std::make_unique<zmq::socket_t>(*m_zmq_context, zmq::socket_type::sub);
    m_metadata_sub->connect(comm::endpoints::METADATA_PUB_INPROC);
    m_metadata_sub->set(zmq::sockopt::subscribe, "");
  } catch (const zmq::error_t& err) {
    spdlog::error("[viz] Failed to initialize ZMQ sockets: {}", err.what());
    m_admin_socket.reset();
    m_control_pub.reset();
    m_metadata_sub.reset();
    return false;
  }
  return true;
}

/** @brief Send an AdminCommand via the inproc REQ/REP pair. */
bool Application::sendAdminCommand(
    ::lilsim::AdminCommandType type,
    const std::function<void(::lilsim::AdminCommand&)>& builder,
    ::lilsim::AdminReply* out) {
  if (!m_admin_socket) {
    return false;
  }
  ::lilsim::AdminCommand cmd;
  auto* header = cmd.mutable_header();
  uint64_t tick = m_sceneDB.tick.load(std::memory_order_relaxed);
  header->set_tick(tick);
  header->set_sim_time(static_cast<double>(tick) * m_simulator.getDt());
  header->set_version(1);
  cmd.set_type(type);
  if (builder) builder(cmd);

  std::string payload = cmd.SerializeAsString();
  zmq::message_t request(payload.size());
  std::memcpy(request.data(), payload.data(), payload.size());

  try {
    if (!m_admin_socket->send(request, zmq::send_flags::none)) {
      spdlog::warn("[viz] Failed to send admin command {}",
                   static_cast<int>(type));
      return false;
    }
    zmq::message_t reply_msg;
    if (!m_admin_socket->recv(reply_msg, zmq::recv_flags::none)) {
      spdlog::warn("[viz] No reply for admin command {}",
                   static_cast<int>(type));
      return false;
    }
    ::lilsim::AdminReply reply;
    if (!reply.ParseFromArray(reply_msg.data(),
                              static_cast<int>(reply_msg.size()))) {
      spdlog::warn("[viz] Failed to parse admin reply");
      return false;
    }
    if (out) {
      *out = reply;
    }
    if (!reply.success()) {
      spdlog::warn("[viz] Admin command {} failed: {}", static_cast<int>(type),
                   reply.message());
    }
    return reply.success();
  } catch (const zmq::error_t& err) {
    spdlog::error("[viz] ZMQ error sending admin command: {}", err.what());
    return false;
  }
}

/** @brief Drain the metadata subscriber and cache the latest payload. */
void Application::pollMetadataUpdates() {
  if (!m_metadata_sub) return;
  zmq::message_t msg;
  while (m_metadata_sub->recv(msg, zmq::recv_flags::dontwait)) {
    ::lilsim::ModelMetadata metadata;
    if (metadata.ParseFromArray(msg.data(), static_cast<int>(msg.size()))) {
      handleMetadataMessage(metadata);
    }
  }
}

/** @brief Cache metadata broadcast from the simulator. */
void Application::handleMetadataMessage(const ::lilsim::ModelMetadata& msg) {
  m_metadata_version = msg.schema_version();
  m_last_metadata = msg;
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
bool Application::stageSettingUpdate(size_t index, uint32_t value) {
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
  bool success = sendAdminCommand(
      ::lilsim::AdminCommandType::SET_TRACK,
      [&](::lilsim::AdminCommand& cmd) { cmd.set_track_path(path); });
  if (success) {
    std::filesystem::path track_path = path;
    if (!track_path.is_absolute()) {
      track_path = m_install_root / track_path;
    }
    std::error_code ec;
    std::filesystem::path canonical =
        std::filesystem::weakly_canonical(track_path, ec);
    std::string normalized = ec ? track_path.string() : canonical.string();
    if (m_guiConfig.trackFile != normalized) {
      m_guiConfig.trackFile = normalized;
      markGuiConfigDirty();
    }
  }
  return success;
}

/** @brief Dispatch LOAD_PARAM_PROFILE or CLEAR_PARAM_PROFILE commands. */
bool Application::sendProfileCommand(::lilsim::AdminCommandType type,
                                     const std::string& path) {
  if (type == ::lilsim::AdminCommandType::LOAD_PARAM_PROFILE && path.empty()) {
    return false;
  }
  bool success = sendAdminCommand(type, [&](::lilsim::AdminCommand& cmd) {
    if (!path.empty()) {
      cmd.set_param_profile_path(path);
    }
  });
  if (success) {
    if (type == ::lilsim::AdminCommandType::LOAD_PARAM_PROFILE) {
      if (m_guiConfig.paramProfilePath != path) {
        m_guiConfig.paramProfilePath = path;
        markGuiConfigDirty();
      }
    } else if (type == ::lilsim::AdminCommandType::CLEAR_PARAM_PROFILE) {
      if (!m_guiConfig.paramProfilePath.empty()) {
        m_guiConfig.paramProfilePath.clear();
        markGuiConfigDirty();
      }
    }
  }
  return success;
}

/** @brief Toggle sync/async control modes via admin command. */
/** @brief Send a SET_CONTROL_MODE admin command with sync + source toggles. */
bool Application::sendSetModeCommand(bool sync, bool external_control) {
  return sendAdminCommand(::lilsim::AdminCommandType::SET_CONTROL_MODE,
                          [&](::lilsim::AdminCommand& cmd) {
                            cmd.set_sync_mode(sync);
                            cmd.set_use_external_control(external_control);
                          });
}

/** @brief Publish GUI-originated control inputs. */
void Application::sendGuiControlInputs(const std::vector<double>& inputs) {
  if (!m_control_pub || inputs.empty()) {
    return;
  }
  if (m_metadata_version == 0 && !requestMetadataSnapshot()) {
    return;
  }
  ::lilsim::ControlAsync msg;
  auto* header = msg.mutable_header();
  uint64_t tick = m_sceneDB.tick.load(std::memory_order_relaxed);
  header->set_tick(tick);
  header->set_sim_time(static_cast<double>(tick) * m_simulator.getDt());
  header->set_version(1);
  msg.set_metadata_version(m_metadata_version);
  for (double val : inputs) {
    msg.add_input_values(val);
  }
  std::string payload = msg.SerializeAsString();
  zmq::message_t m(payload.size());
  std::memcpy(m.data(), payload.data(), payload.size());
  try {
    m_control_pub->send(m, zmq::send_flags::none);
  } catch (const zmq::error_t& err) {
    spdlog::error("[viz] Failed to publish GUI control: {}", err.what());
  }
}

/** @brief Consume incoming marker messages and update the marker cache. */
void Application::pollMarkerMessages() {
  if (!m_marker_sub) {
    return;
  }

  auto map_type = [](::lilsim::MarkerType type) -> MarkerType {
    switch (type) {
      case ::lilsim::MarkerType::TEXT:
        return MarkerType::TEXT;
      case ::lilsim::MarkerType::ARROW:
        return MarkerType::ARROW;
      case ::lilsim::MarkerType::RECTANGLE:
        return MarkerType::RECTANGLE;
      case ::lilsim::MarkerType::CIRCLE:
        return MarkerType::CIRCLE;
      case ::lilsim::MarkerType::LINE_STRIP:
        return MarkerType::LINE_STRIP;
      case ::lilsim::MarkerType::CIRCLE_LIST:
        return MarkerType::CIRCLE_LIST;
      case ::lilsim::MarkerType::TRIANGLE_LIST:
        return MarkerType::TRIANGLE_LIST;
      case ::lilsim::MarkerType::MESH_2D:
        return MarkerType::MESH_2D;
      case ::lilsim::MarkerType::CAR_SPRITE:
        return MarkerType::CAR_SPRITE;
      default:
        return MarkerType::CIRCLE;
    }
  };

  auto map_frame = [](::lilsim::FrameId frame) -> FrameId {
    switch (frame) {
      case ::lilsim::FrameId::CAR:
        return FrameId::CAR;
      case ::lilsim::FrameId::WORLD:
      default:
        return FrameId::WORLD;
    }
  };

  while (true) {
    comm::MarkerSubscriber::PollResult result = m_marker_sub->poll();
    if (result.type == comm::MarkerSubscriber::MessageType::None) {
      break;
    }
    double sim_time =
        static_cast<double>(m_sceneDB.tick.load(std::memory_order_relaxed)) *
        m_simulator.getDt();

    if (result.type == comm::MarkerSubscriber::MessageType::MarkerArray &&
        result.marker_array) {
      for (const auto& marker_proto : result.marker_array->markers()) {
        Marker marker;
        marker.type = map_type(marker_proto.type());
        marker.pose.setFromXYYaw(marker_proto.pose().x(),
                                 marker_proto.pose().y(),
                                 marker_proto.pose().yaw());
        marker.color = Color(static_cast<uint8_t>(marker_proto.color().r()),
                             static_cast<uint8_t>(marker_proto.color().g()),
                             static_cast<uint8_t>(marker_proto.color().b()),
                             static_cast<uint8_t>(marker_proto.color().a()));
        marker.scale =
            Scale2D(marker_proto.scale().x(), marker_proto.scale().y());
        if (marker_proto.ttl_sec() > 0.0) {
          marker.ttl_sec = marker_proto.ttl_sec();
        } else {
          marker.ttl_sec.reset();
        }
        marker.frame_id = map_frame(marker_proto.frame_id());
        marker.text = marker_proto.text();
        marker.points.clear();
        marker.points.reserve(static_cast<size_t>(marker_proto.points_size()));
        for (const auto& pt : marker_proto.points()) {
          marker.points.emplace_back(pt.x(), pt.y());
        }
        marker.colors.clear();
        marker.colors.reserve(static_cast<size_t>(marker_proto.colors_size()));
        for (const auto& col : marker_proto.colors()) {
          marker.colors.emplace_back(
              static_cast<uint8_t>(col.r()), static_cast<uint8_t>(col.g()),
              static_cast<uint8_t>(col.b()), static_cast<uint8_t>(col.a()));
        }
        if (marker.type == MarkerType::CAR_SPRITE && marker_proto.has_car()) {
          CarMarkerData car_data;
          const auto& car_proto = marker_proto.car();
          car_data.wheelbase =
              car_proto.wheelbase() > 0.0 ? car_proto.wheelbase() : 1.0;
          car_data.track_width =
              car_proto.track_width() > 0.0 ? car_proto.track_width() : 1.0;
          if (car_proto.has_wheel_fl_angle()) {
            car_data.has_wheel_fl_angle = true;
            car_data.wheel_fl_angle = car_proto.wheel_fl_angle();
          }
          if (car_proto.has_wheel_fr_angle()) {
            car_data.has_wheel_fr_angle = true;
            car_data.wheel_fr_angle = car_proto.wheel_fr_angle();
          }
          if (car_proto.has_opacity()) {
            car_data.opacity = std::clamp(car_proto.opacity(), 0.0, 1.0);
          }
          if (car_proto.has_tint_opacity()) {
            car_data.tint_opacity =
                std::clamp(car_proto.tint_opacity(), 0.0, 1.0);
          }
          marker.car = car_data;
        } else if (marker.type == MarkerType::CAR_SPRITE) {
          spdlog::warn(
              "[viz] CAR_SPRITE marker missing car payload (ns '{}', id {}).",
              marker_proto.ns(), marker_proto.id());
        }
        marker.visible = marker_proto.visible();
        m_marker_system.addMarker(marker_proto.ns(), marker_proto.id(), marker,
                                  sim_time);
      }
    } else if (result.type ==
                   comm::MarkerSubscriber::MessageType::MarkerCommand &&
               result.marker_command) {
      const auto& cmd = *result.marker_command;
      switch (cmd.type()) {
        case ::lilsim::MarkerCommandType::DELETE_MARKER:
          m_marker_system.deleteMarker(cmd.ns(), cmd.id());
          break;
        case ::lilsim::MarkerCommandType::DELETE_NAMESPACE:
          m_marker_system.deleteNamespace(cmd.ns());
          break;
        case ::lilsim::MarkerCommandType::CLEAR_ALL:
          m_marker_system.clearAll();
          break;
        default:
          break;
      }
    }
  }
}

/**
 * @brief Initialize config paths, load the active config, and ensure fallback
 * defaults exist.
 */
void Application::initializeGuiConfigSystem(
    const std::filesystem::path& install_root) {
  namespace fs = std::filesystem;
  fs::path root = install_root.empty() ? fs::current_path() : install_root;
  m_defaultGuiConfigPath = root / kDefaultGuiConfigName;
  m_configPointerPath = root / kGuiConfigPointerName;

  GuiConfig loaded{};
  fs::path active_path = resolveActiveConfigPath();
  if (!loadGuiConfigFromDisk(active_path, loaded)) {
    spdlog::warn(
        "[viz] Failed to load GUI config '{}'; falling back to default.",
        active_path.string());
    if (!loadGuiConfigFromDisk(m_defaultGuiConfigPath, loaded)) {
      spdlog::warn(
          "[viz] Default GUI config '{}' missing or corrupted; regenerating.",
          m_defaultGuiConfigPath.string());
      loaded = GuiConfig{};
      if (!writeGuiConfigToDisk(m_defaultGuiConfigPath, loaded)) {
        spdlog::error(
            "[viz] Unable to write default GUI config '{}'. Using in-memory "
            "defaults.",
            m_defaultGuiConfigPath.string());
      }
    }
    active_path = m_defaultGuiConfigPath;
    ensureConfigPointerMatches(active_path);
  }

  if (loaded.trackDirectory.empty()) {
    loaded.trackDirectory = (root / "tracks").string();
  }

  m_guiConfig = loaded;
  m_activeGuiConfigPath = active_path;
  m_guiConfigDirty = false;
  m_windowTitleBase = "lilsim";
  if (!m_activeGuiConfigPath.empty()) {
    m_windowTitleBase += " - ";
    m_windowTitleBase += m_activeGuiConfigPath.filename().string();
  }
  ensureConfigPointerMatches(m_activeGuiConfigPath);
}

/**
 * @brief Apply dimensional, visibility, and path state from the loaded GUI
 * config.
 */
void Application::applyBasicGuiConfig() {
  m_width = m_guiConfig.window_width;
  m_height = m_guiConfig.window_height;
  m_left_panel.setContentWidth(m_guiConfig.leftPanelWidth);
  m_left_panel.setCollapsed(m_guiConfig.leftPanelCollapsed);
  m_right_panel.setContentWidth(m_guiConfig.rightPanelWidth);
  m_right_panel.setCollapsed(m_guiConfig.rightPanelCollapsed);
  m_show_car = m_guiConfig.showCar;
  m_show_cones = m_guiConfig.showCones;

  if (!m_guiConfig.trackDirectory.empty()) {
    std::filesystem::path dir_path = m_guiConfig.trackDirectory;
    if (!dir_path.is_absolute()) {
      dir_path = m_install_root / dir_path;
    }
    std::error_code ec;
    std::filesystem::path canonical =
        std::filesystem::weakly_canonical(dir_path, ec);
    std::string normalized = ec ? dir_path.string() : canonical.string();
    std::strncpy(m_track_dir_buffer, normalized.c_str(),
                 sizeof(m_track_dir_buffer) - 1);
    m_track_dir_buffer[sizeof(m_track_dir_buffer) - 1] = '\0';
    m_guiConfig.trackDirectory = normalized;
  }
  scanTrackDirectory();

  if (!m_guiConfig.paramProfilePath.empty()) {
    std::strncpy(m_param_file_buffer, m_guiConfig.paramProfilePath.c_str(),
                 sizeof(m_param_file_buffer) - 1);
    m_param_file_buffer[sizeof(m_param_file_buffer) - 1] = '\0';
  }

  if (!m_guiConfig.trackFile.empty()) {
    std::filesystem::path track_path = m_guiConfig.trackFile;
    if (!track_path.is_absolute()) {
      track_path = m_install_root / track_path;
    }
    std::error_code ec;
    std::filesystem::path canonical =
        std::filesystem::weakly_canonical(track_path, ec);
    if (!ec) {
      m_guiConfig.trackFile = canonical.string();
    } else {
      m_guiConfig.trackFile = track_path.string();
    }
  }

  m_marker_system.applyNamespaceVisibilitySnapshot(
      m_guiConfig.markerNamespaceVisibility);
  m_gui_control_source = m_guiConfig.guiInputSource;
  refreshWindowTitle();
}

/**
 * @brief Stage simulator timing parameters based on the loaded GUI config.
 */
void Application::applySimConfigFromGui() {
  if (m_guiConfig.timestepMs > 0.0) {
    m_simulator.requestDt(m_guiConfig.timestepMs / 1000.0);
  }
  if (m_guiConfig.controlPeriodMs > 0.0) {
    m_simulator.requestControlPeriodMs(m_guiConfig.controlPeriodMs);
  }
  if (m_guiConfig.controlDelayMs > 0.0) {
    m_simulator.requestControlDelayMs(m_guiConfig.controlDelayMs);
  }
}

/**
 * @brief Apply persisted input source and control mode selections via admin
 * command.
 */
void Application::applyControlModeFromConfig() {
  bool target_gui = m_guiConfig.guiInputSource;
  bool target_sync = m_guiConfig.syncControlMode;
  if (target_gui) {
    target_sync = false;
  }

  bool current_gui = !m_simulator.isExternalControlEnabled();
  bool current_sync = m_simulator.isSyncMode();
  if (current_gui == target_gui && current_sync == target_sync) {
    return;
  }

  if (!sendSetModeCommand(target_sync, !target_gui)) {
    spdlog::warn(
        "[viz] Failed to apply input source/control mode from GUI config.");
    return;
  }

  m_gui_control_source = target_gui;
  m_guiConfig.guiInputSource = target_gui;
  m_guiConfig.syncControlMode = target_sync;
}

/**
 * @brief Attempt to reload the model specified in the GUI config.
 */
void Application::restoreModelFromConfig() {
  if (m_guiConfig.modelPath.empty()) {
    return;
  }
  namespace fs = std::filesystem;
  fs::path model_path = fs::path(m_guiConfig.modelPath);
  if (!model_path.is_absolute()) {
    model_path = m_install_root / model_path;
  }
  std::error_code ec;
  if (!fs::exists(model_path, ec)) {
    spdlog::warn(
        "[viz] Configured model '{}' not found; keeping current model.",
        model_path.string());
    return;
  }
  if (m_simulator.loadModel(model_path.string())) {
    spdlog::info("[viz] Loaded model '{}' from GUI config.",
                 model_path.string());
    onModelChanged();
    // sync combo selection
    refreshAvailableModels();
    for (size_t i = 0; i < m_availableModels.size(); ++i) {
      if (m_availableModels[i].path == model_path.string()) {
        m_selected_model_index = i;
        break;
      }
    }
  }
}

/**
 * @brief Load the parameter profile defined in the GUI config via admin
 * command.
 */
void Application::applyProfileFromConfig() {
  if (m_guiConfig.paramProfilePath.empty()) {
    return;
  }
  std::error_code ec;
  if (!std::filesystem::exists(m_guiConfig.paramProfilePath, ec)) {
    spdlog::warn("[viz] GUI config profile '{}' not found; ignoring.",
                 m_guiConfig.paramProfilePath);
    return;
  }
  if (!sendProfileCommand(::lilsim::AdminCommandType::LOAD_PARAM_PROFILE,
                          m_guiConfig.paramProfilePath)) {
    spdlog::warn("[viz] Failed to load parameter profile '{}' from config.",
                 m_guiConfig.paramProfilePath);
  }
}

/**
 * @brief Apply a loaded GUI configuration to the running application state.
 */
bool Application::applyGuiConfig(const GuiConfig& cfg) {
  m_guiConfig = cfg;
  applyBasicGuiConfig();
  applySimConfigFromGui();
  applyProfileFromConfig();
  restoreTrackFromConfig();
  applyControlModeFromConfig();
  syncParamProfileBuffer();
  m_simulator.reset();
  return true;
}

/**
 * @brief Load the configured track file through the admin socket.
 */
void Application::restoreTrackFromConfig() {
  if (m_guiConfig.trackFile.empty()) {
    return;
  }
  std::error_code ec;
  if (!std::filesystem::exists(m_guiConfig.trackFile, ec)) {
    spdlog::warn("[viz] Configured track '{}' missing; skipping.",
                 m_guiConfig.trackFile);
    return;
  }
  if (sendSetTrackCommand(m_guiConfig.trackFile)) {
    namespace fs = std::filesystem;
    fs::path track = fs::path(m_guiConfig.trackFile);
    std::string stem = track.stem().string();
    for (size_t i = 0; i < m_availableTracks.size(); ++i) {
      if (m_availableTracks[i] == stem) {
        m_selected_track_index = i;
        break;
      }
    }
  }
}

/**
 * @brief Deserialize a YAML GUI config into a GuiConfig structure.
 */
bool Application::loadGuiConfigFromDisk(const std::filesystem::path& path,
                                        GuiConfig& outConfig) {
  namespace fs = std::filesystem;
  if (path.empty()) {
    return false;
  }
  std::error_code ec;
  if (!fs::exists(path, ec)) {
    return false;
  }
  try {
    YAML::Node root = YAML::LoadFile(path.string());
    GuiConfig cfg = outConfig;
    if (auto window = root["window"]) {
      cfg.window_width = window["width"].as<uint32_t>(cfg.window_width);
      cfg.window_height = window["height"].as<uint32_t>(cfg.window_height);
    }
    if (auto panels = root["panels"]) {
      if (auto left = panels["left"]) {
        cfg.leftPanelWidth = left["width"].as<float>(cfg.leftPanelWidth);
        cfg.leftPanelCollapsed =
            left["collapsed"].as<bool>(cfg.leftPanelCollapsed);
      }
      if (auto right = panels["right"]) {
        cfg.rightPanelWidth = right["width"].as<float>(cfg.rightPanelWidth);
        cfg.rightPanelCollapsed =
            right["collapsed"].as<bool>(cfg.rightPanelCollapsed);
      }
    }
    if (auto paths = root["paths"]) {
      cfg.modelPath = paths["model"].as<std::string>(cfg.modelPath);
      cfg.paramProfilePath =
          paths["param_profile"].as<std::string>(cfg.paramProfilePath);
      cfg.trackDirectory =
          paths["track_directory"].as<std::string>(cfg.trackDirectory);
      cfg.trackFile = paths["track_file"].as<std::string>(cfg.trackFile);
    }
    if (auto sim = root["simulation"]) {
      cfg.timestepMs = sim["timestep_ms"].as<double>(cfg.timestepMs);
      cfg.controlPeriodMs =
          sim["control_period_ms"].as<double>(cfg.controlPeriodMs);
      cfg.controlDelayMs =
          sim["control_delay_ms"].as<double>(cfg.controlDelayMs);
      if (auto source = sim["input_source"]) {
        std::string value = to_lower_copy(
            source.as<std::string>(cfg.guiInputSource ? "gui" : "client"));
        if (value == "gui" || value == "internal" || value == "ui") {
          cfg.guiInputSource = true;
        } else if (value == "client" || value == "external" || value == "zmq") {
          cfg.guiInputSource = false;
        }
      }
      if (auto mode = sim["control_mode"]) {
        std::string value = to_lower_copy(mode.as<std::string>(
            cfg.syncControlMode ? "synchronous" : "asynchronous"));
        if (value == "synchronous" || value == "sync") {
          cfg.syncControlMode = true;
        } else if (value == "asynchronous" || value == "async") {
          cfg.syncControlMode = false;
        }
      }
    }
    if (auto vis = root["visibility"]) {
      cfg.showCar = vis["show_car"].as<bool>(cfg.showCar);
      cfg.showCones = vis["show_cones"].as<bool>(cfg.showCones);
    }
    if (auto markers = root["markers"]) {
      if (auto namespaces = markers["namespaces"]) {
        cfg.markerNamespaceVisibility.clear();
        for (const auto& entry : namespaces) {
          std::string ns_name = entry.first.as<std::string>();
          bool visible = entry.second.as<bool>(true);
          cfg.markerNamespaceVisibility[ns_name] = visible;
        }
      }
    }
    outConfig = cfg;
    return true;
  } catch (const std::exception& ex) {
    spdlog::warn("[viz] Unable to parse GUI config '{}': {}", path.string(),
                 ex.what());
    return false;
  }
}

/**
 * @brief Serialize the in-memory GUI config to disk.
 */
bool Application::writeGuiConfigToDisk(const std::filesystem::path& path,
                                       const GuiConfig& cfg) {
  namespace fs = std::filesystem;
  try {
    fs::create_directories(path.parent_path());
    YAML::Node root;
    auto window = root["window"];
    window["width"] = cfg.window_width;
    window["height"] = cfg.window_height;
    auto panels = root["panels"];
    auto left = panels["left"];
    left["width"] = cfg.leftPanelWidth;
    left["collapsed"] = cfg.leftPanelCollapsed;
    auto right = panels["right"];
    right["width"] = cfg.rightPanelWidth;
    right["collapsed"] = cfg.rightPanelCollapsed;
    auto paths = root["paths"];
    paths["model"] = cfg.modelPath;
    paths["param_profile"] = cfg.paramProfilePath;
    paths["track_directory"] = cfg.trackDirectory;
    paths["track_file"] = cfg.trackFile;
    auto sim = root["simulation"];
    sim["timestep_ms"] = cfg.timestepMs;
    sim["control_period_ms"] = cfg.controlPeriodMs;
    sim["control_delay_ms"] = cfg.controlDelayMs;
    sim["input_source"] = cfg.guiInputSource ? "gui" : "client";
    sim["control_mode"] = cfg.syncControlMode ? "synchronous" : "asynchronous";
    auto vis = root["visibility"];
    vis["show_car"] = cfg.showCar;
    vis["show_cones"] = cfg.showCones;
    auto markers = root["markers"];
    auto namespaces = markers["namespaces"];
    namespaces = YAML::Node(YAML::NodeType::Map);
    for (const auto& [ns_name, visible] : cfg.markerNamespaceVisibility) {
      namespaces[ns_name] = visible;
    }
    markers["namespaces"] = namespaces;
    root["markers"] = markers;

    YAML::Emitter emitter;
    emitter << root;
    std::ofstream out(path);
    if (!out.is_open()) {
      return false;
    }
    out << emitter.c_str();
    return true;
  } catch (const std::exception& ex) {
    spdlog::error("[viz] Failed to write GUI config '{}': {}", path.string(),
                  ex.what());
    return false;
  }
}

bool Application::loadGuiConfigFromPath(const std::filesystem::path& path) {
  namespace fs = std::filesystem;
  if (path.empty()) {
    spdlog::warn("[viz] GUI config load failed: empty path.");
    return false;
  }
  GuiConfig loaded = m_guiConfig;
  if (!loadGuiConfigFromDisk(path, loaded)) {
    spdlog::warn("[viz] Failed to load GUI config '{}'.", path.string());
    return false;
  }
  if (!applyGuiConfig(loaded)) {
    spdlog::warn("[viz] Failed to apply GUI config '{}'.", path.string());
    return false;
  }
  std::error_code ec;
  fs::path canonical_path = fs::weakly_canonical(path, ec);
  fs::path resolved = ec ? fs::absolute(path) : canonical_path;
  m_activeGuiConfigPath = resolved;
  ensureConfigPointerMatches(m_activeGuiConfigPath);
  m_windowTitleBase = "lilsim";
  if (!m_activeGuiConfigPath.empty()) {
    m_windowTitleBase += " - ";
    m_windowTitleBase += m_activeGuiConfigPath.filename().string();
  }
  m_guiConfigDirty = false;
  refreshWindowTitle();
  spdlog::info("[viz] Loaded GUI config from '{}'.",
               m_activeGuiConfigPath.string());
  return true;
}

void Application::ensureDefaultGuiConfigExists() {
  namespace fs = std::filesystem;
  std::error_code ec;
  if (fs::exists(m_defaultGuiConfigPath, ec)) {
    return;
  }
  GuiConfig defaults{};
  if (!writeGuiConfigToDisk(m_defaultGuiConfigPath, defaults)) {
    spdlog::error("[viz] Failed to recreate default GUI config '{}'.",
                  m_defaultGuiConfigPath.string());
  } else {
    spdlog::info("[viz] Regenerated default GUI config '{}'.",
                 m_defaultGuiConfigPath.string());
  }
}

/**
 * @brief Determine the active config path by reading the pointer file if
 * present.
 */
std::filesystem::path Application::resolveActiveConfigPath() const {
  namespace fs = std::filesystem;
  if (m_configPointerPath.empty()) {
    return m_defaultGuiConfigPath;
  }
  std::ifstream pointer(m_configPointerPath);
  if (!pointer.is_open()) {
    return m_defaultGuiConfigPath;
  }
  std::string line;
  std::getline(pointer, line);
  fs::path candidate = line.empty() ? m_defaultGuiConfigPath : fs::path(line);
  if (!candidate.is_absolute()) {
    candidate = m_install_root / candidate;
  }
  return candidate;
}

/**
 * @brief Overwrite the pointer file so it references the provided path.
 */
void Application::ensureConfigPointerMatches(
    const std::filesystem::path& path) {
  if (m_configPointerPath.empty()) {
    return;
  }
  try {
    std::ofstream pointer(m_configPointerPath);
    if (pointer.is_open()) {
      std::error_code ec;
      auto canonical_path = std::filesystem::weakly_canonical(path, ec);
      const std::string& text = ec ? path.string() : canonical_path.string();
      pointer << text;
    }
  } catch (const std::exception& ex) {
    spdlog::warn("[viz] Failed to update GUI config pointer '{}': {}",
                 m_configPointerPath.string(), ex.what());
  }
}

/**
 * @brief Flag the config as dirty and update the window title indicator.
 */
void Application::markGuiConfigDirty() {
  if (!m_guiConfigDirty) {
    m_guiConfigDirty = true;
    refreshWindowTitle();
  }
}

/**
 * @brief Clear the dirty flag and refresh window title formatting.
 */
void Application::clearGuiConfigDirty() {
  if (m_guiConfigDirty) {
    m_guiConfigDirty = false;
    refreshWindowTitle();
  }
}

/**
 * @brief Ensure the GLFW window title reflects whether unsaved changes exist.
 */
void Application::refreshWindowTitle() {
  std::string title = m_windowTitleBase;
  if (m_guiConfigDirty) {
    title += "*";
  }
  if (title == m_windowTitleCached || !m_window) {
    m_windowTitleCached = title;
    if (m_window) {
      glfwSetWindowTitle(m_window, title.c_str());
    }
    return;
  }
  m_windowTitleCached = title;
  if (m_window) {
    glfwSetWindowTitle(m_window, title.c_str());
  }
}

/**
 * @brief Track panel widths/collapse state and mark the config dirty when they
 * change.
 */
void Application::capturePanelLayoutState() {
  bool changed = false;
  float left_width = m_left_panel.getContentWidth();
  if (std::abs(left_width - m_guiConfig.leftPanelWidth) > 0.5f) {
    m_guiConfig.leftPanelWidth = left_width;
    changed = true;
  }
  bool left_collapsed = m_left_panel.isCollapsed();
  if (left_collapsed != m_guiConfig.leftPanelCollapsed) {
    m_guiConfig.leftPanelCollapsed = left_collapsed;
    changed = true;
  }

  float right_width = m_right_panel.getContentWidth();
  if (std::abs(right_width - m_guiConfig.rightPanelWidth) > 0.5f) {
    m_guiConfig.rightPanelWidth = right_width;
    changed = true;
  }
  bool right_collapsed = m_right_panel.isCollapsed();
  if (right_collapsed != m_guiConfig.rightPanelCollapsed) {
    m_guiConfig.rightPanelCollapsed = right_collapsed;
    changed = true;
  }

  if (changed) {
    markGuiConfigDirty();
  }
}

/**
 * @brief Render the File menu with Save/Save As entries.
 * @return The height of the menu bar so other windows can offset below it.
 */
float Application::handleMenuBar() {
  float height = 0.0f;
  if (ImGui::BeginMainMenuBar()) {
    height = ImGui::GetWindowSize().y;
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Save current GUI config", "Ctrl+S")) {
        saveGuiConfig(false);
      }
      if (ImGui::MenuItem("Save GUI config as...", "Ctrl+Shift+S")) {
        saveGuiConfig(true);
      }
      if (ImGui::MenuItem("Load GUI config...", "Ctrl+O")) {
        openGuiConfigLoadDialog();
      }
      if (ImGui::MenuItem("Reset GUI config path")) {
        resetGuiConfigToDefault();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
  return height;
}

/**
 * @brief Detect keyboard shortcuts for saving the GUI config.
 */
void Application::handleConfigShortcuts() {
  ImGuiIO& io = ImGui::GetIO();
  if (io.WantTextInput) {
    return;
  }
  if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false)) {
    if (io.KeyShift) {
      saveGuiConfig(true);
    } else {
      saveGuiConfig(false);
    }
  }
  if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false)) {
    openGuiConfigLoadDialog();
  }
}

/**
 * @brief Pump any open ImGui file dialogs related to the GUI config.
 */
void Application::processGuiConfigDialogs() {
  if (ImGuiFileDialog::Instance()->Display(
          "GuiConfigSaveAs", ImGuiWindowFlags_None, kFileDialogMinSize,
          kFileDialogMaxSize)) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
      if (!path.empty()) {
        saveGuiConfigToPath(std::filesystem::path(path));
      }
    }
    ImGuiFileDialog::Instance()->Close();
  }
  if (ImGuiFileDialog::Instance()->Display(
          "GuiConfigLoad", ImGuiWindowFlags_None, kFileDialogMinSize,
          kFileDialogMaxSize)) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
      if (!path.empty()) {
        loadGuiConfigFromPath(std::filesystem::path(path));
      }
    }
    ImGuiFileDialog::Instance()->Close();
  }
}

/**
 * @brief Entry point used by UI/menu to save the GUI config.
 */
bool Application::saveGuiConfig(bool save_as_prompt) {
  if (save_as_prompt || m_activeGuiConfigPath.empty()) {
    openGuiConfigSaveDialog();
    return false;
  }
  return saveGuiConfigToPath(m_activeGuiConfigPath);
}

/**
 * @brief Write the config to a specific path and update pointer bookkeeping.
 */
bool Application::saveGuiConfigToPath(const std::filesystem::path& path) {
  if (path.empty()) {
    return false;
  }
  GuiConfig snapshot = m_guiConfig;
  snapshot.window_width = m_width;
  snapshot.window_height = m_height;
  m_guiConfig.window_width = m_width;
  m_guiConfig.window_height = m_height;
  bool gui_is_control_source = !m_simulator.isExternalControlEnabled();
  bool sync_mode = gui_is_control_source ? false : m_simulator.isSyncMode();
  snapshot.guiInputSource = gui_is_control_source;
  snapshot.syncControlMode = sync_mode;
  m_guiConfig.guiInputSource = gui_is_control_source;
  m_guiConfig.syncControlMode = sync_mode;
  auto namespace_snapshot = m_marker_system.snapshotNamespaceVisibility();
  snapshot.markerNamespaceVisibility = namespace_snapshot;
  m_guiConfig.markerNamespaceVisibility = namespace_snapshot;
  if (!writeGuiConfigToDisk(path, snapshot)) {
    return false;
  }
  m_activeGuiConfigPath = path;
  ensureConfigPointerMatches(path);
  m_windowTitleBase = "lilsim";
  if (!m_activeGuiConfigPath.empty()) {
    m_windowTitleBase += " - ";
    m_windowTitleBase += m_activeGuiConfigPath.filename().string();
  }
  clearGuiConfigDirty();
  spdlog::info("[viz] Saved GUI config to '{}'.", path.string());
  refreshWindowTitle();
  return true;
}

/**
 * @brief Launch an ImGui file dialog for choosing a save location.
 */
void Application::openGuiConfigSaveDialog() {
  std::filesystem::path start = m_activeGuiConfigPath.empty()
                                    ? m_defaultGuiConfigPath
                                    : m_activeGuiConfigPath;
  IGFD::FileDialogConfig config;
  config.path =
      start.parent_path().empty() ? "." : start.parent_path().string();
  config.filePathName = start.filename().string();
  config.countSelectionMax = 1;
  config.flags =
      ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_ConfirmOverwrite;
  ImGuiFileDialog::Instance()->OpenDialog("GuiConfigSaveAs", "Save GUI Config",
                                          ".yaml", config);
}

void Application::openGuiConfigLoadDialog() {
  std::filesystem::path start = m_activeGuiConfigPath.empty()
                                    ? m_defaultGuiConfigPath
                                    : m_activeGuiConfigPath;
  IGFD::FileDialogConfig config;
  config.path =
      start.parent_path().empty() ? "." : start.parent_path().string();
  config.filePathName = start.filename().string();
  config.countSelectionMax = 1;
  config.flags =
      ImGuiFileDialogFlags_Modal | ImGuiFileDialogFlags_DontShowHiddenFiles;
  ImGuiFileDialog::Instance()->OpenDialog("GuiConfigLoad", "Load GUI Config",
                                          ".yaml", config);
}

bool Application::resetGuiConfigToDefault() {
  ensureDefaultGuiConfigExists();
  if (!loadGuiConfigFromPath(m_defaultGuiConfigPath)) {
    spdlog::warn("[viz] Failed to reset GUI config to default '{}'.",
                 m_defaultGuiConfigPath.string());
    return false;
  }
  spdlog::info("[viz] GUI config reset to default '{}'.",
               m_defaultGuiConfigPath.string());
  return true;
}

/**
 * @brief Update the config's stored track directory from the current text
 * buffer.
 */
void Application::updateTrackDirectoryFromBuffer() {
  std::string buffer = m_track_dir_buffer;
  if (buffer.empty()) {
    return;
  }
  std::filesystem::path dir_path = std::filesystem::path(buffer);
  if (!dir_path.is_absolute()) {
    dir_path = m_install_root / dir_path;
  }
  std::error_code ec;
  std::filesystem::path canonical =
      std::filesystem::weakly_canonical(dir_path, ec);
  std::string normalized = ec ? dir_path.string() : canonical.string();
  if (normalized != m_guiConfig.trackDirectory) {
    m_guiConfig.trackDirectory = normalized;
    markGuiConfigDirty();
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
    m_param_file_buffer[0] = '\0';
    return;
  }
  std::strncpy(m_param_file_buffer, source.c_str(),
               sizeof(m_param_file_buffer) - 1);
  m_param_file_buffer[sizeof(m_param_file_buffer) - 1] = '\0';
}

void Application::scanTrackDirectory() {
  updateTrackDirectoryFromBuffer();
  m_availableTracks.clear();
  namespace fs = std::filesystem;
  try {
    if (fs::exists(m_track_dir_buffer) &&
        fs::is_directory(m_track_dir_buffer)) {
      for (const auto& entry : fs::directory_iterator(m_track_dir_buffer)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
          m_availableTracks.push_back(entry.path().stem().string());
        }
      }
      std::sort(m_availableTracks.begin(), m_availableTracks.end());
    }
  } catch (...) {
  }
}

void Application::mainLoop() {
  double current_time = glfwGetTime();
  double delta_time = current_time - m_last_frame_time;
  if (delta_time < m_target_frame_time) {
    std::this_thread::sleep_for(
        std::chrono::duration<double>(m_target_frame_time - delta_time));
    current_time = glfwGetTime();
  }
  m_last_frame_time = current_time;

  glfwPollEvents();
  handleInput();

  WGPUSurfaceTexture m_surface_texture = {};
  wgpuSurfaceGetCurrentTexture(m_surface, &m_surface_texture);

  if (m_surface_texture.status !=
          WGPUSurfaceGetCurrentTextureStatus_SuccessOptimal &&
      m_surface_texture.status !=
          WGPUSurfaceGetCurrentTextureStatus_SuccessSuboptimal) {
    if (m_surface_texture.texture)
      wgpuTextureRelease(m_surface_texture.texture);
    return;
  }

  ImGui_ImplWGPU_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize =
      ImVec2(static_cast<float>(m_width), static_cast<float>(m_height));
  ImGui::NewFrame();

  render2D();

  ImGui::Render();

  WGPUTextureViewDescriptor view_desc = {};
  view_desc.format = m_surfaceFormat;
  view_desc.dimension = WGPUTextureViewDimension_2D;
  view_desc.baseMipLevel = 0;
  view_desc.mipLevelCount = 1;
  view_desc.baseArrayLayer = 0;
  view_desc.arrayLayerCount = 1;
  view_desc.aspect = WGPUTextureAspect_All;
  WGPUTextureView backbuffer =
      wgpuTextureCreateView(m_surface_texture.texture, &view_desc);

  WGPURenderPassColorAttachment color_attachment = {};
  color_attachment.view = backbuffer;
  color_attachment.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  color_attachment.loadOp = WGPULoadOp_Clear;
  color_attachment.storeOp = WGPUStoreOp_Store;
  color_attachment.clearValue = {m_clearColor[0], m_clearColor[1],
                                 m_clearColor[2], m_clearColor[3]};

  WGPURenderPassDescriptor render_pass_desc = {};
  render_pass_desc.colorAttachmentCount = 1;
  render_pass_desc.colorAttachments = &color_attachment;

  WGPUCommandEncoderDescriptor encoder_desc = {};
  WGPUCommandEncoder encoder =
      wgpuDeviceCreateCommandEncoder(m_device, &encoder_desc);
  WGPURenderPassEncoder pass =
      wgpuCommandEncoderBeginRenderPass(encoder, &render_pass_desc);
  ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass);
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  WGPUCommandBufferDescriptor cmd_buffer_desc = {};
  WGPUCommandBuffer cmd_buffer =
      wgpuCommandEncoderFinish(encoder, &cmd_buffer_desc);
  wgpuCommandEncoderRelease(encoder);
  wgpuQueueSubmit(m_queue, 1, &cmd_buffer);
  wgpuCommandBufferRelease(cmd_buffer);
  wgpuSurfacePresent(m_surface);
  wgpuTextureViewRelease(backbuffer);
  wgpuTextureRelease(m_surface_texture.texture);
}

}  // namespace viz
