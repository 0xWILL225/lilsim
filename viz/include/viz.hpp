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
#include <functional>
#include <memory>
#include <filesystem>
#include <map>

#include "CarDefaults.hpp"
#include "Constants.hpp"
#include "MarkerSystem.hpp"
#include "panels/SidePanel.hpp"
#include "panels/ViewportPanel.hpp"
#include "scene.hpp"
#include "simulator.hpp"
#include "comm/CommServer.hpp"
#include "comm/endpoints.hpp"
#include "messages.pb.h"

#include <zmq.hpp>

namespace viz {

class Application {
public:
  Application(scene::SceneDB& db, sim::Simulator& sim, const std::filesystem::path& installRoot);
  ~Application(); 

  bool initialize();
  void terminate();
  void mainLoop();
  bool isRunning();
  void onResize(uint32_t new_width, uint32_t new_height);

  // Public for callback access
  ViewportPanel m_viewport_panel;
  SidePanel m_right_panel;
  SidePanel m_left_panel;
  MarkerSystem m_marker_system;
  
  bool m_show_car;
  bool m_show_cones;

  // Communication
  // std::unique_ptr<comm::MarkerSubscriber> m_marker_sub;
  std::chrono::steady_clock::time_point m_last_connection_probe;

private:
  /** @brief GUI state persistent between runs. */
  struct GuiConfig {
    uint32_t window_width = 1400u;
    uint32_t window_height = 800u;
    float leftPanelWidth = 300.0f;
    bool leftPanelCollapsed = false;
    float rightPanelWidth = 300.0f;
    bool rightPanelCollapsed = false;
    std::string modelPath;
    std::string paramProfilePath;
    std::string trackDirectory;
    std::string trackFile;
    double timestepMs = 1.0;
    double controlPeriodMs = 10.0;
    double controlDelayMs = 2.0;
    bool guiInputSource = true; // true: GUI is the control source, false: ZeroMQ Client is the control source
    bool syncControlMode = false; // true: synchronous control mode, false: asynchronous control mode
    bool showCar = true;
    bool showCones = true;
    std::map<std::string, bool> markerNamespaceVisibility;
  };

  scene::SceneDB& m_sceneDB;
  sim::Simulator& m_simulator;

  GLFWwindow* m_window = nullptr;
  uint32_t m_width;
  uint32_t m_height;

  WGPUInstance m_instance = nullptr;
  WGPUSurface m_surface = nullptr;
  WGPUAdapter m_adapter = nullptr;
  WGPUDevice m_device = nullptr;
  WGPUQueue m_queue = nullptr;
  WGPUTextureFormat m_surfaceFormat = WGPUTextureFormat_BGRA8Unorm;

  float m_clearColor[4] = {0.45f, 0.55f, 0.60f, 1.00f};

  double m_target_frame_time = 1.0 / 60.0;
  double m_last_frame_time = 0.0;

  std::vector<double> m_ui_param_values;
  std::vector<uint32_t> m_ui_setting_values;
  
  // Cache indices (common::kNullIndex means not found/invalid)
  size_t m_input_idx_wheel_angle = common::kNullIndex;
  size_t m_input_idx_wheel_rate = common::kNullIndex;
  size_t m_input_idx_ax = common::kNullIndex;
  
  size_t m_state_idx_x = common::kNullIndex;
  size_t m_state_idx_y = common::kNullIndex;
  size_t m_state_idx_yaw = common::kNullIndex;
  size_t m_state_idx_v = common::kNullIndex;
  size_t m_state_idx_ax = common::kNullIndex;
  size_t m_state_idx_steer_wheel_angle = common::kNullIndex;
  size_t m_state_idx_steer_wheel_rate = common::kNullIndex;
  size_t m_state_idx_wheel_fl = common::kNullIndex;
  size_t m_state_idx_wheel_fr = common::kNullIndex;
  
  size_t m_param_idx_wheelbase = common::kNullIndex;
  size_t m_param_idx_track_width = common::kNullIndex;
  size_t m_setting_idx_steering_mode = common::kNullIndex;
  
  int m_stepN = 10;

  char m_track_dir_buffer[512] = "";
  char m_param_file_buffer[512] = "";
  std::vector<std::string> m_availableTracks;
  size_t m_selected_track_index = common::kNullIndex;
  
  std::vector<sim::Simulator::ModelInfo> m_availableModels;
  size_t m_selected_model_index = common::kNullIndex;

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
  bool initZmqInterface();
  bool sendAdminCommand(::lilsim::AdminCommandType type,
                        const std::function<void(::lilsim::AdminCommand&)>& builder = {},
                        ::lilsim::AdminReply* out = nullptr);
  void pollMetadataUpdates();
  void handleMetadataMessage(const ::lilsim::ModelMetadata& msg);
  bool requestMetadataSnapshot();
  bool stageParamUpdate(size_t index, double value);
  bool stageSettingUpdate(size_t index, uint32_t value);
  bool sendSetTrackCommand(const std::string& path);
  bool sendProfileCommand(::lilsim::AdminCommandType type, const std::string& path = {});
  bool sendSetModeCommand(bool sync, bool externalControl);
  void sendGuiControlInputs(const std::vector<double>& inputs);
  void pollMarkerMessages();
  void initializeGuiConfigSystem(const std::filesystem::path& installRoot);
  void applyBasicGuiConfig();
  void applySimConfigFromGui();
  void applyControlModeFromConfig();
  void restoreModelFromConfig();
  void restoreTrackFromConfig();
  void applyProfileFromConfig();
  bool applyGuiConfig(const GuiConfig& cfg);
  bool loadGuiConfigFromPath(const std::filesystem::path& path);
  bool loadGuiConfigFromDisk(const std::filesystem::path& path, GuiConfig& outConfig);
  bool writeGuiConfigToDisk(const std::filesystem::path& path, const GuiConfig& cfg);
  std::filesystem::path resolveActiveConfigPath() const;
  void ensureConfigPointerMatches(const std::filesystem::path& path);
  void ensureDefaultGuiConfigExists();
  void markGuiConfigDirty();
  void clearGuiConfigDirty();
  void refreshWindowTitle();
  void capturePanelLayoutState();
  float handleMenuBar();
  void handleConfigShortcuts();
  void processGuiConfigDialogs();
  bool saveGuiConfig(bool saveAsPrompt);
  bool saveGuiConfigToPath(const std::filesystem::path& path);
  void openGuiConfigSaveDialog();
  void openGuiConfigLoadDialog();
  bool resetGuiConfigToDefault();
  void updateTrackDirectoryFromBuffer();

  std::shared_ptr<zmq::context_t> m_zmq_context;
  std::unique_ptr<zmq::socket_t> m_admin_socket;
  std::unique_ptr<zmq::socket_t> m_control_pub;
  std::unique_ptr<zmq::socket_t> m_metadata_sub;
  bool m_gui_control_source{true};
  uint64_t m_metadata_version{0};
  ::lilsim::ModelMetadata m_last_metadata;
  std::unique_ptr<comm::MarkerSubscriber> m_marker_sub;

  GuiConfig m_guiConfig;
  std::filesystem::path m_install_root;
  std::filesystem::path m_defaultGuiConfigPath;
  std::filesystem::path m_activeGuiConfigPath;
  std::filesystem::path m_configPointerPath;
  bool m_guiConfigDirty{false};
  std::string m_windowTitleBase{"lilsim"};
  std::string m_windowTitleCached{"lilsim"};
};

} // namespace viz
