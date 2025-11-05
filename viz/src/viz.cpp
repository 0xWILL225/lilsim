#include "viz.hpp"
#include "KeyBindings.hpp"
#include "TrackLoader.hpp"

#include <GLFW/glfw3native.h>
#include <ImGuiFileDialog.h>
#include <cstdio>
#include <filesystem>
#include <algorithm>

// Undefine X11 macros that conflict with other headers
#ifdef Success
#undef Success
#endif
#ifdef None
#undef None
#endif

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_wgpu.h>

// Helper macro for wgpu-native string views
#define WGPU_STR(s)                                                            \
  WGPUStringView {                                                             \
    s, WGPU_STRLEN                                                             \
  }

namespace viz {

Application::Application(scene::SceneDB& db, sim::Simulator& sim)
  : m_sceneDB(db)
  , m_simulator(sim)
  , m_rightPanel("Admin Panel", SidePanel::Side::Right, 300.0f)
  , m_viewportPanel(db) {
  // Set default track directory to executable path + /tracks
  std::filesystem::path exePath = std::filesystem::current_path();
  std::string defaultTrackPath = (exePath / "tracks").string();
  strncpy(m_trackDirBuffer, defaultTrackPath.c_str(), sizeof(m_trackDirBuffer) - 1);
  m_trackDirBuffer[sizeof(m_trackDirBuffer) - 1] = '\0';
  
  setupPanels();
  
  // Scan the default directory for tracks
  scanTrackDirectory();
}

/**
 * @brief GLFW callback for framebuffer resize events.
 *
 * This is a C-style callback required by GLFW. It retrieves the Application
 * instance from the window user pointer and forwards the resize event.
 *
 * @param window The GLFW window that was resized
 * @param width New framebuffer width in pixels
 * @param height New framebuffer height in pixels
 */
static void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (app) {
    app->onResize(width, height);
  }
}

/**
 * @brief GLFW callback for scroll events (used for zoom).
 *
 * @param window The GLFW window
 * @param xoffset Horizontal scroll offset
 * @param yoffset Vertical scroll offset (used for zoom)
 */
static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  (void)xoffset;
  auto* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
  if (app) {
    // Only zoom if hovering over the viewport
    if (!app->m_viewportPanel.isHovered()) {
      return;
    }

    // Zoom in/out with scroll wheel (10% per scroll tick)
    float zoomDelta = (yoffset > 0) ? 1.1f : 0.9f;

    if (app->m_viewportPanel.cameraMode == ViewportPanel::CameraMode::Free) {
      app->m_viewportPanel.freeCameraZoom *= zoomDelta;
      app->m_viewportPanel.freeCameraZoom =
        std::clamp(app->m_viewportPanel.freeCameraZoom, 5.0f, 500.0f);
    } else {
      app->m_viewportPanel.followCarZoom *= zoomDelta;
      app->m_viewportPanel.followCarZoom =
        std::clamp(app->m_viewportPanel.followCarZoom, 5.0f, 500.0f);
    }
  }
}

/**
 * @brief Callback invoked when WebGPU m_adapter request completes.
 *
 * This callback is called asynchronously by wgpu-native when the m_adapter
 * request initiated by wgpuInstanceRequestAdapter() finishes.
 *
 * @param status The status of the m_adapter request
 * @param m_adapter The requested m_adapter handle (if successful)
 * @param message Optional error message
 * @param userdata1 Pointer to store the m_adapter handle
 * @param userdata2 Reserved for future use
 */
static void onAdapterRequestEnded(WGPURequestAdapterStatus status,
                                  WGPUAdapter m_adapter, WGPUStringView message,
                                  void* userdata1, void* userdata2) {
  (void)userdata2;
  (void)message;
  if (status == WGPURequestAdapterStatus_Success) {
    *(WGPUAdapter*)userdata1 = m_adapter;
  } else {
    fprintf(stderr, "Failed to get WebGPU m_adapter\n");
  }
}

/**
 * @brief Callback invoked when WebGPU m_device request completes.
 *
 * This callback is called asynchronously by wgpu-native when the m_device
 * request initiated by wgpuAdapterRequestDevice() finishes.
 *
 * @param status The status of the m_device request
 * @param m_device The requested m_device handle (if successful)
 * @param message Optional error message
 * @param userdata1 Pointer to store the m_device handle
 * @param userdata2 Reserved for future use
 */
static void onDeviceRequestEnded(WGPURequestDeviceStatus status,
                                 WGPUDevice m_device, WGPUStringView message,
                                 void* userdata1, void* userdata2) {
  (void)userdata2;
  (void)message;
  if (status == WGPURequestDeviceStatus_Success) {
    *(WGPUDevice*)userdata1 = m_device;
  } else {
    fprintf(stderr, "Failed to get WebGPU m_device\n");
  }
}

/**
 * @brief Initializes the GLFW library and creates the application window.
 *
 * This method:
 * - Initializes GLFW
 * - Configures GLFW to not create a default graphics context (we use WebGPU)
 * - Creates a window with the configured dimensions
 *
 * @return true if initialization succeeds, false otherwise
 */
bool Application::initGLFW() {
  fprintf(stderr, "[DEBUG] Initializing GLFW...\n");
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return false;
  }

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  fprintf(stderr, "[DEBUG] Creating window...\n");
  m_window = glfwCreateWindow(m_width, m_height, "lilsim", nullptr, nullptr);

  if (!m_window) {
    fprintf(stderr, "Failed to create window\n");
    return false;
  }

  // Set up window user pointer and callbacks
  glfwSetWindowUserPointer(m_window, this);
  glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);
  glfwSetScrollCallback(m_window, scrollCallback);

  fprintf(stderr, "[DEBUG] GLFW initialized successfully\n");
  return true;
}

/**
 * @brief Initializes WebGPU instance, m_surface, m_adapter, m_device, and
 * m_queue.
 *
 * This method performs the complete WebGPU initialization sequence:
 * 1. Creates a WebGPU instance
 * 2. Creates a platform-specific m_surface (X11/Metal/Win32)
 * 3. Requests and waits for a compatible m_adapter
 * 4. Requests and waits for a logical m_device
 * 5. Obtains the command m_queue
 * 6. Configures the m_surface with appropriate format and present mode
 *
 * The m_surface creation is platform-specific:
 * - Linux: Uses X11 display and window handles
 * - macOS: Creates a Metal layer and attaches it to the Cocoa window
 * - Windows: Uses Win32 HWND and HINSTANCE
 *
 * @return true if all initialization steps succeed, false otherwise
 */
bool Application::initWebGPU() {
  // Create instance
  fprintf(stderr, "[DEBUG] Creating WebGPU instance...\n");
  WGPUInstanceDescriptor instanceDesc = {};
  instanceDesc.nextInChain = nullptr;
  m_instance = wgpuCreateInstance(&instanceDesc);
  if (!m_instance) {
    fprintf(stderr, "Failed to create WebGPU instance\n");
    return false;
  }
  fprintf(stderr, "[DEBUG] WebGPU instance created\n");

  // Create m_surface (platform-specific)
#if defined(__linux__)
  // Linux X11
  WGPUSurfaceSourceXlibWindow fromXlibWindow = {};
  fromXlibWindow.chain.sType = WGPUSType_SurfaceSourceXlibWindow;
  fromXlibWindow.display = glfwGetX11Display();
  fromXlibWindow.window = glfwGetX11Window(m_window);

  WGPUSurfaceDescriptor surfaceDesc = {};
  surfaceDesc.nextInChain =
    reinterpret_cast<WGPUChainedStruct const*>(&fromXlibWindow);
  m_surface = wgpuInstanceCreateSurface(m_instance, &surfaceDesc);

#elif defined(__APPLE__)
  // macOS Metal
  id metal_layer = nullptr;
  NSWindow* ns_window = glfwGetCocoaWindow(m_window);
  [ns_window.contentView setWantsLayer:YES];
  metal_layer = [CAMetalLayer layer];
  [ns_window.contentView setLayer:metal_layer];

  WGPUSurfaceSourceMetalLayer fromMetalLayer = {};
  fromMetalLayer.chain.sType = WGPUSType_SurfaceSourceMetalLayer;
  fromMetalLayer.layer = metal_layer;

  WGPUSurfaceDescriptor surfaceDesc = {};
  surfaceDesc.nextInChain =
    reinterpret_cast<WGPUChainedStruct const*>(&fromMetalLayer);
  m_surface = wgpuInstanceCreateSurface(m_instance, &surfaceDesc);

#elif defined(_WIN32)
  // Windows
  HWND hwnd = glfwGetWin32Window(m_window);
  HINSTANCE hinstance = GetModuleHandle(nullptr);

  WGPUSurfaceSourceWindowsHWND fromWindowsHWND = {};
  fromWindowsHWND.chain.sType = WGPUSType_SurfaceSourceWindowsHWND;
  fromWindowsHWND.hinstance = hinstance;
  fromWindowsHWND.hwnd = hwnd;

  WGPUSurfaceDescriptor surfaceDesc = {};
  surfaceDesc.nextInChain =
    reinterpret_cast<WGPUChainedStruct const*>(&fromWindowsHWND);
  m_surface = wgpuInstanceCreateSurface(m_instance, &surfaceDesc);

#else
#error "Unsupported platform - only Linux, macOS, and Windows are supported"
#endif

  if (!m_surface) {
    fprintf(stderr, "Failed to create m_surface\n");
    return false;
  }
  fprintf(stderr, "[DEBUG] Surface created\n");

  // Request m_adapter
  fprintf(stderr, "[DEBUG] Requesting m_adapter...\n");
  WGPURequestAdapterOptions m_adapterOpts = {};
  m_adapterOpts.nextInChain = nullptr;
  m_adapterOpts.compatibleSurface = m_surface;
  m_adapterOpts.powerPreference = WGPUPowerPreference_HighPerformance;

  WGPURequestAdapterCallbackInfo m_adapterCallbackInfo = {};
  m_adapterCallbackInfo.mode = WGPUCallbackMode_AllowSpontaneous;
  m_adapterCallbackInfo.callback = onAdapterRequestEnded;
  m_adapterCallbackInfo.userdata1 = &m_adapter;

  wgpuInstanceRequestAdapter(m_instance, &m_adapterOpts, m_adapterCallbackInfo);

  // Wait for m_adapter
  int waitCount = 0;
  while (!m_adapter) {
    if (waitCount % 100 == 0) {
      fprintf(stderr, "[DEBUG] Waiting for m_adapter... (%d iterations)\n",
              waitCount);
    }
    wgpuInstanceProcessEvents(m_instance);
    glfwPollEvents(); // Keep window responsive during initialization
    waitCount++;
  }
  fprintf(stderr, "[DEBUG] Adapter acquired\n");

  // Create m_device
  fprintf(stderr, "[DEBUG] Requesting m_device...\n");
  WGPUDeviceDescriptor m_deviceDesc = {};
  m_deviceDesc.nextInChain = nullptr;
  m_deviceDesc.label = WGPU_STR("Main Device");

  WGPURequestDeviceCallbackInfo callbackInfo = {};
  callbackInfo.mode = WGPUCallbackMode_AllowSpontaneous;
  callbackInfo.callback = onDeviceRequestEnded;
  callbackInfo.userdata1 = &m_device;

  wgpuAdapterRequestDevice(m_adapter, &m_deviceDesc, callbackInfo);

  // Wait for m_device
  waitCount = 0;
  while (!m_device) {
    if (waitCount % 100 == 0) {
      fprintf(stderr, "[DEBUG] Waiting for m_device... (%d iterations)\n",
              waitCount);
    }
    wgpuInstanceProcessEvents(m_instance);
    glfwPollEvents(); // Keep window responsive during initialization
    waitCount++;
  }
  fprintf(stderr, "[DEBUG] Device acquired\n");

  m_queue = wgpuDeviceGetQueue(m_device);
  fprintf(stderr, "[DEBUG] Queue obtained\n");

  // Configure m_surface
  WGPUSurfaceConfiguration config = {};
  config.device = m_device;
  config.format = m_surfaceFormat;
  config.usage = WGPUTextureUsage_RenderAttachment;
  config.width = static_cast<uint32_t>(m_width);
  config.height = static_cast<uint32_t>(m_height);
  config.presentMode = WGPUPresentMode_Fifo;
  config.alphaMode = WGPUCompositeAlphaMode_Auto;

  wgpuSurfaceConfigure(m_surface, &config);
  fprintf(stderr, "[DEBUG] Surface configured\n");
  fprintf(stderr, "[DEBUG] WebGPU initialization complete\n");

  return true;
}

/**
 * @brief Initializes Dear ImGui with GLFW and WebGPU backends.
 *
 * This method:
 * - Creates the ImGui context
 * - Configures ImGui I/O settings (keyboard and gamepad navigation)
 * - Sets the dark color theme
 * - Initializes the GLFW platform backend
 * - Initializes the WebGPU renderer backend with appropriate m_surface format
 *
 * @return true if initialization succeeds, false otherwise
 */
bool Application::initImGui() {
  fprintf(stderr, "[DEBUG] Initializing ImGui...\n");
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOther(m_window, true);

  ImGui_ImplWGPU_InitInfo init_info = {};
  init_info.Device = m_device;
  init_info.NumFramesInFlight = 3;
  init_info.RenderTargetFormat = m_surfaceFormat;
  init_info.DepthStencilFormat = WGPUTextureFormat_Undefined;

  ImGui_ImplWGPU_Init(&init_info);
  fprintf(stderr, "[DEBUG] ImGui initialized\n");

  return true;
}

/**
 * @brief Initializes all application subsystems in order.
 *
 * Calls initialization methods in the required sequence:
 * 1. GLFW (windowing)
 * 2. WebGPU (graphics m_device)
 * 3. ImGui (user interface)
 *
 * If any initialization step fails, the method returns immediately.
 *
 * @return true if all subsystems initialize successfully, false otherwise
 */
bool Application::initialize() {
  if (!initGLFW())
    return false;
  if (!initWebGPU())
    return false;
  if (!initImGui())
    return false;
  return true;
}

/**
 * @brief Executes one iteration of the main rendering loop.
 *
 * This method performs a complete frame rendering cycle:
 * 1. Polls GLFW events (keyboard, mouse, window events)
 * 2. Starts a new ImGui frame
 * 3. Renders ImGui UI elements (currently just the demo window)
 * 4. Acquires the current m_surface texture for rendering
 * 5. Creates a render pass with clear color
 * 6. Records ImGui draw commands into the render pass
 * 7. Submits the command buffer to the GPU m_queue
 * 8. Presents the rendered frame to the screen
 * 9. Releases temporary GPU resources
 *
 * This method should be called repeatedly in the main application loop.
 */
void Application::mainLoop() {
  glfwPollEvents();
  handleInput();

  // Start ImGui frame
  ImGui_ImplWGPU_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // Render 2D scene
  render2D();

  // Render ImGui
  ImGui::Render();

  // Get current texture from m_surface
  WGPUSurfaceTexture m_surfaceTexture = {};
  wgpuSurfaceGetCurrentTexture(m_surface, &m_surfaceTexture);

  // Handle special m_surface states
  if (m_surfaceTexture.status == WGPUSurfaceGetCurrentTextureStatus_Timeout
      || m_surfaceTexture.status == WGPUSurfaceGetCurrentTextureStatus_Outdated
      || m_surfaceTexture.status == WGPUSurfaceGetCurrentTextureStatus_Lost) {
    // Surface needs reconfiguration (probably due to resize)
    if (m_surfaceTexture.texture) {
      wgpuTextureRelease(m_surfaceTexture.texture);
    }

    // Get current framebuffer size and reconfigure
    int fbWidth, fbHeight;
    glfwGetFramebufferSize(m_window, &fbWidth, &fbHeight);
    if (fbWidth > 0 && fbHeight > 0) {
      onResize(fbWidth, fbHeight);
    }
    return; // Skip this frame
  }

  if (m_surfaceTexture.status
        != WGPUSurfaceGetCurrentTextureStatus_SuccessOptimal
      && m_surfaceTexture.status
           != WGPUSurfaceGetCurrentTextureStatus_SuccessSuboptimal) {
    fprintf(stderr, "Failed to get current m_surface texture: status %d\n",
            m_surfaceTexture.status);
    return;
  }

  // Create texture view
  WGPUTextureViewDescriptor viewDesc = {};
  viewDesc.format = m_surfaceFormat;
  viewDesc.dimension = WGPUTextureViewDimension_2D;
  viewDesc.baseMipLevel = 0;
  viewDesc.mipLevelCount = 1;
  viewDesc.baseArrayLayer = 0;
  viewDesc.arrayLayerCount = 1;
  viewDesc.aspect = WGPUTextureAspect_All;

  WGPUTextureView backbuffer =
    wgpuTextureCreateView(m_surfaceTexture.texture, &viewDesc);

  // Create render pass
  WGPURenderPassColorAttachment colorAttachment = {};
  colorAttachment.view = backbuffer;
  colorAttachment.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  colorAttachment.loadOp = WGPULoadOp_Clear;
  colorAttachment.storeOp = WGPUStoreOp_Store;
  colorAttachment.clearValue = {
    m_clearColor[0] * m_clearColor[3], m_clearColor[1] * m_clearColor[3],
    m_clearColor[2] * m_clearColor[3], m_clearColor[3]};

  WGPURenderPassDescriptor renderPassDesc = {};
  renderPassDesc.colorAttachmentCount = 1;
  renderPassDesc.colorAttachments = &colorAttachment;
  renderPassDesc.depthStencilAttachment = nullptr;

  // Create command encoder
  WGPUCommandEncoderDescriptor encoderDesc = {};
  encoderDesc.label = WGPU_STR("Main Encoder");
  WGPUCommandEncoder encoder =
    wgpuDeviceCreateCommandEncoder(m_device, &encoderDesc);

  // Begin render pass
  WGPURenderPassEncoder pass =
    wgpuCommandEncoderBeginRenderPass(encoder, &renderPassDesc);

  // Render ImGui draw data
  ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass);

  // End render pass
  wgpuRenderPassEncoderEnd(pass);
  wgpuRenderPassEncoderRelease(pass);

  // Submit commands
  WGPUCommandBufferDescriptor cmdBufferDesc = {};
  cmdBufferDesc.label = WGPU_STR("Main Command Buffer");
  WGPUCommandBuffer cmdBuffer =
    wgpuCommandEncoderFinish(encoder, &cmdBufferDesc);
  wgpuCommandEncoderRelease(encoder);

  wgpuQueueSubmit(m_queue, 1, &cmdBuffer);
  wgpuCommandBufferRelease(cmdBuffer);

  // Present
  wgpuSurfacePresent(m_surface);

  // Release resources
  wgpuTextureViewRelease(backbuffer);
  wgpuTextureRelease(m_surfaceTexture.texture);
}

/**
 * @brief Checks if the application should continue running.
 *
 * @return true if the window exists and hasn't been closed by the user, false
 * otherwise
 */
bool Application::isRunning() {
  return m_window && !glfwWindowShouldClose(m_window);
}

/**
 * @brief Shuts down ImGui and its backends.
 *
 * Cleans up ImGui resources in the correct order:
 * 1. WebGPU backend
 * 2. GLFW backend
 * 3. ImGui context
 */
void Application::terminateImGui() {
  ImGui_ImplWGPU_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

/**
 * @brief Releases all WebGPU resources.
 *
 * Releases WebGPU handles in reverse order of creation:
 * 1. Queue
 * 2. Device
 * 3. Adapter
 * 4. Surface
 * 5. Instance
 *
 * Each resource is checked for validity before release to allow safe
 * partial cleanup if initialization failed midway.
 */
void Application::terminateWebGPU() {
  if (m_queue)
    wgpuQueueRelease(m_queue);
  if (m_device)
    wgpuDeviceRelease(m_device);
  if (m_adapter)
    wgpuAdapterRelease(m_adapter);
  if (m_surface)
    wgpuSurfaceRelease(m_surface);
  if (m_instance)
    wgpuInstanceRelease(m_instance);
}

/**
 * @brief Destroys the GLFW window and terminates GLFW.
 *
 * Cleans up GLFW resources:
 * 1. Destroys the window if it exists
 * 2. Terminates the GLFW library
 */
void Application::terminateGLFW() {
  if (m_window) {
    glfwDestroyWindow(m_window);
    m_window = nullptr;
  }
  glfwTerminate();
}

/**
 * @brief Terminates all application subsystems in reverse order.
 *
 * Calls termination methods in reverse initialization order:
 * 1. ImGui (user interface)
 * 2. WebGPU (graphics m_device)
 * 3. GLFW (windowing)
 *
 * This ensures proper cleanup of all resources.
 */
void Application::terminate() {
  terminateImGui();
  terminateWebGPU();
  terminateGLFW();
}

/**
 * @brief Handles window resize events by reconfiguring the WebGPU m_surface.
 *
 * This method is called when the window framebuffer size changes. It updates
 * the internal width/height and reconfigures the WebGPU m_surface to match
 * the new dimensions.
 *
 * @param newWidth New framebuffer width in pixels
 * @param newHeight New framebuffer height in pixels
 */
void Application::onResize(int newWidth, int newHeight) {
  if (newWidth <= 0 || newHeight <= 0) {
    // Ignore invalid sizes (minimized window)
    return;
  }

  fprintf(stderr, "[DEBUG] Window resized to %dx%d\n", newWidth, newHeight);

  m_width = newWidth;
  m_height = newHeight;

  // Reconfigure the m_surface with new dimensions
  if (m_surface && m_device) {
    WGPUSurfaceConfiguration config = {};
    config.device = m_device;
    config.format = m_surfaceFormat;
    config.usage = WGPUTextureUsage_RenderAttachment;
    config.width = static_cast<uint32_t>(m_width);
    config.height = static_cast<uint32_t>(m_height);
    config.presentMode = WGPUPresentMode_Fifo;
    config.alphaMode = WGPUCompositeAlphaMode_Auto;

    wgpuSurfaceConfigure(m_surface, &config);
    fprintf(stderr, "[DEBUG] Surface reconfigured\n");
  }
}

/**
 * @brief Handles keyboard and mouse input.
 *
 * Keyboard:
 * - WASD: Car control
 * - Spacebar: Toggle camera mode
 *
 * Mouse:
 * - Left drag: Pan free camera
 * - Scroll: Zoom (handled in callback)
 */
void Application::handleInput() {
  // Handle viewport panel input (camera controls)
  m_viewportPanel.handleInput(m_window);

  // Pause/Resume simulation
  static bool pauseKeyWasPressed = false;
  bool pauseKeyPressed = glfwGetKey(m_window, gKeyBindings.pauseSimulation) == GLFW_PRESS;
  if (pauseKeyPressed && !pauseKeyWasPressed) {
    if (m_simulator.isPaused()) {
      m_simulator.resume();
    } else {
      m_simulator.pause();
    }
  }
  pauseKeyWasPressed = pauseKeyPressed;

  // Reset simulation
  static bool resetKeyWasPressed = false;
  bool resetKeyPressed = glfwGetKey(m_window, gKeyBindings.resetSimulation) == GLFW_PRESS;
  if (resetKeyPressed && !resetKeyWasPressed) {
    m_simulator.reset(m_uiWheelbase, m_uiVMax, m_uiDeltaMax, m_uiDt);
  }
  resetKeyWasPressed = resetKeyPressed;

  // Update key states for car control using keybindings
  m_keyW = glfwGetKey(m_window, gKeyBindings.carAccelerate) == GLFW_PRESS;
  m_keyS = glfwGetKey(m_window, gKeyBindings.carBrake) == GLFW_PRESS;
  m_keyA = glfwGetKey(m_window, gKeyBindings.carSteerLeft) == GLFW_PRESS;
  m_keyD = glfwGetKey(m_window, gKeyBindings.carSteerRight) == GLFW_PRESS;

  // Construct control input
  sim::CarInput input{};

  // Acceleration:
  const double accel = 7.0;
  if (m_keyW)
    input.ax = accel;
  else if (m_keyS)
    input.ax = -accel;
  else
    input.ax = 0.0;

  // Steering: +0.5 rad left, -0.5 rad right
  const double steer_angle = 0.5;
  if (m_keyA)
    input.delta = steer_angle;
  else if (m_keyD)
    input.delta = -steer_angle;
  else
    input.delta = 0.0;

  // Send to simulator
  m_simulator.setInput(input);
}

/**
 * @brief Sets up panel sections with their drawing functions.
 *
 * This is called once during construction to define what each panel contains.
 */
void Application::setupPanels() {
  // Simulation Control section
  m_rightPanel.addSection("Simulation Control", [this]() {
    bool isPaused = m_simulator.isPaused();
    if (ImGui::Button(isPaused ? "Resume" : "Pause", ImVec2(140, 30))) {
      if (isPaused) {
        m_simulator.resume();
      } else {
        m_simulator.pause();
      }
    }

    ImGui::SameLine();
    if (ImGui::Button("Step", ImVec2(70, 30))) {
      m_simulator.pause();
      m_simulator.step(1);
      m_simulator.resume();
    }

    // StepN with input field
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##StepN", &m_stepN, 1, 100);
    m_stepN = std::max(1, m_stepN);

    ImGui::SameLine();
    if (ImGui::Button("Step N", ImVec2(70, 30))) {
      m_simulator.pause();
      m_simulator.step((uint64_t)m_stepN);
      m_simulator.resume();
    }

    // Reset button
    if (ImGui::Button("Reset", ImVec2(-1, 30))) {
      m_simulator.reset(m_uiWheelbase, m_uiVMax, m_uiDeltaMax, m_uiDt);
    }
  });

  // Parameters section
  m_rightPanel.addSection("Parameters", [this]() {
    ImGui::Text("Apply on reset:");
    ImGui::Spacing();

    // Wheelbase
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
    ImGui::BeginChild("##wheelbase_param", ImVec2(0, 60), true);
    ImGui::Text("Wheelbase (m)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputFloat("##wheelbase", &m_uiWheelbase, 0.1f, 1.0f, "%.2f")) {
      m_uiWheelbase = std::max(0.1f, m_uiWheelbase);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // Max Velocity
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.12f, 0.12f, 1.0f));
    ImGui::BeginChild("##vmax_param", ImVec2(0, 60), true);
    ImGui::Text("Max Velocity (m/s)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputFloat("##vmax", &m_uiVMax, 1.0f, 5.0f, "%.1f")) {
      m_uiVMax = std::max(0.1f, m_uiVMax);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // Max Steering
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
    ImGui::BeginChild("##deltamax_param", ImVec2(0, 60), true);
    ImGui::Text("Max Steering (rad)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputFloat("##deltamax", &m_uiDeltaMax, 0.1f, 0.5f, "%.3f")) {
      m_uiDeltaMax = std::max(0.01f, m_uiDeltaMax);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // Timestep
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.12f, 0.12f, 1.0f));
    ImGui::BeginChild("##dt_param", ImVec2(0, 60), true);
    ImGui::Text("Timestep (s)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputFloat("##dt", &m_uiDt, 0.001f, 0.01f, "%.4f")) {
      m_uiDt = std::clamp(m_uiDt, 0.0001f, 0.1f);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    ImGui::Separator();
    ImGui::Text("Track Loading:");
    ImGui::Spacing();

    // Track directory input with Browse button
    ImGui::Text("Track Directory:");
    ImGui::SetNextItemWidth(-80);
    if (ImGui::InputText("##trackdir", m_trackDirBuffer,
                         sizeof(m_trackDirBuffer))) {
      scanTrackDirectory();
    }
    
    ImGui::SameLine();
    if (ImGui::Button("Browse...", ImVec2(70, 0))) {
      IGFD::FileDialogConfig config;
      config.path = m_trackDirBuffer;
      config.flags = ImGuiFileDialogFlags_Modal;
      ImGuiFileDialog::Instance()->OpenDialog("ChooseDirDlgKey", "Choose Directory", 
                                               nullptr, config);
    }

    // Display file dialog
    if (ImGuiFileDialog::Instance()->Display("ChooseDirDlgKey", 
                                              ImGuiWindowFlags_NoCollapse, 
                                              ImVec2(600, 400))) {
      if (ImGuiFileDialog::Instance()->IsOk()) {
        std::string selectedPath = ImGuiFileDialog::Instance()->GetCurrentPath();
        snprintf(m_trackDirBuffer, sizeof(m_trackDirBuffer), "%s", selectedPath.c_str());
        scanTrackDirectory();
      }
      ImGuiFileDialog::Instance()->Close();
    }

    ImGui::Spacing();
    ImGui::Text("Available Tracks:");

    // Track list with alternating colors
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.12f, 0.12f, 1.0f));
    ImGui::BeginChild("##tracklist", ImVec2(0, 150), true);

    if (m_availableTracks.empty()) {
      ImGui::TextDisabled("No tracks found");
    } else {
      for (size_t i = 0; i < m_availableTracks.size(); ++i) {
        // Alternating row colors
        ImVec4 rowColor = (i % 2 == 0) ? ImVec4(0.18f, 0.18f, 0.20f, 1.0f)
                                       : ImVec4(0.15f, 0.15f, 0.17f, 1.0f);

        ImGui::PushStyleColor(ImGuiCol_Header, rowColor);
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered,
                              ImVec4(0.25f, 0.25f, 0.30f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_HeaderActive,
                              ImVec4(0.30f, 0.40f, 0.50f, 1.0f));

        bool isSelected = (m_selectedTrackIndex == static_cast<int>(i));
        if (ImGui::Selectable(m_availableTracks[i].c_str(), isSelected,
                              ImGuiSelectableFlags_AllowDoubleClick,
                              ImVec2(0, 20))) {
          m_selectedTrackIndex = static_cast<int>(i);

          // Load track on double-click
          if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            std::string filepath = std::string(m_trackDirBuffer) + "/" +
                                   m_availableTracks[i] + ".csv";
            scene::TrackData trackData;
            if (scene::TrackLoader::loadFromCSV(filepath, trackData)) {
              m_simulator.setCones(trackData.cones);
              if (trackData.startPose.has_value()) {
                m_simulator.setStartPose(trackData.startPose.value());
              }
            }
          }
        }

        ImGui::PopStyleColor(3);
      }
    }

    ImGui::EndChild();
    ImGui::PopStyleColor();

    // Load button
    ImGui::Spacing();
    if (ImGui::Button("Load Selected Track", ImVec2(-1, 30))) {
      if (m_selectedTrackIndex >= 0 &&
          m_selectedTrackIndex < static_cast<int>(m_availableTracks.size())) {
        std::string filepath = std::string(m_trackDirBuffer) + "/" +
                               m_availableTracks[m_selectedTrackIndex] + ".csv";
        scene::TrackData trackData;
        if (scene::TrackLoader::loadFromCSV(filepath, trackData)) {
          m_simulator.setCones(trackData.cones);
          if (trackData.startPose.has_value()) {
            m_simulator.setStartPose(trackData.startPose.value());
          }
        }
      }
    }
  });

  // Status section
  m_rightPanel.addSection("Status", [this]() {
    const scene::Scene& scene = m_sceneDB.snapshot();
    const scene::CarState& car = scene.car;

    uint64_t tick = m_sceneDB.tick.load();
    double simTime = tick * m_simulator.getDt();
    ImGui::Text("Tick: %lu", (unsigned long)tick);
    ImGui::Text("Sim Time: %.3f s", simTime);
    ImGui::Text("Velocity: %.2f m/s", car.v);
    ImGui::Text("Position: (%.1f, %.1f)", car.x(), car.y());
    ImGui::Text("Yaw: %.1f deg", car.yaw() * 180.0 / M_PI);
    ImGui::Text("Camera: %s",
                m_viewportPanel.cameraMode == ViewportPanel::CameraMode::Free
                  ? "Free"
                  : "Car Follow");
    if (m_viewportPanel.cameraMode == ViewportPanel::CameraMode::Free) {
      ImGui::Text("Zoom: %.1f px/m", m_viewportPanel.freeCameraZoom);
      ImGui::Text("Pos: (%.1f, %.1f)", m_viewportPanel.freeCameraX,
                  m_viewportPanel.freeCameraY);
    } else {
      ImGui::Text("Zoom: %.1f px/m", m_viewportPanel.followCarZoom);
    }

    ImGui::Separator();
    ImGui::Text("Controls:");
    ImGui::BulletText("WASD: Drive car");
    ImGui::BulletText("Space: Toggle camera");
    ImGui::BulletText("Mouse drag: Pan (free mode)");
    ImGui::BulletText("Scroll: Zoom");
  });
}

/**
 * @brief Renders the 2D simulation scene and admin panel.
 */
void Application::render2D() {
  // Draw right panel
  m_rightPanel.draw(m_width, m_height);

  // Draw viewport (fill remaining space)
  float rightPanelWidth = m_rightPanel.getWidth();
  m_viewportPanel.draw(0, 0, (float)m_width - rightPanelWidth, (float)m_height);
}

/**
 * @brief Scans the track directory for CSV files.
 */
void Application::scanTrackDirectory() {
  m_availableTracks.clear();
  m_selectedTrackIndex = -1;

  namespace fs = std::filesystem;
  try {
    if (fs::exists(m_trackDirBuffer) && fs::is_directory(m_trackDirBuffer)) {
      for (const auto& entry : fs::directory_iterator(m_trackDirBuffer)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
          std::string stem = entry.path().stem().string();
          m_availableTracks.push_back(stem);
        }
      }
      std::sort(m_availableTracks.begin(), m_availableTracks.end());
    }
  } catch (...) {
    // Ignore errors
  }
}

} // namespace viz