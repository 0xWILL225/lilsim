#include "viz.hpp"

#include <GLFW/glfw3native.h>
#include <cstdio>

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
    // Zoom in/out with scroll wheel (10% per scroll tick)
    float zoomDelta = (yoffset > 0) ? 1.1f : 0.9f;

    if (app->cameraMode == Application::CameraMode::Free) {
      app->freeCameraZoom *= zoomDelta;
      app->freeCameraZoom = std::clamp(app->freeCameraZoom, 5.0f, 500.0f);
    } else {
      app->followCarZoom *= zoomDelta;
      app->followCarZoom = std::clamp(app->followCarZoom, 5.0f, 500.0f);
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
  // Toggle camera mode with spacebar
  static bool spaceWasPressed = false;
  static bool freeCameraInitialized = false;
  bool spacePressed = glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS;
  if (spacePressed && !spaceWasPressed) {
    if (cameraMode == CameraMode::CarFollow) {
      // Switch to free camera
      // Only initialize position on first switch, otherwise keep saved position
      if (!freeCameraInitialized) {
        const scene::Scene& scene = m_sceneDB.snapshot();
        freeCameraX = (float)scene.car.x();
        freeCameraY = (float)scene.car.y();
        freeCameraInitialized = true;
      }
      cameraMode = CameraMode::Free;
    } else {
      cameraMode = CameraMode::CarFollow;
    }
  }
  spaceWasPressed = spacePressed;

  // Free camera pan with mouse drag
  if (cameraMode == CameraMode::Free) {
    bool mousePressed =
      glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    double mouseX, mouseY;
    glfwGetCursorPos(m_window, &mouseX, &mouseY);

    if (mousePressed && m_mouseLeftPressed) {
      // Pan camera
      float dx = (float)(mouseX - m_lastMouseX);
      float dy = (float)(mouseY - m_lastMouseY);

      // Transform mouse delta to world delta
      // Mouse right (+dx) -> world +Y (left in viewport)
      // Mouse down (+dy) -> world -X (down in viewport, which is -X up)
      freeCameraY += dx / freeCameraZoom;
      freeCameraX += dy / freeCameraZoom;
    }

    m_mouseLeftPressed = mousePressed;
    m_lastMouseX = (float)mouseX;
    m_lastMouseY = (float)mouseY;
  }

  // Update key states for car control
  m_keyW = glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS;
  m_keyS = glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS;
  m_keyA = glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS;
  m_keyD = glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS;

  // Construct control input
  sim::CarInput input{};

  // Acceleration: W = +2.0 m/s², S = -2.0 m/s²
  const double accel = 2.0;
  if (m_keyW)
    input.ax = accel;
  else if (m_keyS)
    input.ax = -accel;
  else
    input.ax = 0.0;

  // Steering: A = -0.5 rad (left), D = +0.5 rad (right)
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
 * @brief Renders the 2D simulation scene and admin panel.
 *
 * Draws:
 * - Admin panel (right side): simulation info and controls reference
 * - Grid background (1m x 1m cells in world frame)
 * - Car as a red triangle
 * - Cones as colored circles (blue with white stripe, yellow with black stripe)
 * - Camera follows car or is free-moving based on mode
 */
void Application::render2D() {
  // Get current scene state
  const scene::Scene& scene = m_sceneDB.snapshot();
  const scene::CarState& car = scene.car;

  // Admin panel (right side)
  const float collapsedWidth = 30.0f;
  const float currentPanelWidth = panelCollapsed ? collapsedWidth : panelWidth;

  ImGui::SetNextWindowPos(ImVec2((float)m_width - currentPanelWidth, 0));
  ImGui::SetNextWindowSize(ImVec2(currentPanelWidth, (float)m_height));
  ImGui::Begin("Admin", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
                 | ImGuiWindowFlags_NoTitleBar);

  if (panelCollapsed) {
    // Show just expand button when collapsed
    if (ImGui::Button("<", ImVec2(20, 30))) {
      panelCollapsed = false;
    }
  } else {
    // Resize handle (invisible button on left edge)
    ImGui::SetCursorPos(ImVec2(0, 0));
    ImGui::InvisibleButton("##resize", ImVec2(5, (float)m_height));

    if (ImGui::IsItemHovered()) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    if (ImGui::IsItemActive()) {
      panelResizing = true;
    }

    if (panelResizing) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        double mouseX, mouseY;
        glfwGetCursorPos(m_window, &mouseX, &mouseY);
        panelWidth = std::clamp((float)(m_width - mouseX), 150.0f, 600.0f);
      } else {
        panelResizing = false;
      }
    }

    ImGui::SetCursorPos(ImVec2(10, 5));
    // Collapse button
    if (ImGui::Button(">", ImVec2(20, 20))) {
      panelCollapsed = true;
    }
    ImGui::SameLine();
    ImGui::Text("Admin Panel");

    ImGui::Separator();
    ImGui::Text("Simulation Control");
    ImGui::Separator();

    // Sim control buttons
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
    static int stepN = 10;
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##StepN", &stepN, 1, 100);
    stepN = std::max(1, stepN);

    ImGui::SameLine();
    if (ImGui::Button("Step N", ImVec2(70, 30))) {
      m_simulator.pause();
      m_simulator.step((uint64_t)stepN);
      m_simulator.resume();
    }

    // Reset button
    static float uiWheelbase = 2.6f;
    static float uiVMax = 15.0f;
    static float uiDeltaMax = 1.745f;
    static float uiDt = 1.0f / 200.0f;

    if (ImGui::Button("Reset", ImVec2(-1, 30))) {
      m_simulator.reset(uiWheelbase, uiVMax, uiDeltaMax, uiDt);
    }

    ImGui::Separator();

    // Simulation parameters (apply on reset)
    ImGui::Text("Parameters (apply on reset):");
    ImGui::Spacing();

    // Wheelbase
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
    ImGui::BeginChild("##wheelbase_param", ImVec2(0, 60), true);
    ImGui::Text("Wheelbase (m)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputFloat("##wheelbase", &uiWheelbase, 0.1f, 1.0f, "%.2f")) {
      uiWheelbase = std::max(0.1f, uiWheelbase);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // Max Velocity
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.12f, 0.12f, 1.0f));
    ImGui::BeginChild("##vmax_param", ImVec2(0, 60), true);
    ImGui::Text("Max Velocity (m/s)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputFloat("##vmax", &uiVMax, 1.0f, 5.0f, "%.1f")) {
      uiVMax = std::max(0.1f, uiVMax);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // Max Steering
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
    ImGui::BeginChild("##deltamax_param", ImVec2(0, 60), true);
    ImGui::Text("Max Steering (rad)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputFloat("##deltamax", &uiDeltaMax, 0.1f, 0.5f, "%.3f")) {
      uiDeltaMax = std::max(0.01f, uiDeltaMax);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    // Timestep
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.12f, 0.12f, 1.0f));
    ImGui::BeginChild("##dt_param", ImVec2(0, 60), true);
    ImGui::Text("Timestep (s)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputFloat("##dt", &uiDt, 0.001f, 0.01f, "%.4f")) {
      uiDt = std::clamp(uiDt, 0.0001f, 0.1f);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    ImGui::Separator();

    // Display info
    uint64_t tick = m_sceneDB.tick.load();
    double simTime = tick * m_simulator.getDt();
    ImGui::Text("Tick: %lu", (unsigned long)tick);
    ImGui::Text("Sim Time: %.3f s", simTime);
    ImGui::Text("Velocity: %.2f m/s", car.v);
    ImGui::Text("Position: (%.1f, %.1f)", car.x(), car.y());
    ImGui::Text("Yaw: %.1f deg", car.yaw() * 180.0 / M_PI);
    ImGui::Text("Camera: %s",
                cameraMode == CameraMode::Free ? "Free" : "Car Follow");
    if (cameraMode == CameraMode::Free) {
      ImGui::Text("Zoom: %.1f px/m", freeCameraZoom);
      ImGui::Text("Pos: (%.1f, %.1f)", freeCameraX, freeCameraY);
    } else {
      ImGui::Text("Zoom: %.1f px/m", followCarZoom);
    }

    ImGui::Separator();
    ImGui::Text("Controls:");
    ImGui::BulletText("WASD: Drive car");
    ImGui::BulletText("Space: Toggle camera");
    ImGui::BulletText("Mouse drag: Pan (free mode)");
    ImGui::BulletText("Scroll: Zoom");
  }

  ImGui::End();

  // Create ImGui window for 2D viewport (leave space for panel)
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(
    ImVec2((float)m_width - currentPanelWidth, (float)m_height));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::Begin(
    "Viewport", nullptr,
    ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
      | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar
      | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoBackground);

  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  // Camera parameters based on mode
  const float cx = (m_width - currentPanelWidth) * 0.5f;
  const float cy = m_height * 0.5f;
  const float car_x = (float)car.x();
  const float car_y = (float)car.y();
  const float car_yaw = (float)car.yaw();

  float cam_x, cam_y, cam_yaw, cam_zoom;

  if (cameraMode == CameraMode::CarFollow) {
    cam_x = car_x;
    cam_y = car_y;
    cam_yaw = car_yaw;
    cam_zoom = followCarZoom;
  } else {
    cam_x = freeCameraX;
    cam_y = freeCameraY;
    cam_yaw = 0.0f; // Free camera doesn't rotate
    cam_zoom = freeCameraZoom;
  }

  // Helper lambda to transform world coordinates to screen coordinates
  // Camera setup: +X up in viewport, +Y left in viewport
  auto worldToScreen = [&](float wx, float wy) -> ImVec2 {
    // Translate to camera frame
    float dx = wx - cam_x;
    float dy = wy - cam_y;
    // Rotate by -cam_yaw
    float cos_yaw = std::cos(-cam_yaw);
    float sin_yaw = std::sin(-cam_yaw);
    float rx = dx * cos_yaw - dy * sin_yaw;
    float ry = dx * sin_yaw + dy * cos_yaw;
    // Apply -90° rotation: +X world -> up in viewport, +Y world -> left
    float vx = -ry;
    float vy = rx;
    // Scale and translate to screen
    float sx = cx + vx * cam_zoom;
    float sy = cy - vy * cam_zoom; // Flip Y for screen coords
    return ImVec2(sx, sy);
  };

  // Draw grid (1m x 1m cells)
  const float grid_size = 1.0f; // meters
  const int grid_range = 50;    // draw ±50 meters
  ImU32 grid_color = IM_COL32(100, 100, 100, 100);

  for (int i = -grid_range; i <= grid_range; ++i) {
    float world_coord = i * grid_size;
    // Vertical lines (constant X)
    ImVec2 p1 = worldToScreen(world_coord, -grid_range * grid_size);
    ImVec2 p2 = worldToScreen(world_coord, grid_range * grid_size);
    draw_list->AddLine(p1, p2, grid_color, 1.0f);
    // Horizontal lines (constant Y)
    ImVec2 p3 = worldToScreen(-grid_range * grid_size, world_coord);
    ImVec2 p4 = worldToScreen(grid_range * grid_size, world_coord);
    draw_list->AddLine(p3, p4, grid_color, 1.0f);
  }

  // Draw car as triangle
  // Car dimensions: wheelbase length, 1m wide
  const float car_length = (float)car.wheelbase;
  const float car_width = 1.0f;

  // Triangle vertices in car local frame (+X forward, +Y left)
  // Front tip, rear left, rear right
  ImVec2 v1 = worldToScreen(car_x + car_length * 0.5f * std::cos(car_yaw),
                            car_y + car_length * 0.5f * std::sin(car_yaw));
  ImVec2 v2 =
    worldToScreen(car_x
                    + (-car_length * 0.5f * std::cos(car_yaw)
                       + car_width * 0.5f * std::cos(car_yaw + M_PI_2)),
                  car_y
                    + (-car_length * 0.5f * std::sin(car_yaw)
                       + car_width * 0.5f * std::sin(car_yaw + M_PI_2)));
  ImVec2 v3 =
    worldToScreen(car_x
                    + (-car_length * 0.5f * std::cos(car_yaw)
                       - car_width * 0.5f * std::cos(car_yaw + M_PI_2)),
                  car_y
                    + (-car_length * 0.5f * std::sin(car_yaw)
                       - car_width * 0.5f * std::sin(car_yaw + M_PI_2)));

  ImU32 car_color = IM_COL32(255, 100, 100, 255);
  draw_list->AddTriangleFilled(v1, v2, v3, car_color);
  draw_list->AddTriangle(v1, v2, v3, IM_COL32(200, 50, 50, 255), 2.0f);

  // Draw cones
  for (const auto& cone : scene.cones) {
    ImVec2 center = worldToScreen((float)cone.x, (float)cone.y);

    // Cone dimensions (viewed from above)
    const float base_radius = 0.125f;  // 0.25m diameter base
    const float stripe_radius = 0.08f; // middle stripe
    const float top_radius = 0.04f;    // top circle

    // Colors based on cone type
    ImU32 base_color, stripe_color;
    if (cone.type == scene::ConeType::Blue) {
      base_color = IM_COL32(50, 100, 255, 255);    // Blue
      stripe_color = IM_COL32(255, 255, 255, 255); // White stripe
    } else {
      base_color = IM_COL32(255, 220, 0, 255);  // Yellow
      stripe_color = IM_COL32(50, 50, 50, 255); // Black stripe
    }

    // Draw three circles (largest to smallest)
    draw_list->AddCircleFilled(center, base_radius * cam_zoom, base_color, 16);
    draw_list->AddCircleFilled(center, stripe_radius * cam_zoom, stripe_color,
                               12);
    draw_list->AddCircleFilled(center, top_radius * cam_zoom, base_color, 8);
  }

  ImGui::End();
  ImGui::PopStyleVar();
}

} // namespace viz