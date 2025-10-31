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
    float* zoom = &static_cast<Application*>(app)->zoom;
    *zoom *= (yoffset > 0) ? 1.1f : 0.9f;
    *zoom = std::clamp(*zoom, 5.0f, 500.0f); // Clamp zoom range
  }
}

/**
 * @brief Callback invoked when WebGPU adapter request completes.
 *
 * This callback is called asynchronously by wgpu-native when the adapter
 * request initiated by wgpuInstanceRequestAdapter() finishes.
 *
 * @param status The status of the adapter request
 * @param adapter The requested adapter handle (if successful)
 * @param message Optional error message
 * @param userdata1 Pointer to store the adapter handle
 * @param userdata2 Reserved for future use
 */
static void onAdapterRequestEnded(WGPURequestAdapterStatus status,
                                  WGPUAdapter adapter, WGPUStringView message,
                                  void* userdata1, void* userdata2) {
  (void)userdata2;
  (void)message;
  if (status == WGPURequestAdapterStatus_Success) {
    *(WGPUAdapter*)userdata1 = adapter;
  } else {
    fprintf(stderr, "Failed to get WebGPU adapter\n");
  }
}

/**
 * @brief Callback invoked when WebGPU device request completes.
 *
 * This callback is called asynchronously by wgpu-native when the device request
 * initiated by wgpuAdapterRequestDevice() finishes.
 *
 * @param status The status of the device request
 * @param device The requested device handle (if successful)
 * @param message Optional error message
 * @param userdata1 Pointer to store the device handle
 * @param userdata2 Reserved for future use
 */
static void onDeviceRequestEnded(WGPURequestDeviceStatus status,
                                 WGPUDevice device, WGPUStringView message,
                                 void* userdata1, void* userdata2) {
  (void)userdata2;
  (void)message;
  if (status == WGPURequestDeviceStatus_Success) {
    *(WGPUDevice*)userdata1 = device;
  } else {
    fprintf(stderr, "Failed to get WebGPU device\n");
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
  window = glfwCreateWindow(width, height, "lilsim", nullptr, nullptr);

  if (!window) {
    fprintf(stderr, "Failed to create window\n");
    return false;
  }

  // Set up window user pointer and callbacks
  glfwSetWindowUserPointer(window, this);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
  glfwSetScrollCallback(window, scrollCallback);

  fprintf(stderr, "[DEBUG] GLFW initialized successfully\n");
  return true;
}

/**
 * @brief Initializes WebGPU instance, surface, adapter, device, and queue.
 *
 * This method performs the complete WebGPU initialization sequence:
 * 1. Creates a WebGPU instance
 * 2. Creates a platform-specific surface (X11/Metal/Win32)
 * 3. Requests and waits for a compatible adapter
 * 4. Requests and waits for a logical device
 * 5. Obtains the command queue
 * 6. Configures the surface with appropriate format and present mode
 *
 * The surface creation is platform-specific:
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
  instance = wgpuCreateInstance(&instanceDesc);
  if (!instance) {
    fprintf(stderr, "Failed to create WebGPU instance\n");
    return false;
  }
  fprintf(stderr, "[DEBUG] WebGPU instance created\n");

  // Create surface (platform-specific)
#if defined(__linux__)
  // Linux X11
  WGPUSurfaceSourceXlibWindow fromXlibWindow = {};
  fromXlibWindow.chain.sType = WGPUSType_SurfaceSourceXlibWindow;
  fromXlibWindow.display = glfwGetX11Display();
  fromXlibWindow.window = glfwGetX11Window(window);

  WGPUSurfaceDescriptor surfaceDesc = {};
  surfaceDesc.nextInChain =
    reinterpret_cast<WGPUChainedStruct const*>(&fromXlibWindow);
  surface = wgpuInstanceCreateSurface(instance, &surfaceDesc);

#elif defined(__APPLE__)
  // macOS Metal
  id metal_layer = nullptr;
  NSWindow* ns_window = glfwGetCocoaWindow(window);
  [ns_window.contentView setWantsLayer:YES];
  metal_layer = [CAMetalLayer layer];
  [ns_window.contentView setLayer:metal_layer];

  WGPUSurfaceSourceMetalLayer fromMetalLayer = {};
  fromMetalLayer.chain.sType = WGPUSType_SurfaceSourceMetalLayer;
  fromMetalLayer.layer = metal_layer;

  WGPUSurfaceDescriptor surfaceDesc = {};
  surfaceDesc.nextInChain =
    reinterpret_cast<WGPUChainedStruct const*>(&fromMetalLayer);
  surface = wgpuInstanceCreateSurface(instance, &surfaceDesc);

#elif defined(_WIN32)
  // Windows
  HWND hwnd = glfwGetWin32Window(window);
  HINSTANCE hinstance = GetModuleHandle(nullptr);

  WGPUSurfaceSourceWindowsHWND fromWindowsHWND = {};
  fromWindowsHWND.chain.sType = WGPUSType_SurfaceSourceWindowsHWND;
  fromWindowsHWND.hinstance = hinstance;
  fromWindowsHWND.hwnd = hwnd;

  WGPUSurfaceDescriptor surfaceDesc = {};
  surfaceDesc.nextInChain =
    reinterpret_cast<WGPUChainedStruct const*>(&fromWindowsHWND);
  surface = wgpuInstanceCreateSurface(instance, &surfaceDesc);

#else
#error "Unsupported platform - only Linux, macOS, and Windows are supported"
#endif

  if (!surface) {
    fprintf(stderr, "Failed to create surface\n");
    return false;
  }
  fprintf(stderr, "[DEBUG] Surface created\n");

  // Request adapter
  fprintf(stderr, "[DEBUG] Requesting adapter...\n");
  WGPURequestAdapterOptions adapterOpts = {};
  adapterOpts.nextInChain = nullptr;
  adapterOpts.compatibleSurface = surface;
  adapterOpts.powerPreference = WGPUPowerPreference_HighPerformance;

  WGPURequestAdapterCallbackInfo adapterCallbackInfo = {};
  adapterCallbackInfo.mode = WGPUCallbackMode_AllowSpontaneous;
  adapterCallbackInfo.callback = onAdapterRequestEnded;
  adapterCallbackInfo.userdata1 = &adapter;

  wgpuInstanceRequestAdapter(instance, &adapterOpts, adapterCallbackInfo);

  // Wait for adapter
  int waitCount = 0;
  while (!adapter) {
    if (waitCount % 100 == 0) {
      fprintf(stderr, "[DEBUG] Waiting for adapter... (%d iterations)\n",
              waitCount);
    }
    wgpuInstanceProcessEvents(instance);
    glfwPollEvents(); // Keep window responsive during initialization
    waitCount++;
  }
  fprintf(stderr, "[DEBUG] Adapter acquired\n");

  // Create device
  fprintf(stderr, "[DEBUG] Requesting device...\n");
  WGPUDeviceDescriptor deviceDesc = {};
  deviceDesc.nextInChain = nullptr;
  deviceDesc.label = WGPU_STR("Main Device");

  WGPURequestDeviceCallbackInfo callbackInfo = {};
  callbackInfo.mode = WGPUCallbackMode_AllowSpontaneous;
  callbackInfo.callback = onDeviceRequestEnded;
  callbackInfo.userdata1 = &device;

  wgpuAdapterRequestDevice(adapter, &deviceDesc, callbackInfo);

  // Wait for device
  waitCount = 0;
  while (!device) {
    if (waitCount % 100 == 0) {
      fprintf(stderr, "[DEBUG] Waiting for device... (%d iterations)\n",
              waitCount);
    }
    wgpuInstanceProcessEvents(instance);
    glfwPollEvents(); // Keep window responsive during initialization
    waitCount++;
  }
  fprintf(stderr, "[DEBUG] Device acquired\n");

  queue = wgpuDeviceGetQueue(device);
  fprintf(stderr, "[DEBUG] Queue obtained\n");

  // Configure surface
  WGPUSurfaceConfiguration config = {};
  config.device = device;
  config.format = surfaceFormat;
  config.usage = WGPUTextureUsage_RenderAttachment;
  config.width = static_cast<uint32_t>(width);
  config.height = static_cast<uint32_t>(height);
  config.presentMode = WGPUPresentMode_Fifo;
  config.alphaMode = WGPUCompositeAlphaMode_Auto;

  wgpuSurfaceConfigure(surface, &config);
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
 * - Initializes the WebGPU renderer backend with appropriate surface format
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

  ImGui_ImplGlfw_InitForOther(window, true);

  ImGui_ImplWGPU_InitInfo init_info = {};
  init_info.Device = device;
  init_info.NumFramesInFlight = 3;
  init_info.RenderTargetFormat = surfaceFormat;
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
 * 2. WebGPU (graphics device)
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
 * 4. Acquires the current surface texture for rendering
 * 5. Creates a render pass with clear color
 * 6. Records ImGui draw commands into the render pass
 * 7. Submits the command buffer to the GPU queue
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

  // Get current texture from surface
  WGPUSurfaceTexture surfaceTexture = {};
  wgpuSurfaceGetCurrentTexture(surface, &surfaceTexture);

  // Handle special surface states
  if (surfaceTexture.status == WGPUSurfaceGetCurrentTextureStatus_Timeout
      || surfaceTexture.status == WGPUSurfaceGetCurrentTextureStatus_Outdated
      || surfaceTexture.status == WGPUSurfaceGetCurrentTextureStatus_Lost) {
    // Surface needs reconfiguration (probably due to resize)
    if (surfaceTexture.texture) {
      wgpuTextureRelease(surfaceTexture.texture);
    }

    // Get current framebuffer size and reconfigure
    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    if (fbWidth > 0 && fbHeight > 0) {
      onResize(fbWidth, fbHeight);
    }
    return; // Skip this frame
  }

  if (surfaceTexture.status != WGPUSurfaceGetCurrentTextureStatus_SuccessOptimal
      && surfaceTexture.status
           != WGPUSurfaceGetCurrentTextureStatus_SuccessSuboptimal) {
    fprintf(stderr, "Failed to get current surface texture: status %d\n",
            surfaceTexture.status);
    return;
  }

  // Create texture view
  WGPUTextureViewDescriptor viewDesc = {};
  viewDesc.format = surfaceFormat;
  viewDesc.dimension = WGPUTextureViewDimension_2D;
  viewDesc.baseMipLevel = 0;
  viewDesc.mipLevelCount = 1;
  viewDesc.baseArrayLayer = 0;
  viewDesc.arrayLayerCount = 1;
  viewDesc.aspect = WGPUTextureAspect_All;

  WGPUTextureView backbuffer =
    wgpuTextureCreateView(surfaceTexture.texture, &viewDesc);

  // Create render pass
  WGPURenderPassColorAttachment colorAttachment = {};
  colorAttachment.view = backbuffer;
  colorAttachment.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
  colorAttachment.loadOp = WGPULoadOp_Clear;
  colorAttachment.storeOp = WGPUStoreOp_Store;
  colorAttachment.clearValue = {clearColor[0] * clearColor[3],
                                clearColor[1] * clearColor[3],
                                clearColor[2] * clearColor[3], clearColor[3]};

  WGPURenderPassDescriptor renderPassDesc = {};
  renderPassDesc.colorAttachmentCount = 1;
  renderPassDesc.colorAttachments = &colorAttachment;
  renderPassDesc.depthStencilAttachment = nullptr;

  // Create command encoder
  WGPUCommandEncoderDescriptor encoderDesc = {};
  encoderDesc.label = WGPU_STR("Main Encoder");
  WGPUCommandEncoder encoder =
    wgpuDeviceCreateCommandEncoder(device, &encoderDesc);

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

  wgpuQueueSubmit(queue, 1, &cmdBuffer);
  wgpuCommandBufferRelease(cmdBuffer);

  // Present
  wgpuSurfacePresent(surface);

  // Release resources
  wgpuTextureViewRelease(backbuffer);
  wgpuTextureRelease(surfaceTexture.texture);
}

/**
 * @brief Checks if the application should continue running.
 *
 * @return true if the window exists and hasn't been closed by the user, false
 * otherwise
 */
bool Application::isRunning() {
  return window && !glfwWindowShouldClose(window);
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
  if (queue)
    wgpuQueueRelease(queue);
  if (device)
    wgpuDeviceRelease(device);
  if (adapter)
    wgpuAdapterRelease(adapter);
  if (surface)
    wgpuSurfaceRelease(surface);
  if (instance)
    wgpuInstanceRelease(instance);
}

/**
 * @brief Destroys the GLFW window and terminates GLFW.
 *
 * Cleans up GLFW resources:
 * 1. Destroys the window if it exists
 * 2. Terminates the GLFW library
 */
void Application::terminateGLFW() {
  if (window) {
    glfwDestroyWindow(window);
    window = nullptr;
  }
  glfwTerminate();
}

/**
 * @brief Terminates all application subsystems in reverse order.
 *
 * Calls termination methods in reverse initialization order:
 * 1. ImGui (user interface)
 * 2. WebGPU (graphics device)
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
 * @brief Handles window resize events by reconfiguring the WebGPU surface.
 *
 * This method is called when the window framebuffer size changes. It updates
 * the internal width/height and reconfigures the WebGPU surface to match
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

  width = newWidth;
  height = newHeight;

  // Reconfigure the surface with new dimensions
  if (surface && device) {
    WGPUSurfaceConfiguration config = {};
    config.device = device;
    config.format = surfaceFormat;
    config.usage = WGPUTextureUsage_RenderAttachment;
    config.width = static_cast<uint32_t>(width);
    config.height = static_cast<uint32_t>(height);
    config.presentMode = WGPUPresentMode_Fifo;
    config.alphaMode = WGPUCompositeAlphaMode_Auto;

    wgpuSurfaceConfigure(surface, &config);
    fprintf(stderr, "[DEBUG] Surface reconfigured\n");
  }
}

/**
 * @brief Handles keyboard input and sends control commands to the simulator.
 *
 * Reads WASD key states and constructs CarInput commands:
 * - W: Accelerate forward
 * - S: Accelerate backward (brake)
 * - A: Steer left
 * - D: Steer right
 */
void Application::handleInput() {
  // Update key states
  keyW = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
  keyS = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
  keyA = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
  keyD = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;

  // Construct control input
  sim::CarInput input{};

  // Acceleration: W = +2.0 m/s², S = -2.0 m/s²
  const double accel = 2.0;
  if (keyW)
    input.ax = accel;
  else if (keyS)
    input.ax = -accel;
  else
    input.ax = 0.0;

  // Steering: A = -0.5 rad (left), D = +0.5 rad (right)
  const double steer_angle = 0.5;
  if (keyA)
    input.delta = steer_angle;
  else if (keyD)
    input.delta = -steer_angle;
  else
    input.delta = 0.0;

  // Send to simulator
  simulator.setInput(input);
}

/**
 * @brief Renders the 2D simulation scene using ImGui draw lists.
 *
 * Draws:
 * - Grid background (1m x 1m cells in world frame)
 * - Car as a triangle
 * - Camera follows car and rotates with it
 */
void Application::render2D() {
  // Get current scene state
  const scene::Scene& scene = sceneDB.snapshot();
  const scene::CarState& car = scene.car;

  // Create fullscreen ImGui window for 2D viewport
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2((float)width, (float)height));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::Begin(
    "Viewport", nullptr,
    ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
      | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar
      | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoBackground);

  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  // Camera transform: center on car, rotate with car
  const float cx = width * 0.5f;
  const float cy = height * 0.5f;
  const float car_x = (float)car.x();
  const float car_y = (float)car.y();
  const float car_yaw = (float)car.yaw();

  // Helper lambda to transform world coordinates to screen coordinates
  // Camera setup: +X up in viewport, +Y left in viewport
  auto worldToScreen = [&](float wx, float wy) -> ImVec2 {
    // Translate to car frame
    float dx = wx - car_x;
    float dy = wy - car_y;
    // Rotate by -car_yaw (to keep car pointing up)
    float cos_yaw = std::cos(-car_yaw);
    float sin_yaw = std::sin(-car_yaw);
    float rx = dx * cos_yaw - dy * sin_yaw;
    float ry = dx * sin_yaw + dy * cos_yaw;
    // Apply additional -90° rotation: +X world -> up in viewport, +Y world ->
    // left World (rx, ry) -> Viewport (-ry, rx) to get +X up and +Y left
    float vx = -ry;
    float vy = rx;
    // Scale and translate to screen
    // Note: Y axis is flipped in screen coordinates (+Y down)
    float sx = cx + vx * zoom;
    float sy = cy - vy * zoom; // Flip Y for screen coords
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

  ImGui::End();
  ImGui::PopStyleVar();
}

} // namespace viz