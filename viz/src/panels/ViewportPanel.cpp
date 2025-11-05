#include "panels/ViewportPanel.hpp"
#include "KeyBindings.hpp"
#include <imgui.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace viz {

void ViewportPanel::draw(float x, float y, float width, float height) {
  const scene::Scene& scene = m_sceneDB.snapshot();
  const scene::CarState& car = scene.car;

  // Create ImGui window for viewport
  ImGui::SetNextWindowPos(ImVec2(x, y));
  ImGui::SetNextWindowSize(ImVec2(width, height));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar
                           | ImGuiWindowFlags_NoResize
                           | ImGuiWindowFlags_NoMove
                           | ImGuiWindowFlags_NoScrollbar
                           | ImGuiWindowFlags_NoScrollWithMouse
                           | ImGuiWindowFlags_NoCollapse
                           | ImGuiWindowFlags_NoBringToFrontOnFocus
                           | ImGuiWindowFlags_NoBackground;

  ImGui::Begin("Viewport", nullptr, flags);
  
  // Track hover state
  m_isHovered = ImGui::IsWindowHovered();

  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  
  // Draw dark gray background
  ImU32 bg_color = IM_COL32(45, 45, 45, 255);
  draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + width, y + height), bg_color);

  // Camera parameters
  const float cx = width * 0.5f;
  const float cy = height * 0.5f;
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
    float sx = x + cx + vx * cam_zoom;
    float sy = y + cy - vy * cam_zoom; // Flip Y for screen coords
    return ImVec2(sx, sy);
  };

  // Draw grid (1m x 1m cells)
  const float grid_size = 1.0f; // meters
  const int grid_range = 100;    // draw ±75 meters
  ImU32 grid_color = IM_COL32(100, 100, 100, 100);

  for (int i = -grid_range; i <= grid_range; ++i) {
    float world_coord = (float)i * grid_size;
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
  const float car_length = (float)car.wheelbase;
  const float car_width = 1.0f;

  // Triangle vertices in car local frame (+X forward, +Y left)
  ImVec2 v1 = worldToScreen(car_x + car_length * 0.5f * std::cos(car_yaw),
                            car_y + car_length * 0.5f * std::sin(car_yaw));
  ImVec2 v2 = worldToScreen(
    car_x
      + (-car_length * 0.5f * std::cos(car_yaw)
         + car_width * 0.5f * std::cos(car_yaw + (float)M_PI_2)),
    car_y
      + (-car_length * 0.5f * std::sin(car_yaw)
         + car_width * 0.5f * std::sin(car_yaw + (float)M_PI_2)));
  ImVec2 v3 = worldToScreen(
    car_x
      + (-car_length * 0.5f * std::cos(car_yaw)
         - car_width * 0.5f * std::cos(car_yaw + (float)M_PI_2)),
    car_y
      + (-car_length * 0.5f * std::sin(car_yaw)
         - car_width * 0.5f * std::sin(car_yaw + (float)M_PI_2)));

  ImU32 car_color = IM_COL32(255, 100, 100, 255);
  draw_list->AddTriangleFilled(v1, v2, v3, car_color);
  draw_list->AddTriangle(v1, v2, v3, IM_COL32(200, 50, 50, 255), 2.0f);

  // Draw cones
  for (const auto& cone : scene.cones) {
    ImVec2 center = worldToScreen((float)cone.x, (float)cone.y);

    // Cone dimensions and colors based on type
    float base_radius, stripe_radius, top_radius;
    ImU32 base_color, stripe_color;

    if (cone.type == scene::ConeType::BigOrange) {
      // Large orange cone: 0.50m diameter = 0.25m radius
      base_radius = 0.25f;
      stripe_radius = 0.16f;
      top_radius = 0.08f;
      base_color = IM_COL32(255, 140, 0, 255);     // Orange
      stripe_color = IM_COL32(255, 255, 255, 255); // White stripe
    } else {
      // Standard cones: 0.35m diameter = 0.175m radius
      base_radius = 0.175f;
      stripe_radius = 0.1f;
      top_radius = 0.06f;

      if (cone.type == scene::ConeType::Blue) {
        base_color = IM_COL32(50, 100, 255, 255);    // Blue
        stripe_color = IM_COL32(255, 255, 255, 255); // White stripe
      } else if (cone.type == scene::ConeType::Yellow) {
        base_color = IM_COL32(255, 220, 0, 255);  // Yellow
        stripe_color = IM_COL32(50, 50, 50, 255); // Black stripe
      } else { // Orange
        base_color = IM_COL32(255, 140, 0, 255);     // Orange
        stripe_color = IM_COL32(255, 255, 255, 255); // White stripe
      }
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

void ViewportPanel::handleInput(GLFWwindow* window) {
  // Toggle camera mode
  static bool toggleKeyWasPressed = false;
  bool toggleKeyPressed =
    glfwGetKey(window, gKeyBindings.toggleCameraMode) == GLFW_PRESS;
  if (toggleKeyPressed && !toggleKeyWasPressed) {
    if (cameraMode == CameraMode::CarFollow) {
      // Switch to free camera
      if (!m_freeCameraInitialized) {
        const scene::Scene& scene = m_sceneDB.snapshot();
        freeCameraX = (float)scene.car.x();
        freeCameraY = (float)scene.car.y();
        m_freeCameraInitialized = true;
      }
      cameraMode = CameraMode::Free;
    } else {
      cameraMode = CameraMode::CarFollow;
    }
  }
  toggleKeyWasPressed = toggleKeyPressed;

  // Free camera pan with mouse drag
  if (cameraMode == CameraMode::Free) {
    bool mousePressed =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    if (mousePressed && m_mouseLeftPressed) {
      // Pan camera
      float dx = (float)(mouseX - m_lastMouseX);
      float dy = (float)(mouseY - m_lastMouseY);

      // Transform mouse delta to world delta
      freeCameraY += dx / freeCameraZoom;
      freeCameraX += dy / freeCameraZoom;
    }

    m_mouseLeftPressed = mousePressed;
    m_lastMouseX = (float)mouseX;
    m_lastMouseY = (float)mouseY;
  }
}

} // namespace viz



