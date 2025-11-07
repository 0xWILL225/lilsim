#include "panels/ViewportPanel.hpp"
#include "KeyBindings.hpp"
#include "MarkerSystem.hpp"
#include "TextureManager.hpp"
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


  // Draw cones
  if (m_showCones && *m_showCones) {
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
  }

  // Draw markers
  if (m_markerSystem) {
    const auto& markers = m_markerSystem->getMarkers();
    for (const auto& [key, marker] : markers) {
      // Check visibility
      if (!m_markerSystem->isMarkerVisible(key.ns, key.id)) {
        continue;
      }

      // Convert marker color
      ImU32 markerColor = IM_COL32(marker.color.r, marker.color.g, 
                                     marker.color.b, marker.color.a);

      // Render based on marker type
      switch (marker.type) {
        case MarkerType::LineStrip:
          if (marker.points.size() >= 2) {
            for (size_t i = 0; i < marker.points.size() - 1; ++i) {
              const auto& p1 = marker.points[i];
              const auto& p2 = marker.points[i + 1];
              ImVec2 screen1 = worldToScreen((float)p1.x(), (float)p1.y());
              ImVec2 screen2 = worldToScreen((float)p2.x(), (float)p2.y());
              float lineWidth = marker.scale.x * cam_zoom;
              draw_list->AddLine(screen1, screen2, markerColor, lineWidth);
            }
          }
          break;

        case MarkerType::Circle: {
          ImVec2 center = worldToScreen((float)marker.pose.x(), 
                                        (float)marker.pose.y());
          float radius = marker.scale.x * cam_zoom;
          draw_list->AddCircleFilled(center, radius, markerColor, 32);
          break;
        }

        // Other marker types can be added here as needed
        default:
          break;
      }
    }
  }

  // Draw car as sprite (if visible)
  if (m_showCar && *m_showCar) {
    // Load texture on first use
    auto& texMgr = TextureManager::getInstance();
    const auto* texData = texMgr.getTexture("pixel_x2.png");
    if (!texData) {
      texData = texMgr.loadTexture("pixel_x2.png");
    }

    if (texData && texData->imguiTextureID) {
      const float car_length = (float)car.wheelbase;
      const float car_width = 1.46f;

      // Calculate sprite dimensions in world space
      // The sprite should be scaled to match car_length and car_width
      float sprite_half_length = car_length * 0.5f;
      float sprite_half_width = car_width * 0.5f;

      // Calculate the 4 corners of the sprite in car's local frame
      // Sprite faces upward in image (+Y up in image = +X forward in world)
      // So we need to rotate the sprite by car_yaw
      
      // Local frame corners (before rotation):
      // Front-right, front-left, back-left, back-right
      float cos_yaw = std::cos(car_yaw);
      float sin_yaw = std::sin(car_yaw);

      // Corner offsets in local frame (sprite centered at origin)
      float corners_local[4][2] = {
        { sprite_half_length, -sprite_half_width},  // Front-right
        { sprite_half_length,  sprite_half_width},  // Front-left
        {-sprite_half_length,  sprite_half_width},  // Back-left
        {-sprite_half_length, -sprite_half_width}   // Back-right
      };

      // Transform to world space and then to screen space
      ImVec2 corners_screen[4];
      for (int i = 0; i < 4; ++i) {
        float local_x = corners_local[i][0];
        float local_y = corners_local[i][1];
        
        // Rotate by car_yaw
        float world_x = car_x + local_x * cos_yaw - local_y * sin_yaw;
        float world_y = car_y + local_x * sin_yaw + local_y * cos_yaw;
        
        corners_screen[i] = worldToScreen(world_x, world_y);
      }

      // UV coordinates (full texture)
      ImVec2 uv0(0.0f, 0.0f);  // Top-left
      ImVec2 uv1(1.0f, 0.0f);  // Top-right
      ImVec2 uv2(1.0f, 1.0f);  // Bottom-right
      ImVec2 uv3(0.0f, 1.0f);  // Bottom-left

      // Draw sprite as a rotated quad
      // ImGui's AddImageQuad takes vertices in this order: p1, p2, p3, p4
      // where p1=top-left, p2=top-right, p3=bottom-right, p4=bottom-left in UV space
      // Our corners are: [0]=front-right, [1]=front-left, [2]=back-left, [3]=back-right
      // Sprite image has car facing up, so top of image = front of car
      draw_list->AddImageQuad(
        texData->imguiTextureID,
        corners_screen[1],  // Front-left (top-left in sprite)
        corners_screen[0],  // Front-right (top-right in sprite)
        corners_screen[3],  // Back-right (bottom-right in sprite)
        corners_screen[2],  // Back-left (bottom-left in sprite)
        uv0, uv1, uv2, uv3
      );
    } else {
      // Fallback to triangle if texture failed to load
      const float car_length = (float)car.wheelbase;
      const float car_width = 1.0f;

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
    }
  }
  
  // ===== HUD Overlay (bottom-right corner) =====
  // Fixed in screen space, not affected by camera zoom/pan
  
  const float hud_margin_right = 20.0f;
  const float hud_margin_bottom = 30.0f;
  const float gap_between_elements = 20.0f;
  const float bar_width = 30.0f * 1.5f;   // 45px (scaled 1.5x)
  const float bar_height = 150.0f * 2.0f; // 300px (scaled 2x)
  
  // Load steering wheel texture
  auto& texMgr = TextureManager::getInstance();
  const auto* wheelTexData = texMgr.getTexture("steering_wheel.png");
  if (!wheelTexData) {
    wheelTexData = texMgr.loadTexture("steering_wheel.png");
  }
  
  if (wheelTexData && wheelTexData->imguiTextureID) {
    // Preserve aspect ratio of steering wheel texture
    // Original: 57×35 pixels (wider than tall)
    float aspect_ratio = static_cast<float>(wheelTexData->width) / static_cast<float>(wheelTexData->height);
    const float wheel_height = 100.0f * 1.5f;  // 150px (scaled 1.5x)
    const float wheel_width = wheel_height * aspect_ratio;  // Preserve aspect ratio
    
    const float wheel_center_x = x + width - hud_margin_right - gap_between_elements - bar_width - gap_between_elements - wheel_width * 0.5f;
    const float wheel_center_y = y + height - hud_margin_bottom - wheel_height * 0.5f;
    
    // Get steering angle directly from scene snapshot (no smoothing/interpolation)
    const float delta = static_cast<float>(scene.car_input.delta);  // radians
    
    // Calculate rotated corners of steering wheel sprite
    // Rotation matrix: [cos -sin]
    //                  [sin  cos]
    float cos_delta = std::cos(delta);
    float sin_delta = std::sin(delta);
    float half_width = wheel_width * 0.5f;
    float half_height = wheel_height * 0.5f;
    
    // Four corners relative to center (preserving aspect ratio), then rotated
    ImVec2 corners[4];
    float local_corners[4][2] = {
      {-half_width, -half_height},  // Top-left
      { half_width, -half_height},  // Top-right
      { half_width,  half_height},  // Bottom-right
      {-half_width,  half_height}   // Bottom-left
    };
    
    for (int i = 0; i < 4; ++i) {
      float lx = local_corners[i][0];
      float ly = local_corners[i][1];
      float rx =   lx * cos_delta + ly * sin_delta;
      float ry = - lx * sin_delta + ly * cos_delta;
      corners[i] = ImVec2(wheel_center_x + rx, wheel_center_y + ry);
    }
    
    // UV coordinates
    ImVec2 uv0(0.0f, 0.0f);  // Top-left
    ImVec2 uv1(1.0f, 0.0f);  // Top-right
    ImVec2 uv2(1.0f, 1.0f);  // Bottom-right
    ImVec2 uv3(0.0f, 1.0f);  // Bottom-left
    
    // Draw rotated steering wheel
    draw_list->AddImageQuad(
      wheelTexData->imguiTextureID,
      corners[0], corners[1], corners[2], corners[3],
      uv0, uv1, uv2, uv3
    );
    
    // Draw horizontal steering bar below wheel
    const float steering_bar_height = 30.0f * 1.5f;  // 45px (scaled 1.5x)
    const float steering_bar_width = wheel_width;     // Match steering wheel width
    const float steering_bar_gap = 10.0f;             // Gap between wheel and bar
    
    const float steering_bar_center_x = wheel_center_x;
    const float steering_bar_top_y = wheel_center_y + wheel_height * 0.5f + steering_bar_gap;
    const float steering_bar_bottom_y = steering_bar_top_y + steering_bar_height;
    const float steering_bar_left_x = steering_bar_center_x - steering_bar_width * 0.5f;
    const float steering_bar_right_x = steering_bar_center_x + steering_bar_width * 0.5f;
    
    // Draw bar border
    ImVec2 steering_bar_min(steering_bar_left_x, steering_bar_top_y);
    ImVec2 steering_bar_max(steering_bar_right_x, steering_bar_bottom_y);
    draw_list->AddRect(steering_bar_min, steering_bar_max, IM_COL32(60, 60, 60, 255), 0.0f, 0, 2.0f);
    
    // Determine what to visualize based on steering mode
    float steering_value = 0.0f;
    float steering_limit = 1.0f;
    
    if (scene.steering_mode == scene::SteeringMode::Rate) {
      // Show steering rate
      steering_value = static_cast<float>(scene.car_input.delta_dot);
      steering_limit = static_cast<float>(scene.car_input.steer_rate_max);
    } else {
      // Show steering angle
      steering_value = static_cast<float>(scene.car_input.delta);
      steering_limit = static_cast<float>(scene.car_input.delta_max);
    }
    
    // Calculate fill (bidirectional from center)
    if (std::abs(steering_value) > 0.001f && std::abs(steering_limit) > 0.001f) {
      float fill_fraction = std::clamp(std::abs(steering_value) / steering_limit, 0.0f, 1.0f);
      float fill_width = (steering_bar_width * 0.5f - 2.0f) * fill_fraction;
      
      // Color gradient: purple to blue
      ImU32 color_center = IM_COL32(200, 100, 255, 200);  // Purple
      ImU32 color_edge = IM_COL32(100, 150, 255, 200);    // Blue
      
      if (steering_value > 0.0f) {
        // Positive value: fill left (steering left)
        draw_list->AddRectFilledMultiColor(
          ImVec2(steering_bar_center_x - fill_width, steering_bar_top_y + 2),
          ImVec2(steering_bar_center_x, steering_bar_bottom_y - 2),
          color_center, color_edge, color_edge, color_center
        );
      } else {
        // Negative value: fill right (steering right)
        draw_list->AddRectFilledMultiColor(
          ImVec2(steering_bar_center_x, steering_bar_top_y + 2),
          ImVec2(steering_bar_center_x + fill_width, steering_bar_bottom_y - 2),
          color_edge, color_center, color_center, color_edge
        );
      }
    }
  }
  
  // Draw acceleration bar
  const float bar_center_x = x + width - hud_margin_right - bar_width * 0.5f;
  const float bar_bottom_y = y + height - hud_margin_bottom;
  const float bar_top_y = bar_bottom_y - bar_height;
  
  // Get acceleration input from scene
  const float ax = static_cast<float>(scene.car_input.ax);
  const float ax_max = static_cast<float>(scene.car_input.ax_max);
  
  // Draw bar border (dark grey)
  ImVec2 bar_min(bar_center_x - bar_width * 0.5f, bar_top_y);
  ImVec2 bar_max(bar_center_x + bar_width * 0.5f, bar_bottom_y);
  draw_list->AddRect(bar_min, bar_max, IM_COL32(60, 60, 60, 255), 0.0f, 0, 2.0f);
  
  // Calculate fill amount
  if (std::abs(ax) > 0.001f && std::abs(ax_max) > 0.001f) {
    float fill_fraction = std::clamp(std::abs(ax) / ax_max, 0.0f, 1.0f);
    float fill_height = bar_height * fill_fraction;
    
    if (ax > 0.0f) {
      // Positive acceleration: Yellow (bottom) to Red (top)
      float bar_fill_bottom_y = bar_bottom_y;
      float bar_fill_top_y = bar_bottom_y - fill_height;
      
      // Create gradient: Yellow at bottom, Red at top
      ImU32 color_bottom = IM_COL32(255, 255, 0, 200);  // Yellow
      ImU32 color_top = IM_COL32(255, 0, 0, 200);       // Red
      
      draw_list->AddRectFilledMultiColor(
        ImVec2(bar_min.x + 2, bar_fill_top_y),
        ImVec2(bar_max.x - 2, bar_fill_bottom_y - 2),
        color_top, color_top, color_bottom, color_bottom
      );
    } else {
      // Negative acceleration (braking): Green (bottom) to Blue (top)
      float bar_fill_bottom_y = bar_bottom_y;
      float bar_fill_top_y = bar_bottom_y - fill_height;
      
      // Create gradient: Green at bottom, Blue at top
      ImU32 color_bottom = IM_COL32(0, 255, 0, 200);    // Green
      ImU32 color_top = IM_COL32(0, 100, 255, 200);     // Blue
      
      draw_list->AddRectFilledMultiColor(
        ImVec2(bar_min.x + 2, bar_fill_top_y),
        ImVec2(bar_max.x - 2, bar_fill_bottom_y - 2),
        color_top, color_top, color_bottom, color_bottom
      );
    }
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



