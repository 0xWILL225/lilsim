#include "panels/ViewportPanel.hpp"
#include "KeyBindings.hpp"
#include "MarkerSystem.hpp"
#include "TextureManager.hpp"
#include "scene.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace viz {

void ViewportPanel::draw(float x, float y, float width, float height, const RenderState& state) {
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
  
  m_isHovered = ImGui::IsWindowHovered();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  
  ImU32 bg_color = IM_COL32(45, 45, 45, 255);
  draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + width, y + height), bg_color);

  const float cx = width * 0.5f;
  const float cy = height * 0.5f;
  const float car_x = (float)state.x;
  const float car_y = (float)state.y;
  const float car_yaw = (float)state.yaw;

  float cam_x, cam_y, cam_yaw, cam_zoom;

  if (cameraMode == CameraMode::CarFollow) {
    cam_x = car_x;
    cam_y = car_y;
    cam_yaw = car_yaw;
    cam_zoom = followCarZoom;
  } else {
    cam_x = freeCameraX;
    cam_y = freeCameraY;
    cam_yaw = 0.0f;
    cam_zoom = freeCameraZoom;
  }

  auto worldToScreen = [&](float wx, float wy) -> ImVec2 {
    float dx = wx - cam_x;
    float dy = wy - cam_y;
    float cos_yaw = std::cos(-cam_yaw);
    float sin_yaw = std::sin(-cam_yaw);
    float rx = dx * cos_yaw - dy * sin_yaw;
    float ry = dx * sin_yaw + dy * cos_yaw;
    float vx = -ry;
    float vy = rx;
    float sx = x + cx + vx * cam_zoom;
    float sy = y + cy - vy * cam_zoom; 
    return ImVec2(sx, sy);
  };

  const float grid_size = 1.0f;
  const int grid_range = 100;
  ImU32 grid_color = IM_COL32(100, 100, 100, 100);

  for (int i = -grid_range; i <= grid_range; ++i) {
    float world_coord = (float)i * grid_size;
    ImVec2 p1 = worldToScreen(world_coord, -grid_range * grid_size);
    ImVec2 p2 = worldToScreen(world_coord, grid_range * grid_size);
    draw_list->AddLine(p1, p2, grid_color, 1.0f);
    ImVec2 p3 = worldToScreen(-grid_range * grid_size, world_coord);
    ImVec2 p4 = worldToScreen(grid_range * grid_size, world_coord);
    draw_list->AddLine(p3, p4, grid_color, 1.0f);
  }

  if (m_showCones && *m_showCones && state.cones) {
    for (const auto& cone : *state.cones) {
      ImVec2 center = worldToScreen((float)cone.x, (float)cone.y);
      float base_radius, stripe_radius, top_radius;
      ImU32 base_color, stripe_color;

      if (cone.type == scene::ConeType::BigOrange) {
        base_radius = 0.25f;
        stripe_radius = 0.16f;
        top_radius = 0.08f;
        base_color = IM_COL32(255, 140, 0, 255);
        stripe_color = IM_COL32(255, 255, 255, 255);
      } else {
        base_radius = 0.175f;
        stripe_radius = 0.1f;
        top_radius = 0.06f;

        if (cone.type == scene::ConeType::Blue) {
          base_color = IM_COL32(50, 100, 255, 255);
          stripe_color = IM_COL32(255, 255, 255, 255);
        } else if (cone.type == scene::ConeType::Yellow) {
          base_color = IM_COL32(255, 220, 0, 255);
          stripe_color = IM_COL32(50, 50, 50, 255);
        } else {
          base_color = IM_COL32(255, 140, 0, 255);
          stripe_color = IM_COL32(255, 255, 255, 255);
        }
      }

      draw_list->AddCircleFilled(center, base_radius * cam_zoom, base_color, 16);
      draw_list->AddCircleFilled(center, stripe_radius * cam_zoom, stripe_color, 12);
      draw_list->AddCircleFilled(center, top_radius * cam_zoom, base_color, 8);
    }
  }

  // Draw markers
  if (m_markerSystem) {
    struct Pose2D { double x, y, yaw; };
    Pose2D carPose{state.x, state.y, state.yaw};
    
    auto transformPoint = [&](double x, double y, int frame) -> std::pair<double, double> {
        if (frame == 1) { // CAR frame
            double cx = x * std::cos(carPose.yaw) - y * std::sin(carPose.yaw);
            double cy = x * std::sin(carPose.yaw) + y * std::cos(carPose.yaw);
            return {carPose.x + cx, carPose.y + cy};
        }
        return {x, y};
    };
    
    const auto& markers = m_markerSystem->getMarkers();
    for (const auto& [key, marker] : markers) {
        if (!m_markerSystem->isMarkerVisible(key.ns, key.id)) continue;
        if (marker.type == viz::lilsim::LINE_STRIP && marker.points.size() >= 2) {
            std::vector<ImVec2> screenPoints;
            for(const auto& pt : marker.points) {
                auto [wx, wy] = transformPoint(pt.x, pt.y, marker.frame_id);
                screenPoints.push_back(worldToScreen((float)wx, (float)wy));
            }
            ImU32 color = IM_COL32(marker.color.r, marker.color.g, marker.color.b, marker.color.a);
            for(size_t i=0; i<screenPoints.size()-1; ++i) {
                draw_list->AddLine(screenPoints[i], screenPoints[i+1], color, marker.scale.x * cam_zoom);
            }
        }
    }
  }

  if (m_showCar && *m_showCar) {
    auto& texMgr = TextureManager::getInstance();
    const auto* texData = texMgr.getTexture("pixel_x2.png");
    if (!texData) texData = texMgr.loadTexture("pixel_x2.png");

    if (texData && texData->imguiTextureID) {
      const float car_length = (float)state.wheelbase;
      const float car_width = (float)state.track_width; // Approx
      float sprite_half_length = car_length * 0.5f;
      float sprite_half_width = car_width * 0.5f;
      
      float cos_yaw = std::cos(car_yaw);
      float sin_yaw = std::sin(car_yaw);

      float corners_local[4][2] = {
        { sprite_half_length, -sprite_half_width},
        { sprite_half_length,  sprite_half_width},
        {-sprite_half_length,  sprite_half_width},
        {-sprite_half_length, -sprite_half_width}
      };

      ImVec2 corners_screen[4];
      for (int i = 0; i < 4; ++i) {
        float local_x = corners_local[i][0];
        float local_y = corners_local[i][1];
        float world_x = car_x + local_x * cos_yaw - local_y * sin_yaw;
        float world_y = car_y + local_x * sin_yaw + local_y * cos_yaw;
        corners_screen[i] = worldToScreen(world_x, world_y);
      }

      draw_list->AddImageQuad(texData->imguiTextureID,
        corners_screen[1], corners_screen[0], corners_screen[3], corners_screen[2],
        ImVec2(0,0), ImVec2(1,0), ImVec2(1,1), ImVec2(0,1));
    }
  }
  
  // HUD
  const float hud_margin_right = 10.0f;
  const float hud_margin_bottom = 15.0f;
  const float gap = 10.0f;
  const float bar_width = 30.0f;
  const float bar_height = 240.0f;
  
  float current_x = x + width - hud_margin_right;
  float current_y = y + height - hud_margin_bottom;

  // 1. Acceleration Bar (AX)
  if (state.ax.has_value()) {
      float val = (float)state.ax.value();
      float limit = 10.0f; // Fixed visual limit as per request? User said "if not defined ... hidden". 
                           // "ax_max" input limit might be different. 
                           // User defined state range -10 to 10 in macros.h.
      
      float bar_center_x = current_x - bar_width * 0.5f;
      float bar_top_y = current_y - bar_height;
      
      ImVec2 bar_min(bar_center_x - bar_width * 0.5f, bar_top_y);
      ImVec2 bar_max(bar_center_x + bar_width * 0.5f, current_y);
      draw_list->AddRect(bar_min, bar_max, IM_COL32(60, 60, 60, 255), 0.0f, 0, 2.0f);
      
      if (std::abs(val) > 0.001f) {
          float fill_fraction = std::clamp(std::abs(val) / limit, 0.0f, 1.0f);
          float fill_height = bar_height * fill_fraction;
          
          if (val > 0) {
              draw_list->AddRectFilledMultiColor(
                ImVec2(bar_min.x + 2, current_y - fill_height),
                ImVec2(bar_max.x - 2, current_y - 2),
                IM_COL32(255,0,0,200), IM_COL32(255,0,0,200), IM_COL32(255,255,0,200), IM_COL32(255,255,0,200));
          } else {
              draw_list->AddRectFilledMultiColor(
                ImVec2(bar_min.x + 2, current_y - fill_height),
                ImVec2(bar_max.x - 2, current_y - 2),
                IM_COL32(0,100,255,200), IM_COL32(0,100,255,200), IM_COL32(0,255,0,200), IM_COL32(0,255,0,200));
          }
      }
      
      current_x -= (bar_width + gap);
  }

  // 2. Steering Wheel
  if (state.steering_wheel_angle.has_value()) {
      auto& texMgr = TextureManager::getInstance();
      const auto* wheelTex = texMgr.getTexture("steering_wheel.png");
      if (!wheelTex) wheelTex = texMgr.loadTexture("steering_wheel.png");
      
      if (wheelTex && wheelTex->imguiTextureID) {
          float aspect = (float)wheelTex->width / (float)wheelTex->height;
          float h = 100.0f;
          float w = h * aspect;
          
          float centerX = current_x - w * 0.5f;
          float centerY = current_y - 30.0f - 30.0f - h * 0.5f; // leave space for rate bar
          
          float delta = -(float)state.steering_wheel_angle.value();
          float cos_d = std::cos(delta);
          float sin_d = std::sin(delta);
          
          ImVec2 corners[4];
          float hw = w * 0.5f;
          float hh = h * 0.5f;
          
          float lc[4][2] = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
          for(int i=0; i<4; ++i) {
              float lx = lc[i][0];
              float ly = lc[i][1];
              float rx = lx * cos_d - ly * sin_d;
              float ry = lx * sin_d + ly * cos_d;
              corners[i] = ImVec2(centerX + rx, centerY + ry);
          }
          
          draw_list->AddImageQuad(
              wheelTex->imguiTextureID,
              corners[0], corners[1], corners[2], corners[3],
              ImVec2(0,0), ImVec2(1,0), ImVec2(1,1), ImVec2(0,1));
          
          // Rate Bar (below wheel)
          if (state.steering_wheel_rate.has_value()) {
              float rate = (float)state.steering_wheel_rate.value();
              float limit = 6.0f; // match state range
              
              float bw = w;
              float bh = 30.0f;
              float by = current_y - bh;
              float bx = centerX;
              
              ImVec2 rmin(bx - bw*0.5f, by - bh);
              ImVec2 rmax(bx + bw*0.5f, by);
              draw_list->AddRect(rmin, rmax, IM_COL32(60, 60, 60, 255));
              
              if (std::abs(rate) > 0.001f) {
                  float fill = std::clamp(std::abs(rate)/limit, 0.0f, 1.0f);
                  float fw = (bw*0.5f - 2.0f) * fill;
                  
                  if (rate > 0) { // Left turn rate? Positive usually left
                      draw_list->AddRectFilled(ImVec2(bx - fw, by - bh + 2), ImVec2(bx, by - 2), IM_COL32(200,100,255,200));
                  } else {
                      draw_list->AddRectFilled(ImVec2(bx, by - bh + 2), ImVec2(bx + fw, by - 2), IM_COL32(200,100,255,200));
                  }
              }
          }
          
          current_x -= (w + gap);
      }
  }

  ImGui::End();
  ImGui::PopStyleVar();
}

void ViewportPanel::handleInput(GLFWwindow* window, const RenderState& state) {
  static bool toggleKeyWasPressed = false;
  bool toggleKeyPressed = glfwGetKey(window, gKeyBindings.toggleCameraMode) == GLFW_PRESS;
  if (toggleKeyPressed && !toggleKeyWasPressed) {
    if (cameraMode == CameraMode::CarFollow) {
      if (!m_freeCameraInitialized) {
        freeCameraX = (float)state.x;
        freeCameraY = (float)state.y;
        m_freeCameraInitialized = true;
      }
      cameraMode = CameraMode::Free;
    } else {
      cameraMode = CameraMode::CarFollow;
    }
  }
  toggleKeyWasPressed = toggleKeyPressed;

  if (cameraMode == CameraMode::Free) {
    bool mousePressed = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    if (mousePressed && m_mouseLeftPressed) {
      float dx = (float)(mouseX - m_lastMouseX);
      float dy = (float)(mouseY - m_lastMouseY);
      freeCameraY += dx / freeCameraZoom;
      freeCameraX += dy / freeCameraZoom;
    }

    m_mouseLeftPressed = mousePressed;
    m_lastMouseX = (float)mouseX;
    m_lastMouseY = (float)mouseY;
  }
}

} // namespace viz
