#include "panels/ViewportPanel.hpp"
#include "KeyBindings.hpp"
#include "MarkerSystem.hpp"
#include "TextureManager.hpp"
#include "scene.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include "ImGuiFileDialog/stb/stb_image.h"

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

  auto markerColorToU32 = [](const Color& color) -> ImU32 {
      return IM_COL32(color.r, color.g, color.b, color.a);
  };

  auto drawCircleMarker = [&](double wx, double wy, float diameterMeters, const Color& color) {
      if (diameterMeters <= 0.0f) {
          return;
      }
      ImVec2 center = worldToScreen((float)wx, (float)wy);
      float radiusPixels = 0.5f * diameterMeters * cam_zoom;
      if (radiusPixels <= 0.0f) {
          return;
      }
      draw_list->AddCircleFilled(center, radiusPixels, markerColorToU32(color), 32);
  };

  // Draw markers

  struct CarSpriteParams {
      float x{0.0f};
      float y{0.0f};
      float yaw{0.0f};
      float wheelbase{1.0f};
      float trackWidth{1.0f};
      std::optional<double> wheel_fl_angle;
      std::optional<double> wheel_fr_angle;
      std::optional<double> steering_angle;
      float opacity{1.0f};
      ImVec4 tintColor{1.0f, 1.0f, 1.0f, 1.0f};
      float tintOpacity{0.0f};
      double simTime{0.0};
      bool drawTsal{false};
  };

  auto drawFallbackCar = [&](const CarSpriteParams& params) {
      auto& texMgr = TextureManager::getInstance();
      const auto* texData = texMgr.getTexture("pixel_x2.png");
      if (!texData) texData = texMgr.loadTexture("pixel_x2.png");
      if (!texData || !texData->imguiTextureID) return;

      float car_length = std::max(params.wheelbase, 1.0f);
      float car_width = std::max(params.trackWidth, 0.5f);
      float sprite_half_length = car_length * 0.5f;
      float sprite_half_width = car_width * 0.5f;

      float cosCarYaw = std::cos(params.yaw);
      float sinCarYaw = std::sin(params.yaw);

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
          float world_x = params.x + local_x * cosCarYaw - local_y * sinCarYaw;
          float world_y = params.y + local_x * sinCarYaw + local_y * cosCarYaw;
          corners_screen[i] = worldToScreen(world_x, world_y);
      }

      auto baseAlpha = std::clamp(params.opacity, 0.0f, 1.0f);
      ImU32 baseColor = IM_COL32(120, 160, 255, (int)(baseAlpha * 255.0f));
      draw_list->AddImageQuad(texData->imguiTextureID,
                              corners_screen[1], corners_screen[0], corners_screen[3], corners_screen[2],
                              ImVec2(0,0), ImVec2(1,0), ImVec2(1,1), ImVec2(0,1), baseColor);

      if (params.tintOpacity > 0.0f) {
          ImVec4 tint = params.tintColor;
          tint.w = std::clamp(params.tintOpacity * params.opacity, 0.0f, 1.0f);
          ImU32 tintColor = ImGui::GetColorU32(tint);
          draw_list->AddImageQuad(texData->imguiTextureID,
                                  corners_screen[1], corners_screen[0], corners_screen[3], corners_screen[2],
                                  ImVec2(0,0), ImVec2(1,0), ImVec2(1,1), ImVec2(0,1), tintColor);
      }
  };

  auto drawCarSprite = [&](const CarSpriteParams& params) -> bool {
      if (!ensureCarVisualAssets()) {
          return false;
      }

      const auto& car = m_carVisual;
      if (!car.chassis || !car.overlay || !car.tire || !car.tsalRed) {
          return false;
      }

      double wheelbaseNorm = (car.wheelbaseNorm > 1e-6) ? car.wheelbaseNorm : 1.0;
      double trackWidthNorm = (car.trackWidthNorm > 1e-6) ? car.trackWidthNorm : 1.0;
      double wheelbaseMeters = params.wheelbase > 1e-6f ? params.wheelbase : 1.0f;
      double trackWidthMeters = params.trackWidth > 1e-6f ? params.trackWidth : 1.0f;

      float cosCarYaw = std::cos(params.yaw);
      float sinCarYaw = std::sin(params.yaw);

      float scaleX = (float)(wheelbaseMeters / wheelbaseNorm);
      float scaleY = (float)(trackWidthMeters / trackWidthNorm);

      auto localToScreen = [&](float lx, float ly) -> ImVec2 {
          float world_x = params.x + lx * cosCarYaw - ly * sinCarYaw;
          float world_y = params.y + lx * sinCarYaw + ly * cosCarYaw;
          return worldToScreen(world_x, world_y);
      };

      auto normalizedToLocal = [&](double nx, double ny) -> ImVec2 {
          double dx = nx - car.blueXNorm;
          double dy = car.blueYNorm - ny;
          return ImVec2((float)(dy * scaleX), (float)(-dx * scaleY));
      };

      auto normalizedToScreen = [&](double nx, double ny) -> ImVec2 {
          ImVec2 local = normalizedToLocal(nx, ny);
          return localToScreen(local.x, local.y);
      };

      auto makeLayerColor = [&](float alphaScale = 1.0f) -> ImU32 {
          float a = std::clamp(params.opacity * alphaScale, 0.0f, 1.0f);
          return IM_COL32(255, 255, 255, (int)(a * 255.0f));
      };

      auto drawLayer = [&](const TextureManager::TextureData* tex,
                           double left, double top, double right, double bottom,
                           ImU32 color) {
          if (!tex || !tex->imguiTextureID) return;
          double u0 = left;
          double v0 = top;
          double u1 = right;
          double v1 = bottom;
          constexpr double uvInset = 0.5;
          if (tex->width > 0 && tex->height > 0) {
              double du = uvInset / tex->width;
              double dv = uvInset / tex->height;
              u0 += du;
              u1 -= du;
              v0 += dv;
              v1 -= dv;
          }
          ImVec2 tl = normalizedToScreen(left, top);
          ImVec2 tr = normalizedToScreen(right, top);
          ImVec2 br = normalizedToScreen(right, bottom);
          ImVec2 bl = normalizedToScreen(left, bottom);
          draw_list->AddImageQuad(tex->imguiTextureID, tr, tl, bl, br,
                                  ImVec2((float)u1, (float)v0),
                                  ImVec2((float)u0, (float)v0),
                                  ImVec2((float)u0, (float)v1),
                                  ImVec2((float)u1, (float)v1),
                                  color);
      };

      auto drawFullLayer = [&](const TextureManager::TextureData* tex, ImU32 color) {
          drawLayer(tex, 0.0, 0.0, 1.0, 1.0, color);
      };

      ImU32 layerColor = makeLayerColor();
      drawFullLayer(car.chassis, layerColor);

      double chassisWidth = std::max(car.baseWidth, 1);
      double chassisHeight = std::max(car.baseHeight, 1);
      double tireWidthNorm = (double)car.tireWidth / chassisWidth;
      double tireHeightNorm = (double)car.tireHeight / chassisHeight;
      float wheelHalfForward = (float)(0.5 * tireHeightNorm * scaleX);
      float wheelHalfSide = (float)(0.5 * tireWidthNorm * scaleY);
      constexpr float wheelInsetScale = 0.94f;
      wheelHalfForward *= wheelInsetScale;
      wheelHalfSide *= wheelInsetScale;

      auto drawWheel = [&](const TextureManager::TextureData* tex,
                           const ImVec2& center,
                           double angle,
                           ImU32 color) {
          if (!tex || !tex->imguiTextureID) return;
          ImVec2 cornersLocal[4] = {
              ImVec2( wheelHalfForward, -wheelHalfSide),
              ImVec2( wheelHalfForward,  wheelHalfSide),
              ImVec2(-wheelHalfForward,  wheelHalfSide),
              ImVec2(-wheelHalfForward, -wheelHalfSide)
          };
          float cosA = std::cos((float)angle);
          float sinA = std::sin((float)angle);
          ImVec2 screen[4];
          for (int i = 0; i < 4; ++i) {
              float rx = cornersLocal[i].x * cosA - cornersLocal[i].y * sinA;
              float ry = cornersLocal[i].x * sinA + cornersLocal[i].y * cosA;
              screen[i] = localToScreen(center.x + rx, center.y + ry);
          }
          draw_list->AddImageQuad(tex->imguiTextureID,
                                  screen[1], screen[0], screen[3], screen[2],
                                  ImVec2(1,0), ImVec2(0,0), ImVec2(0,1), ImVec2(1,1),
                                  color);
      };

      ImVec2 frontLeftCenter = normalizedToLocal(car.redXNorm, car.redYNorm);
      ImVec2 frontRightCenter = normalizedToLocal(car.greenXNorm, car.greenYNorm);
      double fallbackAngle = params.steering_angle.value_or(0.0);
      double frontLeftAngle = params.wheel_fl_angle.value_or(fallbackAngle);
      double frontRightAngle = params.wheel_fr_angle.value_or(fallbackAngle);

      drawWheel(car.tire, frontLeftCenter, frontLeftAngle, layerColor);
      drawWheel(car.tire, frontRightCenter, frontRightAngle, layerColor);

      drawFullLayer(car.overlay, layerColor);

      if (params.tintOpacity > 0.0f) {
          ImVec4 tint = params.tintColor;
          tint.w = std::clamp(params.tintOpacity * params.opacity, 0.0f, 1.0f);
          ImU32 tintColor = ImGui::GetColorU32(tint);
          drawFullLayer(car.overlay, tintColor);
      }

      if (params.drawTsal) {
          double phase = std::fmod(std::max(params.simTime, 0.0), 1.0);
          if (phase < 0.5) {
              drawFullLayer(car.tsalRed, layerColor);
          }
      }

      return true;
  };

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
        if (!m_markerSystem->isMarkerVisible(key.ns, key.id)) {
            continue;
        }
        if (marker.type == viz::lilsim::CAR_SPRITE && marker.car.has_value()) {
            CarSpriteParams params;
            params.x = (float)marker.pose.x();
            params.y = (float)marker.pose.y();
            params.yaw = (float)marker.pose.yaw();
            params.wheelbase = (float)std::max(marker.car->wheelbase, 0.1);
            params.trackWidth = (float)std::max(marker.car->track_width, 0.1);
            if (marker.car->has_wheel_fl_angle) {
                params.wheel_fl_angle = marker.car->wheel_fl_angle;
            }
            if (marker.car->has_wheel_fr_angle) {
                params.wheel_fr_angle = marker.car->wheel_fr_angle;
            }
            params.steering_angle = std::nullopt;
            params.opacity = (float)std::clamp(marker.car->opacity, 0.0, 1.0);
            params.tintOpacity = (float)std::clamp(marker.car->tint_opacity, 0.0, 1.0);
            if (params.tintOpacity > 0.0f) {
                params.tintColor = ImVec4(
                    marker.color.r / 255.0f,
                    marker.color.g / 255.0f,
                    marker.color.b / 255.0f,
                    marker.color.a / 255.0f);
            } else {
                params.tintColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
            }
            params.simTime = state.sim_time;
            params.drawTsal = false;
            if (!drawCarSprite(params)) {
                drawFallbackCar(params);
            }
            continue;
        }
        if (marker.type == viz::lilsim::CIRCLE) {
            auto [wx, wy] = transformPoint(marker.pose.x(), marker.pose.y(), marker.frame_id);
            float diameter = marker.scale.x > 0.0f ? marker.scale.x : marker.scale.y;
            drawCircleMarker(wx, wy, diameter, marker.color);
            continue;
        }
        if (marker.type == viz::lilsim::CIRCLE_LIST && !marker.points.empty()) {
            float diameter = marker.scale.x > 0.0f ? marker.scale.x : marker.scale.y;
            if (diameter <= 0.0f) {
                continue;
            }
            for (size_t i = 0; i < marker.points.size(); ++i) {
                const auto& pt = marker.points[i];
                auto [wx, wy] = transformPoint(pt.x, pt.y, marker.frame_id);
                Color color = marker.color;
                if (!marker.colors.empty()) {
                    color = marker.colors[std::min(i, marker.colors.size() - 1)];
                }
                drawCircleMarker(wx, wy, diameter, color);
            }
            continue;
        }
        if (marker.type == viz::lilsim::LINE_STRIP && marker.points.size() >= 2) {
            std::vector<ImVec2> screenPoints;
            screenPoints.reserve(marker.points.size());
            for (const auto& pt : marker.points) {
                auto [wx, wy] = transformPoint(pt.x, pt.y, marker.frame_id);
                screenPoints.push_back(worldToScreen((float)wx, (float)wy));
            }

            auto colorAt = [&](size_t idx) -> ImU32 {
                if (!marker.colors.empty()) {
                    const auto& c = marker.colors[std::min(idx, marker.colors.size() - 1)];
                    return IM_COL32(c.r, c.g, c.b, c.a);
                }
                return IM_COL32(marker.color.r, marker.color.g, marker.color.b, marker.color.a);
            };

            const float lineWidth = marker.scale.x * cam_zoom;
            for (size_t i = 0; i + 1 < screenPoints.size(); ++i) {
                ImU32 segColor = colorAt(i);
                draw_list->AddLine(screenPoints[i], screenPoints[i + 1], segColor, lineWidth);
            }
        }
    }
  }

  if (m_showCar && *m_showCar) {
      CarSpriteParams params;
      params.x = car_x;
      params.y = car_y;
      params.yaw = car_yaw;
      params.wheelbase = (float)state.wheelbase;
      params.trackWidth = (float)state.track_width;
      params.wheel_fl_angle = state.wheel_fl_angle;
      params.wheel_fr_angle = state.wheel_fr_angle;
      params.steering_angle = state.steering_wheel_angle;
      params.opacity = 1.0f;
      params.tintColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
      params.tintOpacity = 0.0f;
      params.simTime = state.sim_time;
      params.drawTsal = true;
      if (!drawCarSprite(params)) {
          drawFallbackCar(params);
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
                           // User defined state range -10 to 10 in macros.hpp.
      
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
      constexpr float kSteeringWheelHeightPx = 100.0f;
      constexpr float kWheelVerticalSpacingPx = 30.0f;
      constexpr float kRateBarHeightPx = 30.0f;
      constexpr float kRateBarPaddingPx = 2.0f;
      constexpr float kSteeringRateLimitRadPerSec = 6.0f;
      const ImU32 rateBarFillColor = IM_COL32(200, 100, 255, 200);
      const ImU32 rateBarFrameColor = IM_COL32(60, 60, 60, 255);

      auto& texMgr = TextureManager::getInstance();
      const auto* wheelTex = texMgr.getTexture("steering_wheel.png");
      if (!wheelTex) wheelTex = texMgr.loadTexture("steering_wheel.png");
      
      if (wheelTex && wheelTex->imguiTextureID) {
          const float textureAspect = static_cast<float>(wheelTex->width) / static_cast<float>(wheelTex->height);
          const float wheelHeightPx = kSteeringWheelHeightPx;
          const float wheelWidthPx = wheelHeightPx * textureAspect;
          
          const float centerX = current_x - wheelWidthPx * 0.5f;
          const float centerY = current_y - (kWheelVerticalSpacingPx * 2.0f) - (wheelHeightPx * 0.5f); // leave space for rate bar
          
          const float wheelAngleRadians = -(float)state.steering_wheel_angle.value();
          const float cosAngle = std::cos(wheelAngleRadians);
          const float sinAngle = std::sin(wheelAngleRadians);
          
          ImVec2 corners[4];
          const float halfWidthPx = wheelWidthPx * 0.5f;
          const float halfHeightPx = wheelHeightPx * 0.5f;
          
          float localCorners[4][2] = {{-halfWidthPx, -halfHeightPx}, {halfWidthPx, -halfHeightPx}, {halfWidthPx, halfHeightPx}, {-halfWidthPx, halfHeightPx}};
          for(int i=0; i<4; ++i) {
              const float localX = localCorners[i][0];
              const float localY = localCorners[i][1];
              const float rotatedX = localX * cosAngle - localY * sinAngle;
              const float rotatedY = localX * sinAngle + localY * cosAngle;
              corners[i] = ImVec2(centerX + rotatedX, centerY + rotatedY);
          }
          
          draw_list->AddImageQuad(
              wheelTex->imguiTextureID,
              corners[0], corners[1], corners[2], corners[3],
              ImVec2(0,0), ImVec2(1,0), ImVec2(1,1), ImVec2(0,1));
          
          // Rate Bar (below wheel)
          if (state.steering_wheel_rate.has_value()) {
              const float rateRadPerSec = static_cast<float>(state.steering_wheel_rate.value());

              const float barWidthPx = wheelWidthPx;
              const float barHeightPx = kRateBarHeightPx;
              const float barBottomY = current_y;
              const float barCenterX = centerX;
              
              ImVec2 barMin(barCenterX - barWidthPx * 0.5f, barBottomY - barHeightPx);
              ImVec2 barMax(barCenterX + barWidthPx * 0.5f, barBottomY);
              draw_list->AddRect(barMin, barMax, rateBarFrameColor);
              
              if (std::abs(rateRadPerSec) > 0.001f) {
                  const float fillRatio = std::clamp(std::abs(rateRadPerSec) / kSteeringRateLimitRadPerSec, 0.0f, 1.0f);
                  const float filledHalfWidth = (barWidthPx * 0.5f - kRateBarPaddingPx) * fillRatio;
                  
                  if (rateRadPerSec > 0) { // Left turn rate? Positive usually left
                      draw_list->AddRectFilled(ImVec2(barCenterX - filledHalfWidth, barBottomY - barHeightPx + kRateBarPaddingPx),
                                               ImVec2(barCenterX, barBottomY - kRateBarPaddingPx),
                                               rateBarFillColor);
                  } else {
                      draw_list->AddRectFilled(ImVec2(barCenterX, barBottomY - barHeightPx + kRateBarPaddingPx),
                                               ImVec2(barCenterX + filledHalfWidth, barBottomY - kRateBarPaddingPx),
                                               rateBarFillColor);
                  }
              }
          }
          
          current_x -= (wheelWidthPx + gap);
      }
  }

  ImGui::End();
  ImGui::PopStyleVar();
}

bool ViewportPanel::ensureCarVisualAssets() {
    if (m_carVisualLoaded) {
        return true;
    }

    auto& texMgr = TextureManager::getInstance();
    constexpr int kUpscale = 4;
    const std::array<uint8_t,3> fillColor{10, 10, 10};
    m_carVisual.chassis = texMgr.loadTexture("cars/x2/chassis.png", kUpscale, fillColor);
    m_carVisual.overlay = texMgr.loadTexture("cars/x2/overlay.png", kUpscale, fillColor);
    m_carVisual.tire = texMgr.loadTexture("cars/x2/tire.png", kUpscale, fillColor);
    m_carVisual.tsalRed = texMgr.loadTexture("cars/x2/tsal_red.png", kUpscale, fillColor);

    if (!m_carVisual.chassis || !m_carVisual.overlay ||
        !m_carVisual.tire || !m_carVisual.tsalRed) {
        spdlog::error("[ViewportPanel] Failed to load car sprite textures");
        return false;
    }

    m_carVisual.baseWidth = m_carVisual.chassis->width;
    m_carVisual.baseHeight = m_carVisual.chassis->height;
    m_carVisual.tireWidth = m_carVisual.tire->width;
    m_carVisual.tireHeight = m_carVisual.tire->height;

    if (!loadReferencePoints("cars/x2/points.png")) {
        return false;
    }

    m_carVisualLoaded = true;
    return true;
}

bool ViewportPanel::loadReferencePoints(const std::string& assetPath) {
    auto& texMgr = TextureManager::getInstance();
    std::filesystem::path fullPath = texMgr.resolveAssetPath(assetPath);

    int width = 0;
    int height = 0;
    int channels = 0;
    unsigned char* data = stbi_load(fullPath.string().c_str(), &width, &height, &channels, 4);
    if (!data) {
        spdlog::error("[ViewportPanel] Failed to load reference points image: {}", fullPath.string());
        return false;
    }

    auto findColor = [&](unsigned char r, unsigned char g, unsigned char b, double& outX, double& outY) -> bool {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                unsigned char* px = data + (y * width + x) * 4;
                if (px[0] == r && px[1] == g && px[2] == b && px[3] > 0) {
                    outX = (x + 0.5) / static_cast<double>(width);
                    outY = (y + 0.5) / static_cast<double>(height);
                    return true;
                }
            }
        }
        return false;
    };

    double redX = 0.0, redY = 0.0;
    double greenX = 0.0, greenY = 0.0;
    double blueX = 0.0, blueY = 0.0;

    bool foundRed = findColor(255, 0, 0, redX, redY);
    bool foundGreen = findColor(0, 255, 0, greenX, greenY);
    bool foundBlue = findColor(0, 0, 255, blueX, blueY);

    stbi_image_free(data);

    if (!foundRed || !foundGreen || !foundBlue) {
        spdlog::error("[ViewportPanel] Missing required reference pixels in {}", fullPath.string());
        return false;
    }

    m_carVisual.redXNorm = redX;
    m_carVisual.redYNorm = redY;
    m_carVisual.greenXNorm = greenX;
    m_carVisual.greenYNorm = greenY;
    m_carVisual.blueXNorm = blueX;
    m_carVisual.blueYNorm = blueY;

    m_carVisual.wheelbaseNorm = std::abs(m_carVisual.blueYNorm - m_carVisual.redYNorm);
    m_carVisual.trackWidthNorm = std::abs(m_carVisual.redXNorm - m_carVisual.greenXNorm);

    if (m_carVisual.wheelbaseNorm <= 1e-6) {
        spdlog::warn("[ViewportPanel] Wheelbase reference degenerates, defaulting to 1.0");
        m_carVisual.wheelbaseNorm = 1.0;
    }
    if (m_carVisual.trackWidthNorm <= 1e-6) {
        spdlog::warn("[ViewportPanel] Track width reference degenerates, defaulting to 1.0");
        m_carVisual.trackWidthNorm = 1.0;
    }

    return true;
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
