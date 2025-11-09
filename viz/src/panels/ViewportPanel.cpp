#include "panels/ViewportPanel.hpp"
#include "KeyBindings.hpp"
#include "MarkerSystem.hpp"
#include "TextureManager.hpp"
#include "scene.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
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
    
    // Helper to transform a position based on frame_id
    auto transformPosition = [&](const common::Position& pos, FrameId frame) -> common::Position {
      if (frame == lilsim::CAR) {
        // Transform from car frame to world frame
        Eigen::Vector2d localPos(pos.x, pos.y);
        Eigen::Vector2d worldPos = car.pose.T * localPos;
        return common::Position(worldPos.x(), worldPos.y());
      }
      // Already in world frame
      return pos;
    };
    
    // Helper to transform a pose based on frame_id
    auto transformPose = [&](const common::SE2& pose, FrameId frame) -> common::SE2 {
      if (frame == lilsim::CAR) {
        // Transform from car frame to world frame
        return car.pose * pose;
      }
      return pose;
    };
    
    // Helper to get color for a vertex (with fallback to marker color)
    auto getVertexColor = [&](const Marker& marker, size_t idx) -> ImU32 {
      if (idx < marker.colors.size()) {
        const auto& c = marker.colors[idx];
        return IM_COL32(c.r, c.g, c.b, c.a);
      }
      return IM_COL32(marker.color.r, marker.color.g, marker.color.b, marker.color.a);
    };
    
    for (const auto& [key, marker] : markers) {
      // Check visibility
      if (!m_markerSystem->isMarkerVisible(key.ns, key.id)) {
        continue;
      }

      // Check for color count mismatches and log warnings
      if (!marker.colors.empty() && marker.colors.size() != marker.points.size()) {
        if (marker.colors.size() < marker.points.size()) {
          spdlog::warn("[Marker {}:{}] colors array size ({}) < points size ({}), using fallback color", 
                       key.ns, key.id, marker.colors.size(), marker.points.size());
        } else {
          spdlog::warn("[Marker {}:{}] colors array size ({}) > points size ({}), extra colors ignored", 
                       key.ns, key.id, marker.colors.size(), marker.points.size());
        }
      }

      // Convert marker color (fallback)
      ImU32 markerColor = IM_COL32(marker.color.r, marker.color.g, 
                                     marker.color.b, marker.color.a);

      // Render based on marker type
      switch (marker.type) {
        case lilsim::TEXT: {
          // Transform pose to world frame
          common::SE2 worldPose = transformPose(marker.pose, marker.frame_id);
          ImVec2 screenPos = worldToScreen((float)worldPose.x(), (float)worldPose.y());
          
          // Apply scale to font size
          ImGui::SetWindowFontScale(marker.scale.x);
          draw_list->AddText(screenPos, markerColor, marker.text.c_str());
          ImGui::SetWindowFontScale(1.0f);
          break;
        }

        case lilsim::ARROW: {
          // Transform pose to world frame
          common::SE2 worldPose = transformPose(marker.pose, marker.frame_id);
          float worldX = (float)worldPose.x();
          float worldY = (float)worldPose.y();
          float worldYaw = (float)worldPose.yaw();
          
          // Arrow parameters
          float length = marker.scale.x;
          float thickness = marker.scale.y;
          float headLength = length * 0.2f;  // Head is 20% of total length
          float headWidth = thickness * 3.0f;
          
          // Shaft end point
          float shaftEndX = worldX + length * std::cos(worldYaw);
          float shaftEndY = worldY + length * std::sin(worldYaw);
          
          // Draw shaft
          ImVec2 start = worldToScreen(worldX, worldY);
          ImVec2 shaftEnd = worldToScreen(shaftEndX - headLength * std::cos(worldYaw), 
                                          shaftEndY - headLength * std::sin(worldYaw));
          draw_list->AddLine(start, shaftEnd, markerColor, thickness * cam_zoom);
          
          // Draw triangular head
          ImVec2 tip = worldToScreen(shaftEndX, shaftEndY);
          float perpX = -std::sin(worldYaw);
          float perpY = std::cos(worldYaw);
          ImVec2 head1 = worldToScreen(shaftEndX - headLength * std::cos(worldYaw) + headWidth * 0.5f * perpX,
                                       shaftEndY - headLength * std::sin(worldYaw) + headWidth * 0.5f * perpY);
          ImVec2 head2 = worldToScreen(shaftEndX - headLength * std::cos(worldYaw) - headWidth * 0.5f * perpX,
                                       shaftEndY - headLength * std::sin(worldYaw) - headWidth * 0.5f * perpY);
          draw_list->AddTriangleFilled(tip, head1, head2, markerColor);
          break;
        }

        case lilsim::RECTANGLE: {
          // Transform pose to world frame
          common::SE2 worldPose = transformPose(marker.pose, marker.frame_id);
          float worldX = (float)worldPose.x();
          float worldY = (float)worldPose.y();
          float worldYaw = (float)worldPose.yaw();
          
          float halfWidth = marker.scale.x * 0.5f;
          float halfHeight = marker.scale.y * 0.5f;
          
          // Compute 4 corners in world frame
          float cosYaw = std::cos(worldYaw);
          float sinYaw = std::sin(worldYaw);
          
          ImVec2 corners[4];
          float localCorners[4][2] = {
            {-halfWidth, -halfHeight},
            { halfWidth, -halfHeight},
            { halfWidth,  halfHeight},
            {-halfWidth,  halfHeight}
          };
          
          for (int i = 0; i < 4; ++i) {
            float lx = localCorners[i][0];
            float ly = localCorners[i][1];
            float wx = worldX + lx * cosYaw - ly * sinYaw;
            float wy = worldY + lx * sinYaw + ly * cosYaw;
            corners[i] = worldToScreen(wx, wy);
          }
          
          draw_list->AddQuadFilled(corners[0], corners[1], corners[2], corners[3], markerColor);
          break;
        }

        case lilsim::CIRCLE: {
          // Transform pose to world frame
          common::SE2 worldPose = transformPose(marker.pose, marker.frame_id);
          ImVec2 center = worldToScreen((float)worldPose.x(), (float)worldPose.y());
          float radius = marker.scale.x * 0.5f * cam_zoom;  // scale.x is diameter
          draw_list->AddCircleFilled(center, radius, markerColor, 32);
          break;
        }

        case lilsim::LINE_STRIP: {
          if (marker.points.size() >= 2) {
            // Transform all points to world frame
            std::vector<ImVec2> screenPoints;
            screenPoints.reserve(marker.points.size());
            for (const auto& pt : marker.points) {
              common::Position worldPt = transformPosition(pt, marker.frame_id);
              screenPoints.push_back(worldToScreen((float)worldPt.x, (float)worldPt.y));
            }
            
            // Draw line segments with per-vertex color interpolation
            float lineWidth = marker.scale.x * cam_zoom;
            
            if (marker.colors.empty()) {
              // No per-vertex colors, use marker color for all segments
              for (size_t i = 0; i < screenPoints.size() - 1; ++i) {
                draw_list->AddLine(screenPoints[i], screenPoints[i + 1], markerColor, lineWidth);
              }
            } else {
              // Use per-vertex colors with gradient effect
              // For each segment, we'll use AddLine with the start vertex color
              // For smooth interpolation, we could use multiple segments, but for simplicity
              // we'll use a gradient approach by drawing each segment with interpolated colors
              for (size_t i = 0; i < screenPoints.size() - 1; ++i) {
                ImU32 color1 = getVertexColor(marker, i);
                ImU32 color2 = getVertexColor(marker, i + 1);
                
                // For gradient effect, use ImDrawList path stroke with gradient
                // Since ImGui doesn't support gradient lines directly, we approximate
                // by drawing multiple sub-segments with interpolated colors
                const int subSegments = 8;
                for (int j = 0; j < subSegments; ++j) {
                  float t1 = (float)j / subSegments;
                  float t2 = (float)(j + 1) / subSegments;
                  
                  ImVec2 p1 = ImVec2(
                    screenPoints[i].x + t1 * (screenPoints[i + 1].x - screenPoints[i].x),
                    screenPoints[i].y + t1 * (screenPoints[i + 1].y - screenPoints[i].y)
                  );
                  ImVec2 p2 = ImVec2(
                    screenPoints[i].x + t2 * (screenPoints[i + 1].x - screenPoints[i].x),
                    screenPoints[i].y + t2 * (screenPoints[i + 1].y - screenPoints[i].y)
                  );
                  
                  // Interpolate color
                  float t_mid = (t1 + t2) * 0.5f;
                  float r1 = static_cast<float>((color1 >> 0) & 0xFF);
                  float g1 = static_cast<float>((color1 >> 8) & 0xFF);
                  float b1 = static_cast<float>((color1 >> 16) & 0xFF);
                  float a1 = static_cast<float>((color1 >> 24) & 0xFF);
                  float r2 = static_cast<float>((color2 >> 0) & 0xFF);
                  float g2 = static_cast<float>((color2 >> 8) & 0xFF);
                  float b2 = static_cast<float>((color2 >> 16) & 0xFF);
                  float a2 = static_cast<float>((color2 >> 24) & 0xFF);
                  
                  ImU32 r = static_cast<ImU32>((1.0f - t_mid) * r1 + t_mid * r2);
                  ImU32 g = static_cast<ImU32>((1.0f - t_mid) * g1 + t_mid * g2);
                  ImU32 b = static_cast<ImU32>((1.0f - t_mid) * b1 + t_mid * b2);
                  ImU32 a = static_cast<ImU32>((1.0f - t_mid) * a1 + t_mid * a2);
                  
                  ImU32 interpColor = IM_COL32(r, g, b, a);
                  draw_list->AddLine(p1, p2, interpColor, lineWidth);
                }
              }
            }
          }
          break;
        }

        case lilsim::CIRCLE_LIST: {
          if (marker.points.size() > 0) {
            float radius = marker.scale.x * 0.5f * cam_zoom;  // scale.x is diameter
            
            for (size_t i = 0; i < marker.points.size(); ++i) {
              common::Position worldPt = transformPosition(marker.points[i], marker.frame_id);
              ImVec2 center = worldToScreen((float)worldPt.x, (float)worldPt.y);
              ImU32 circleColor = getVertexColor(marker, i);
              draw_list->AddCircleFilled(center, radius, circleColor, 32);
            }
          }
          break;
        }

        case lilsim::TRIANGLE_LIST: {
          // Every 3 points forms a triangle
          size_t numTriangles = marker.points.size() / 3;
          
          for (size_t t = 0; t < numTriangles; ++t) {
            size_t i0 = t * 3;
            size_t i1 = t * 3 + 1;
            size_t i2 = t * 3 + 2;
            
            // Transform points to world frame
            common::Position wp0 = transformPosition(marker.points[i0], marker.frame_id);
            common::Position wp1 = transformPosition(marker.points[i1], marker.frame_id);
            common::Position wp2 = transformPosition(marker.points[i2], marker.frame_id);
            
            ImVec2 sp0 = worldToScreen((float)wp0.x, (float)wp0.y);
            ImVec2 sp1 = worldToScreen((float)wp1.x, (float)wp1.y);
            ImVec2 sp2 = worldToScreen((float)wp2.x, (float)wp2.y);
            
            if (marker.colors.empty()) {
              // Solid color triangle
              draw_list->AddTriangleFilled(sp0, sp1, sp2, markerColor);
            } else {
              // Per-vertex colored triangle with interpolation
              ImU32 c0 = getVertexColor(marker, i0);
              ImU32 c1 = getVertexColor(marker, i1);
              ImU32 c2 = getVertexColor(marker, i2);
              
              // ImGui supports gradient triangles via AddTriangleFilled with per-vertex colors
              // However, the standard AddTriangleFilled doesn't support per-vertex colors
              // We need to use the lower-level vertex API
              // For now, use a simple approximation: average color
              ImU32 r = (((c0 >> 0) & 0xFF) + ((c1 >> 0) & 0xFF) + ((c2 >> 0) & 0xFF)) / 3;
              ImU32 g = (((c0 >> 8) & 0xFF) + ((c1 >> 8) & 0xFF) + ((c2 >> 8) & 0xFF)) / 3;
              ImU32 b = (((c0 >> 16) & 0xFF) + ((c1 >> 16) & 0xFF) + ((c2 >> 16) & 0xFF)) / 3;
              ImU32 a = (((c0 >> 24) & 0xFF) + ((c1 >> 24) & 0xFF) + ((c2 >> 24) & 0xFF)) / 3;
              ImU32 avgColor = IM_COL32(r, g, b, a);
              
              // TODO: Use ImDrawList vertex buffer for proper per-vertex colors
              draw_list->AddTriangleFilled(sp0, sp1, sp2, avgColor);
            }
          }
          break;
        }

        case lilsim::MESH_2D:
          // Not yet implemented
          break;

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

  const float hud_margin_right = 10.0f;
  const float hud_margin_bottom = 15.0f;
  const float gap_between_elements = 10.0f;
  const float bar_width = 30.0f;
  const float bar_height = 240.0f;
  
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
    const float wheel_height = 100.0f;
    const float wheel_width = wheel_height * aspect_ratio;  // Preserve aspect ratio
    
    // Steering bar dimensions (calculated here to position wheel above it)
    const float steering_bar_height = 30.0f;
    const float steering_bar_gap = 30.0f;             // Gap between wheel and steering bar
    
    const float wheel_center_x = x + width - hud_margin_right - gap_between_elements - bar_width - gap_between_elements - wheel_width * 0.5f;
    // Position wheel above the steering bar (which is at hud_margin_bottom)
    const float wheel_center_y = y + height - hud_margin_bottom - steering_bar_height - steering_bar_gap - wheel_height * 0.5f;
    
    // Get steering angle directly from scene snapshot (no smoothing/interpolation)
    const float delta = static_cast<float>(scene.car_input.delta);  // radians
    
    // Calculate rotated corners of steering wheel sprite
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
    
    // Draw horizontal steering bar (positioned at bottom margin)
    const float steering_bar_width = wheel_width;     // Match steering wheel width
    
    const float steering_bar_center_x = wheel_center_x;
    const float steering_bar_bottom_y = y + height - hud_margin_bottom;
    const float steering_bar_top_y = steering_bar_bottom_y - steering_bar_height;
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



