#include "panels/SidePanel.hpp"
#include <GLFW/glfw3.h>
#include <algorithm>

namespace viz {

void SidePanel::draw(float window_width, float window_height) {
  const float current_width = getWidth();

  // Position based on side
  float xPos = (m_side == Side::Left) ? 0.0f : (window_width - current_width);

  float clamped_height = std::max(0.0f, window_height - m_top_margin);
  ImGui::SetNextWindowPos(ImVec2(xPos, m_top_margin));
  ImGui::SetNextWindowSize(ImVec2(current_width, clamped_height));

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
                           | ImGuiWindowFlags_NoTitleBar;

  ImGui::Begin(m_name.c_str(), nullptr, flags);

  // Check if mouse is hovering over this window
  m_isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);

  if (m_collapsed) {
    // Show expand button when collapsed
    const char* button_label = (m_side == Side::Left) ? ">" : "<";
    if (ImGui::Button(button_label, ImVec2(20, 30))) {
      m_collapsed = false;
    }
  } else {
    // Collapse button and header
    const char* collapse_label = (m_side == Side::Left) ? "<" : ">";
    if (ImGui::Button(collapse_label, ImVec2(20, 20))) {
      m_collapsed = true;
    }
    ImGui::SameLine();
    ImGui::Text("%s", m_name.c_str());

    ImGui::Separator();

    // Draw sections
    for (auto& section : m_sections) {
      if (ImGui::CollapsingHeader(section.name.c_str(),
                                  section.expanded ? ImGuiTreeNodeFlags_DefaultOpen
                                                   : 0)) {
        section.expanded = true;
        section.drawContent();
      } else {
        section.expanded = false;
      }
    }
  }

  // Handle resizing (manual check for mouse at edge)
  if (!m_collapsed) {
    ImVec2 mouse_pos = ImGui::GetMousePos();
    float edge_x = (m_side == Side::Left) ? (xPos + current_width) : xPos;
    float dist_to_edge = std::abs(mouse_pos.x - edge_x);
    
    // Check if mouse is near the resize edge
    if (dist_to_edge < 5.0f && mouse_pos.y >= 0 && mouse_pos.y <= window_height) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      
      if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        m_resizing = true;
      }
    }
    
    if (m_resizing) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        float mouseX = mouse_pos.x;
        
        if (m_side == Side::Left) {
          m_width = std::clamp(mouseX - xPos, m_minWidth, m_maxWidth);
        } else {
          m_width = std::clamp((window_width - mouseX), m_minWidth,
                               m_maxWidth);
        }
      } else {
        m_resizing = false;
      }
    }
  }

  ImGui::End();
}

/**
 * @brief Set the stored (uncollapsed) width of the panel, clamped to allowed bounds.
 */
void SidePanel::setContentWidth(float width) {
  m_width = std::clamp(width, m_minWidth, m_maxWidth);
}

/**
 * @brief Force the collapsed state to a specific value.
 */
void SidePanel::setCollapsed(bool collapsed) {
  m_collapsed = collapsed;
  if (!m_collapsed) {
    m_width = std::clamp(m_width, m_minWidth, m_maxWidth);
  }
}

void SidePanel::setTopMargin(float margin) {
  m_top_margin = std::max(0.0f, margin);
}

} // namespace viz



