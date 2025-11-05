#include "panels/SidePanel.hpp"
#include <GLFW/glfw3.h>
#include <algorithm>

namespace viz {

void SidePanel::draw(int windowWidth, int windowHeight) {
  const float currentWidth = getWidth();

  // Position based on side
  float xPos = (m_side == Side::Left) ? 0.0f : (windowWidth - currentWidth);

  ImGui::SetNextWindowPos(ImVec2(xPos, 0));
  ImGui::SetNextWindowSize(ImVec2(currentWidth, (float)windowHeight));

  ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
                           | ImGuiWindowFlags_NoTitleBar;

  ImGui::Begin(m_name.c_str(), nullptr, flags);

  // Check if mouse is hovering over this window
  m_isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);

  if (m_collapsed) {
    // Show expand button when collapsed
    const char* buttonLabel = (m_side == Side::Left) ? ">" : "<";
    if (ImGui::Button(buttonLabel, ImVec2(20, 30))) {
      m_collapsed = false;
    }
  } else {
    // Resize handle (invisible button on edge)
    float resizeHandleX = (m_side == Side::Left) ? (currentWidth - 5.0f) : 0.0f;
    ImGui::SetCursorPos(ImVec2(resizeHandleX, 0));
    ImGui::InvisibleButton("##resize", ImVec2(5, (float)windowHeight));

    if (ImGui::IsItemHovered()) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    if (ImGui::IsItemActive()) {
      m_resizing = true;
    }

    if (m_resizing) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        // Use ImGui's mouse position instead of GLFW to avoid null window issues
        ImVec2 mousePos = ImGui::GetMousePos();
        float mouseX = mousePos.x;

        if (m_side == Side::Left) {
          m_width = std::clamp(mouseX, m_minWidth, m_maxWidth);
        } else {
          m_width = std::clamp((float)(windowWidth - mouseX), m_minWidth,
                               m_maxWidth);
        }
      } else {
        m_resizing = false;
      }
    }

    // Collapse button and header
    ImGui::SetCursorPos(ImVec2(10, 5));
    const char* collapseLabel = (m_side == Side::Left) ? "<" : ">";
    if (ImGui::Button(collapseLabel, ImVec2(20, 20))) {
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

  ImGui::End();
}

} // namespace viz



