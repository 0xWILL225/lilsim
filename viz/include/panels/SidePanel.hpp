#pragma once

#include <imgui.h>
#include <functional>
#include <string>
#include <vector>

namespace viz {

/**
 * @brief A collapsible section within a side panel.
 *
 * Each section has a header that can be clicked to expand/collapse,
 * and a draw function that renders the content when expanded.
 */
struct PanelSection {
  std::string name;
  bool expanded = true;
  std::function<void()> drawContent;

  PanelSection(const std::string& name_, std::function<void()> drawFunc)
    : name(name_)
    , drawContent(drawFunc) {
  }
};

/**
 * @brief A resizable and collapsible side panel (left or right).
 *
 * Features:
 * - Can be positioned on left or right side of window
 * - Resizable by dragging edge
 * - Collapsible to thin strip with expand button
 * - Contains multiple collapsible sections
 */
class SidePanel {
public:
  enum class Side { Left, Right };

  SidePanel(const std::string& name, Side side, float defaultWidth = 300.0f)
    : m_name(name)
    , m_side(side)
    , m_width(defaultWidth)
    , m_defaultWidth(defaultWidth) {
  }

  /**
   * @brief Draws the panel and all its sections.
   *
   * @param windowWidth Total window width in pixels
   * @param windowHeight Total window height in pixels
   */
  void draw(int windowWidth, int windowHeight);

  /**
   * @brief Adds a collapsible section to the panel.
   *
   * @param name Section name (shown in header)
   * @param drawFunc Function to call to render section content
   */
  void addSection(const std::string& name, std::function<void()> drawFunc) {
    m_sections.emplace_back(name, drawFunc);
  }

  /**
   * @brief Returns the current width of the panel.
   *
   * This is the collapsed width if collapsed, otherwise the full width.
   */
  float getWidth() const {
    return m_collapsed ? m_collapsedWidth : m_width;
  }

  bool isCollapsed() const {
    return m_collapsed;
  }

  /**
   * @brief Returns true if the mouse is currently hovering over this panel.
   */
  bool isHovered() const {
    return m_isHovered;
  }

private:
  std::string m_name;
  Side m_side;
  float m_width;
  float m_defaultWidth;
  bool m_collapsed = false;
  bool m_resizing = false;
  bool m_isHovered = false;
  const float m_collapsedWidth = 30.0f;
  const float m_minWidth = 150.0f;
  const float m_maxWidth = 600.0f;

  std::vector<PanelSection> m_sections;
};

} // namespace viz



