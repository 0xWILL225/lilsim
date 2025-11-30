#pragma once

#include <imgui.h>

/**
 * @brief Flags that tweak the appearance/behaviour of ImGui toggle switches.
 */
enum ImGuiToggleFlags_ {
  ImGuiToggleFlags_None = 0,
  ImGuiToggleFlags_Animated = 1 << 0,
  ImGuiToggleFlags_Bordered = 1 << 1,
  ImGuiToggleFlags_Shadowed = 1 << 2,
  ImGuiToggleFlags_A11y = 1 << 3
};
using ImGuiToggleFlags = int;

/**
 * @brief Optional configuration for the toggle widget.
 *
 * This subset mimics the cmdwtf/imgui_toggle interface closely enough for our use-case.
 */
struct ImGuiToggleConfig {
  ImGuiToggleFlags Flags = ImGuiToggleFlags_None;
  float AnimationDuration = 0.08f;
  float Height = 18.0f;
  float Width = 34.0f;
};

namespace ImGui {

/**
 * @brief Draw a toggle widget that behaves like cmdwtf/imgui_toggle.
 * @param label Widget label (can be hidden with "##").
 * @param v Pointer to the boolean value to mutate.
 * @param config Extra style/behaviour knobs.
 * @return True if the value changed this frame.
 */
bool Toggle(const char* label, bool* v, const ImGuiToggleConfig& config = {});

/**
 * @brief Convenience overload that only takes flags.
 */
bool Toggle(const char* label, bool* v, ImGuiToggleFlags flags);

}  // namespace ImGui

