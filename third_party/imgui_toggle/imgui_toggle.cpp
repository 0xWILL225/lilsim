#include "imgui_toggle.h"

#include <algorithm>
#include <imgui_internal.h>

namespace {

float Lerp(float a, float b, float t) {
  return a + (b - a) * t;
}

}  // namespace

namespace ImGui {

bool Toggle(const char* label, bool* v, ImGuiToggleFlags flags) {
  ImGuiToggleConfig cfg;
  cfg.Flags = flags;
  return Toggle(label, v, cfg);
}

bool Toggle(const char* label, bool* v, const ImGuiToggleConfig& config) {
  IM_ASSERT(label != nullptr);
  IM_ASSERT(v != nullptr);

  ImGuiWindow* window = GetCurrentWindow();
  if (window->SkipItems) {
    return false;
  }

  const ImGuiStyle& style = GetStyle();
  const float height = config.Height > 0.0f ? config.Height : GetFrameHeight();
  const float width =
      config.Width > 0.0f ? config.Width : height * 1.9f;

  ImVec2 pos = GetCursorScreenPos();
  ImRect bb(pos, ImVec2(pos.x + width, pos.y + height));
  ItemSize(bb, style.FramePadding.y);

  ImGuiID id = window->GetID(label);
  if (!ItemAdd(bb, id)) {
    return false;
  }

  bool hovered = false;
  bool held = false;
  bool pressed = ButtonBehavior(bb, id, &hovered, &held, ImGuiButtonFlags_PressedOnClick);
  if (pressed) {
    *v = !*v;
    MarkItemEdited(id);
  }

  ImDrawList* draw_list = GetWindowDrawList();
  const ImU32 col_on = GetColorU32(hovered ? ImGuiCol_ButtonHovered : ImGuiCol_Button);
  const ImU32 col_off = GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg);
  const ImU32 col_knob = GetColorU32(ImGuiCol_Text);

  const bool animated = (config.Flags & ImGuiToggleFlags_Animated) != 0;
  ImGuiStorage* storage = window->DC.StateStorage;
  float* animated_value = storage->GetFloatRef(id, *v ? 1.0f : 0.0f);
  const float target = *v ? 1.0f : 0.0f;
  if (animated) {
    const float speed = config.AnimationDuration > 0.0f
                            ? std::min(1.0f, GetIO().DeltaTime / config.AnimationDuration)
                            : 1.0f;
    *animated_value = Lerp(*animated_value, target, speed);
  } else {
    *animated_value = target;
  }

  const float t = *animated_value;
  const float radius = (height - 4.0f) * 0.5f;
  const float padding = 2.0f;

  const ImVec2 frame_min = ImVec2(bb.Min.x, bb.Min.y);
  const ImVec2 frame_max = ImVec2(bb.Max.x, bb.Max.y);
  draw_list->AddRectFilled(frame_min, frame_max, *v ? col_on : col_off, height * 0.5f);

  if (config.Flags & ImGuiToggleFlags_Bordered) {
    draw_list->AddRect(frame_min, frame_max, GetColorU32(ImGuiCol_Border), height * 0.5f, 0, 1.0f);
  }

  float knob_x = Lerp(frame_min.x + padding + radius,
                      frame_max.x - padding - radius,
                      t);
  ImVec2 knob_center(knob_x, frame_min.y + height * 0.5f);

  if (config.Flags & ImGuiToggleFlags_Shadowed) {
    const ImU32 shadow_col = GetColorU32(ImGuiCol_BorderShadow);
    draw_list->AddCircleFilled(ImVec2(knob_center.x, knob_center.y + 1.0f),
                               radius + 1.5f,
                               shadow_col,
                               32);
  }

  draw_list->AddCircleFilled(knob_center, radius, col_knob, 32);

  if (config.Flags & ImGuiToggleFlags_A11y) {
    const char glyph = *v ? 'I' : 'O';
    ImVec2 text_size = CalcTextSize(&glyph, &glyph + 1);
    draw_list->AddText(ImVec2(knob_center.x - text_size.x * 0.5f,
                              knob_center.y - text_size.y * 0.5f),
                       GetColorU32(ImGuiCol_Text),
                       &glyph,
                       &glyph + 1);
  }

  if (label[0] != '#' || label[1] != '#') {
    RenderText(ImVec2(bb.Max.x + style.ItemInnerSpacing.x, bb.Min.y + style.FramePadding.y), label);
  }

  return pressed;
}

}  // namespace ImGui

