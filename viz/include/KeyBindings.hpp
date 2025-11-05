#pragma once

#include <GLFW/glfw3.h>

namespace viz {

/**
 * @brief Centralized keybinding configuration.
 *
 * Modify the values in this struct to change keybindings throughout the
 * application. All keybindings are defined in one place for easy rebinding.
 */
struct KeyBindings {
  // Car control
  int carAccelerate = GLFW_KEY_W;
  int carBrake = GLFW_KEY_S;
  int carSteerLeft = GLFW_KEY_A;
  int carSteerRight = GLFW_KEY_D;

  // Camera control
  int toggleCameraMode = GLFW_KEY_TAB;

  // Simulation control
  int pauseSimulation = GLFW_KEY_SPACE;
  int resetSimulation = GLFW_KEY_R;
  int stepSimulation = GLFW_KEY_ENTER;
  int stepSimulationN = GLFW_KEY_N;
};

// Global keybindings instance (defined in KeyBindings.cpp)
extern KeyBindings gKeyBindings;

} // namespace viz

