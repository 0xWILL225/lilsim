#include <cstdio>
#include <filesystem>

#include "scene.hpp"
#include "simulator.hpp"
#include "viz.hpp"

int main(int argc, char** argv) {
  fprintf(stderr, "[DEBUG] Starting lilsim...\n");

  // Initialize scene and simulator
  scene::SceneDB db;
  sim::Simulator sim(db);
  sim.start();
  fprintf(stderr, "[DEBUG] Simulator started\n");

  std::filesystem::path installRoot;
  try {
    if (argc > 0 && argv && argv[0]) {
      std::filesystem::path binaryPath = std::filesystem::path(argv[0]);
      if (!binaryPath.is_absolute()) {
        binaryPath = std::filesystem::absolute(binaryPath);
      }
      if (std::filesystem::is_regular_file(binaryPath)) {
        installRoot = binaryPath.parent_path();
      } else if (std::filesystem::is_directory(binaryPath)) {
        installRoot = binaryPath;
      }
    }
  } catch (const std::exception& ex) {
    fprintf(stderr, "[WARN] Failed to resolve executable directory: %s\n", ex.what());
  }
  if (installRoot.empty()) {
    installRoot = std::filesystem::current_path();
  }

  // Initialize visualization
  fprintf(stderr, "[DEBUG] Initializing visualization...\n");
  viz::Application app(db, sim, installRoot);
  if (!app.initialize()) {
    fprintf(stderr, "Failed to initialize application\n");
    sim.stop();
    return -1;
  }
  fprintf(stderr, "[DEBUG] Visualization initialized successfully\n");

  // Main loop
  fprintf(stderr, "[DEBUG] Entering main loop...\n");
  while (app.isRunning()) {
    app.mainLoop();
  }
  fprintf(stderr, "[DEBUG] Exited main loop\n");

  // Cleanup
  sim.stop();
  app.terminate();
  fprintf(stderr, "[DEBUG] Cleanup complete\n");

  return 0;
}
