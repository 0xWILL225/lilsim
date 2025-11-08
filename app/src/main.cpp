#include <cstdio>

#include "scene.hpp"
#include "simulator.hpp"
#include "viz.hpp"

int main() {
  fprintf(stderr, "[DEBUG] Starting lilsim...\n");

  // Initialize scene and simulator
  scene::SceneDB db;
  sim::Simulator sim(db);
  sim.start();
  fprintf(stderr, "[DEBUG] Simulator started\n");

  // Initialize visualization
  fprintf(stderr, "[DEBUG] Initializing visualization...\n");
  viz::Application app(db, sim);
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
