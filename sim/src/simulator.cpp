#include <chrono>
#include <cmath>

#include "simulator.hpp"
#include "comm/CommServer.hpp"
#include <spdlog/spdlog.h>

namespace sim {

Simulator::Simulator(scene::SceneDB& db)
  : m_db(db) {
}

Simulator::~Simulator() {
  stop();
}

void Simulator::start(double dt) {
  if (m_running.exchange(true))
    return;
  m_dt = dt;
  m_thread = std::thread(&Simulator::loop, this, dt);
}

void Simulator::stop() {
  if (!m_running.exchange(false))
    return;
  if (m_thread.joinable())
    m_thread.join();
  
  // Stop communication server if running
  if (m_commServer) {
    m_commServer->stop();
  }
}

bool Simulator::isSyncClientConnected() const {
  return m_commServer && m_commServer->isSyncClientConnected();
}

void Simulator::enableComm(bool enable) {
  if (enable && !m_commEnabled.load(std::memory_order_relaxed)) {
    // Start communication
    if (!m_commServer) {
      m_commServer = std::make_unique<comm::CommServer>();
    }
    if (m_commServer->start()) {
      m_commEnabled.store(true, std::memory_order_relaxed);
      // Reset control input to zero when ZMQ is enabled
      m_lastControlInput = CarInput{0.0, 0.0};
      // Reset sync mode to async
      m_syncMode.store(false, std::memory_order_relaxed);
      spdlog::info("[sim] Communication enabled");
    } else {
      spdlog::error("[sim] Failed to start communication server");
    }
  } else if (!enable && m_commEnabled.load(std::memory_order_relaxed)) {
    // Stop communication
    if (m_commServer) {
      m_commServer->stop();
    }
    m_commEnabled.store(false, std::memory_order_relaxed);
    spdlog::info("[sim] Communication disabled");
  }
}

void Simulator::loop(double dt) {
  using clock = std::chrono::steady_clock;
  auto next = clock::now();
  uint64_t tick = 0;

  // Lambda to initialize/reset state
  auto resetState = [&]() {
    m_state.car.pose = m_startPose;
    m_state.car.v = 0.0;
    m_state.car.yaw_rate = 0.0;

    m_db.publish(m_state);
    tick = 0;
    m_db.tick.store(tick, std::memory_order_relaxed);
  };

  // Helper to create StateUpdate message from current state
  auto createStateUpdate = [&]() -> lilsim::StateUpdate {
    lilsim::StateUpdate update;
    auto* scene = update.mutable_scene();
    auto* header = scene->mutable_header();
    header->set_tick(tick);
    header->set_sim_time(tick * dt);
    header->set_version(1);

    auto* car = scene->mutable_car();
    auto* pos = car->mutable_pos();
    pos->set_x(m_state.car.pose.x());
    pos->set_y(m_state.car.pose.y());
    car->set_yaw(m_state.car.pose.yaw());
    car->set_v(m_state.car.v);
    car->set_yaw_rate(m_state.car.yaw_rate);

    return update;
  };

  resetState();

  while (m_running.load(std::memory_order_relaxed)) {
    // Check for start pose update request
    if (m_startPoseUpdateRequested.exchange(false, std::memory_order_relaxed)) {
      m_startPose = m_newStartPose;
    }
    
    // Check for cone update request
    if (m_conesUpdateRequested.exchange(false, std::memory_order_relaxed)) {
      m_state.cones = m_newCones;
      m_db.publish(m_state);
    }
    
    // Handle admin commands (if comm enabled)
    if (m_commEnabled.load(std::memory_order_relaxed) && m_commServer) {
      auto adminCmd = m_commServer->pollAdminCommand();
      if (adminCmd.has_value()) {
        lilsim::AdminReply reply;
        auto* header = reply.mutable_header();
        header->set_tick(tick);
        header->set_sim_time(tick * dt);
        header->set_version(1);

        const auto& cmd = adminCmd.value();
        bool success = true;
        std::string msg = "OK";

        switch (cmd.type()) {
          case lilsim::AdminCommandType::PAUSE:
            m_paused.store(true, std::memory_order_relaxed);
            msg = "Simulation paused";
            break;

          case lilsim::AdminCommandType::RUN:
            m_paused.store(false, std::memory_order_relaxed);
            msg = "Simulation running";
            break;

          case lilsim::AdminCommandType::STEP:
            m_stepTarget.store(cmd.step_count(), std::memory_order_relaxed);
            m_paused.store(false, std::memory_order_relaxed);
            msg = "Stepping " + std::to_string(cmd.step_count()) + " ticks";
            break;

          case lilsim::AdminCommandType::RESET:
            m_resetRequested.store(true, std::memory_order_relaxed);
            msg = "Reset requested";
            break;

          case lilsim::AdminCommandType::SET_PARAMS:
            if (cmd.has_params()) {
              m_newParams.dt = cmd.params().dt();
              m_newParams.wheelbase = cmd.params().wheelbase();
              m_newParams.v_max = cmd.params().v_max();
              m_newParams.delta_max = cmd.params().delta_max();
              m_resetRequested.store(true, std::memory_order_relaxed);
              msg = "Parameters set, reset requested";
            } else {
              success = false;
              msg = "No parameters provided";
            }
            break;

          case lilsim::AdminCommandType::SET_MODE:
            setSyncMode(cmd.sync_mode());
            if (cmd.control_period_ms() > 0) {
              setControlPeriodMs(cmd.control_period_ms());
              // Recompute control period in ticks
              uint32_t ticks = static_cast<uint32_t>(std::max(1.0, cmd.control_period_ms() / (dt * 1000.0)));
              m_controlPeriodTicks.store(ticks, std::memory_order_relaxed);
            }
            if (cmd.sync_mode()) {
              // Switching to sync mode - reset connection state
              m_commServer->setSyncClientConnected(false);
            }
            msg = cmd.sync_mode() ? "Synchronous mode enabled" : "Asynchronous mode enabled";
            break;

          case lilsim::AdminCommandType::INIT:
            if (cmd.has_params()) {
              m_newParams.dt = cmd.params().dt();
              m_newParams.wheelbase = cmd.params().wheelbase();
              m_newParams.v_max = cmd.params().v_max();
              m_newParams.delta_max = cmd.params().delta_max();
              m_resetRequested.store(true, std::memory_order_relaxed);
              msg = "Initialized with new parameters";
            } else {
              success = false;
              msg = "No parameters provided for init";
            }
            break;

          default:
            success = false;
            msg = "Unknown command type";
            break;
        }

        reply.set_success(success);
        reply.set_message(msg);
        m_commServer->replyAdmin(reply);
      }
    }
    
    // Check for reset request
    if (m_resetRequested.exchange(false, std::memory_order_relaxed)) {
      // Apply new parameters
      m_state.car =
        scene::CarState(m_newParams.wheelbase, m_newParams.wheelbase / 2.0,
                        m_newParams.v_max, m_newParams.delta_max);
      dt = m_newParams.dt;
      m_dt = dt;
      // Recompute control period in ticks based on new dt
      uint32_t periodMs = m_controlPeriodMs.load(std::memory_order_relaxed);
      uint32_t ticks = static_cast<uint32_t>(std::max(1.0, periodMs / (dt * 1000.0)));
      m_controlPeriodTicks.store(ticks, std::memory_order_relaxed);
      // Reset state and tick counter
      resetState();
      m_paused.store(true, std::memory_order_relaxed);
      next = clock::now();
      continue;
    }
    // Check if paused
    if (m_paused.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      next = clock::now(); // Reset timing when resuming
      continue;
    }

    // Check step mode
    uint64_t stepsRemaining = m_stepTarget.load(std::memory_order_relaxed);
    if (stepsRemaining > 0) {
      // Step mode: execute one tick then decrement
      m_stepTarget.fetch_sub(1, std::memory_order_relaxed);
      if (stepsRemaining == 1) {
        // Last step completed, pause
        m_paused.store(true, std::memory_order_relaxed);
      }
    }

    // Determine control input based on mode
    CarInput u;
    
    if (m_commEnabled.load(std::memory_order_relaxed) && m_commServer) {
      bool syncMode = m_syncMode.load(std::memory_order_relaxed);
      uint32_t controlPeriodTicks = m_controlPeriodTicks.load(std::memory_order_relaxed);

      if (syncMode) {
        bool isControlTick = (tick % controlPeriodTicks == 0);
        
        if (isControlTick) {
          // Synchronous mode: request control from client
          lilsim::ControlRequest request;
          auto* header = request.mutable_header();
          header->set_tick(tick);
          header->set_sim_time(tick * dt);
          header->set_version(1);

          // Include current state in request
          auto* scene = request.mutable_scene();
          auto* sceneHeader = scene->mutable_header();
          sceneHeader->set_tick(tick);
          sceneHeader->set_sim_time(tick * dt);
          sceneHeader->set_version(1);

          auto* car = scene->mutable_car();
          auto* pos = car->mutable_pos();
          pos->set_x(m_state.car.pose.x());
          pos->set_y(m_state.car.pose.y());
          car->set_yaw(m_state.car.pose.yaw());
          car->set_v(m_state.car.v);
          car->set_yaw_rate(m_state.car.yaw_rate);

          // Poll for control response with 50ms intervals, 500ms total timeout
          lilsim::ControlReply reply;
          bool gotReply = false;
          auto pollStart = clock::now();
          const int pollInterval = 50;  // ms
          const int totalTimeout = 500; // ms
          
          while (!gotReply) {
            // Check if we should exit polling loop (mode changed, comm disabled)
            if (!m_commEnabled.load(std::memory_order_relaxed) || 
                !m_syncMode.load(std::memory_order_relaxed)) {
              spdlog::info("[sim] Sync mode disabled during control request, aborting");
              break;
            }
            
            // Try to get response
            if (m_commServer->requestControl(request, reply, pollInterval)) {
              gotReply = true;
              u.delta = reply.steer_angle();
              u.ax = reply.ax();
              m_lastControlInput = u;
              m_commServer->setSyncClientConnected(true);
              break;
            }
            
            // Check total timeout
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
              clock::now() - pollStart).count();
            if (elapsed >= totalTimeout) {
              // Client disconnected
              spdlog::warn("[sim] Control request timeout ({}ms), client disconnected", elapsed);
              m_commServer->setSyncClientConnected(false);
              m_paused.store(true, std::memory_order_relaxed);
              break;
            }
          }
          
          // If still in sync mode but didn't get reply, pause and wait
          if (syncMode && !gotReply && m_commEnabled.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            next = clock::now();
            continue;
          }
        } else {
          // Not a control tick in sync mode: use last control
          u = m_lastControlInput;
        }
      } else {
        // Asynchronous mode: poll for async control
        auto asyncControl = m_commServer->pollAsyncControl();
        if (asyncControl.has_value()) {
          u.delta = asyncControl->steer_angle();
          u.ax = asyncControl->ax();
          m_lastControlInput = u;
        } else {
          // No new async control: use last control
          u = m_lastControlInput;
        }
      }
    } else {
      // No comm: use manual input
      u = m_input.load(std::memory_order_relaxed);
    }

    // Clamp steering angle to limits
    u.delta =
      std::clamp(u.delta, -m_state.car.delta_max, m_state.car.delta_max);

    // simple kinematic bicycle (minimal)
    const double yaw = m_state.car.pose.yaw();
    const double vx = m_state.car.v;

    const double dx = vx * std::cos(yaw);
    const double dy = vx * std::sin(yaw);
    const double dyaw = (vx / m_state.car.wheelbase) * std::tan(u.delta);
    const double dv = u.ax;

    double x = m_state.car.pose.x() + dt * dx;
    double y = m_state.car.pose.y() + dt * dy;
    double ynew = yaw + dt * dyaw;
    double v = m_state.car.v + dt * dv;

    // Clamp velocity to [0, v_max]
    v = std::clamp(v, 0.0, m_state.car.v_max);

    // update state
    m_state.car.pose.setFromXYYaw(x, y,
                                  std::atan2(std::sin(ynew), std::cos(ynew)));
    m_state.car.v = v;
    m_state.car.yaw_rate = dyaw;

    // Increment tick
    tick++;
    m_db.tick.store(tick, std::memory_order_relaxed);

    // Publish to local scene database
    m_db.publish(m_state);

    // Publish state update over ZMQ (if comm enabled)
    if (m_commEnabled.load(std::memory_order_relaxed) && m_commServer) {
      m_commServer->publishState(createStateUpdate());
    }

    next += std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(dt));
    std::this_thread::sleep_until(next);
  }
}

} // namespace sim
