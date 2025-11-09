#include <chrono>
#include <cmath>

#include "simulator.hpp"
#include "comm/CommServer.hpp"
#include "TrackLoader.hpp"
#include <spdlog/spdlog.h>

namespace sim {

Simulator::Simulator(scene::SceneDB& db)
  : m_db(db) {
}

Simulator::~Simulator() {
  stop();
}

void Simulator::start() {
  if (m_running.exchange(true))
    return;
  m_thread = std::thread(&Simulator::loop, this);
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

void Simulator::probeConnection() {
  if (m_commServer && m_commEnabled.load(std::memory_order_relaxed) && 
      m_syncMode.load(std::memory_order_relaxed)) {
    m_commServer->probeConnection(50);  // 50ms timeout for heartbeat
  }
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
      m_lastControlInput = scene::CarInput{0.0, 0.0};
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

void Simulator::loop() {
  double dt = m_params.dt;
  using clock = std::chrono::steady_clock;
  auto next = clock::now();
  uint64_t tick = 0;

  // Lambda to initialize/reset state
  auto resetState = [&]() {
    m_state.car.pose = m_startPose;
    m_state.car.v = 0.0;
    m_state.car.yaw_rate = 0.0;
    
    // Reset control inputs
    m_state.car_input.delta = 0.0;
    m_state.car_input.delta_dot = 0.0;
    m_state.car_input.ax = 0.0;

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
    
    // Check for input update request (from manual setInput)
    if (m_inputUpdateRequested.exchange(false, std::memory_order_relaxed)) {
      // In Rate mode, preserve the integrated delta (steering angle state)
      // Only update the rate and acceleration inputs
      if (m_state.steering_mode == scene::SteeringMode::Rate) {
        double preserved_delta = m_state.car_input.delta;
        m_state.car_input = m_newInput;
        m_state.car_input.delta = preserved_delta;
      } else {
        // In Angle mode, just copy everything
        m_state.car_input = m_newInput;
      }
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
              // Only update parameters that were explicitly set
              if (cmd.params().has_dt()) {
                m_params.dt = cmd.params().dt();
              }
              if (cmd.params().has_wheelbase()) {
                m_params.wheelbase = cmd.params().wheelbase();
              }
              if (cmd.params().has_v_max()) {
                m_params.v_max = cmd.params().v_max();
              }
              if (cmd.params().has_delta_max()) {
                m_params.delta_max = cmd.params().delta_max();
              }
              if (cmd.params().has_ax_max()) {
                m_params.ax_max = cmd.params().ax_max();
              }
              if (cmd.params().has_steer_rate_max()) {
                m_params.steer_rate_max = cmd.params().steer_rate_max();
              }
              if (cmd.params().has_steering_mode()) {
                // Convert protobuf steering mode enum to scene::SteeringMode
                if (cmd.params().steering_mode() == lilsim::SteeringInputMode::ANGLE) {
                  m_params.steering_mode = scene::SteeringMode::Angle;
                } else {
                  m_params.steering_mode = scene::SteeringMode::Rate;
                }
              }
              m_paramsUpdatedExternally.store(true, std::memory_order_relaxed);
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
            msg = cmd.sync_mode() ? "Synchronous mode enabled" : "Asynchronous mode enabled";
            break;

          case lilsim::AdminCommandType::SET_TRACK: {
            if (!cmd.track_path().empty()) {
              scene::TrackData trackData;
              if (scene::TrackLoader::loadFromCSV(cmd.track_path(), trackData)) {
                setCones(trackData.cones);
                if (trackData.startPose.has_value()) {
                  setStartPose(trackData.startPose.value());
                }
                msg = "Track loaded: " + cmd.track_path();
                spdlog::info("[sim] Loaded track from {}", cmd.track_path());
              } else {
                success = false;
                msg = "Failed to load track: " + cmd.track_path();
                spdlog::error("[sim] Failed to load track from {}", cmd.track_path());
              }
            } else {
              success = false;
              msg = "No track path provided";
            }
            break;
          }

          case lilsim::AdminCommandType::INIT:
            if (cmd.has_params()) {
              m_params.dt = cmd.params().dt();
              m_params.wheelbase = cmd.params().wheelbase();
              m_params.v_max = cmd.params().v_max();
              m_params.delta_max = cmd.params().delta_max();
              m_params.ax_max = cmd.params().ax_max();
              m_params.steer_rate_max = cmd.params().steer_rate_max();
              // Convert protobuf steering mode enum to scene::SteeringMode
              if (cmd.params().steering_mode() == lilsim::SteeringInputMode::ANGLE) {
                m_params.steering_mode = scene::SteeringMode::Angle;
              } else {
                m_params.steering_mode = scene::SteeringMode::Rate;
              }
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
        scene::CarState(m_params.wheelbase, m_params.wheelbase / 2.0,
                        m_params.v_max);
      m_state.car_input.ax_max = m_params.ax_max;
      m_state.car_input.steer_rate_max = m_params.steer_rate_max;
      m_state.car_input.delta_max = m_params.delta_max;
      m_state.steering_mode = m_params.steering_mode;
      dt = m_params.dt;
      
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
    scene::CarInput u;
    
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
            
            // Try to get response (connection state updated automatically)
            if (m_commServer->requestControl(request, reply, pollInterval)) {
              // spdlog::info("[sim] Control request successful (tick={}, steer_angle={:.2f}, steer_rate={:.2f}, ax={:.2f})",
              //              tick, reply.steer_angle(), reply.steer_rate(), reply.ax()); 
              gotReply = true;
              u.delta = reply.steer_angle();
              u.delta_dot = reply.steer_rate();
              u.ax = reply.ax();
              m_lastControlInput = u;
              break;
            }
            
            // Check total timeout
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
              clock::now() - pollStart).count();
            if (elapsed >= totalTimeout) {
              // Client disconnected (connection state already set to false by requestControl)
              spdlog::warn("[sim] Control request timeout ({}ms), client disconnected", elapsed);
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
          u.delta_dot = asyncControl->steer_rate();
          u.ax = asyncControl->ax();
          m_lastControlInput = u;
        } else {
          // No new async control: use last control
          u = m_lastControlInput;
        }
      }
    } else {
      // No comm: use manual input from scene
      u = m_state.car_input;
    }

    // Apply steering based on mode
    if (m_state.steering_mode == scene::SteeringMode::Rate) {
      // Steering rate mode: integrate delta_dot to get delta
      // Clamp rate input
      u.delta_dot = std::clamp(u.delta_dot, -m_state.car_input.steer_rate_max, 
                                             m_state.car_input.steer_rate_max);
      // Integrate
      m_state.car_input.delta += u.delta_dot * dt;
      // Clamp angle
      m_state.car_input.delta = std::clamp(m_state.car_input.delta, 
                                           -m_state.car_input.delta_max, 
                                            m_state.car_input.delta_max);
      // Update rate in state for visualization
      m_state.car_input.delta_dot = u.delta_dot;
    } else {
      // Steering angle mode: use delta directly
      // Clamp angle input
      m_state.car_input.delta = std::clamp(u.delta, -m_state.car_input.delta_max, 
                                                    m_state.car_input.delta_max);
    }
    
    // Clamp acceleration
    m_state.car_input.ax = std::clamp(u.ax, -m_state.car_input.ax_max, 
                                             m_state.car_input.ax_max);

    // simple kinematic bicycle (minimal)
    const double yaw = m_state.car.pose.yaw();
    const double vx = m_state.car.v;

    const double dx = vx * std::cos(yaw);
    const double dy = vx * std::sin(yaw);
    const double dyaw = (vx / m_state.car.wheelbase) * std::tan(m_state.car_input.delta);
    const double dv = m_state.car_input.ax;

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
