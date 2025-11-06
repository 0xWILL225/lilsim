/**
 * @file endpoints.hpp
 * @brief Fixed ZeroMQ endpoint URIs and port layout for lilsim communication
 */

#pragma once

#include <string>

namespace comm {

/**
 * @brief ZeroMQ endpoint configuration
 * 
 * This defines the fixed port layout for all communication channels:
 * - 5556: State stream (sim → client, PUB/SUB)
 * - 5557: Synchronous control (sim ↔ client, REQ/REP)
 * - 5558: Admin commands (client ↔ sim, REQ/REP)
 * - 5560: Marker stream (client → viz, PUB/SUB)
 */
namespace endpoints {

// State stream: simulator publishes state updates
constexpr const char* STATE_PUB = "tcp://*:5556";
constexpr const char* STATE_SUB = "tcp://localhost:5556";

// Synchronous control: simulator requests control, client replies
constexpr const char* CONTROL_REQ = "tcp://*:5557";
constexpr const char* CONTROL_REP = "tcp://localhost:5557";

// Admin commands: client sends commands, simulator replies
constexpr const char* ADMIN_REP = "tcp://*:5558";
constexpr const char* ADMIN_REQ = "tcp://localhost:5558";

// Asynchronous control: client publishes control, simulator subscribes
constexpr const char* CONTROL_ASYNC_SUB = "tcp://*:5559";
constexpr const char* CONTROL_ASYNC_PUB = "tcp://localhost:5559";

// Marker stream: client publishes markers, viz subscribes
constexpr const char* MARKER_PUB = "tcp://localhost:5560";
constexpr const char* MARKER_SUB = "tcp://*:5560";

} // namespace endpoints

} // namespace comm

