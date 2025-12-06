/**
 * @file CommServer.hpp
 * @brief Central communication server managing all ZeroMQ sockets
 */

#pragma once

#include <zmq.hpp>
#include <atomic>
#include <memory>
#include <functional>
#include <optional>

#include "messages.pb.h"

namespace comm {

/**
 * @brief Central server for ZeroMQ communication
 * 
 * Manages three sockets used by the simulator:
 * 1. State publisher (PUB) - broadcasts state updates
 * 2. Control requester (REQ) - requests control in sync mode (optional)
 * 3. Admin replier (REP) - handles admin commands
 */
class CommServer {
public:
  CommServer();
  ~CommServer();

  // Disable copy/move
  CommServer(const CommServer&) = delete;
  CommServer& operator=(const CommServer&) = delete;

  bool start();
  void stop();
  bool isRunning() const { return m_running.load(std::memory_order_relaxed); }
  void publishState(const lilsim::StateUpdate& update);
  void publishMetadata(const lilsim::ModelMetadata& metadata);

  bool sendControlRequest(const lilsim::ControlRequest& request);
  std::optional<lilsim::ControlReply> pollControlReply();
  bool waitControlReply(lilsim::ControlReply& reply, int timeout_ms);

  bool probeConnection(int timeout_ms = 50);
  std::optional<lilsim::AdminCommand> pollAdminCommand();
  void replyAdmin(const lilsim::AdminReply& reply);
  bool hasAdminCommandPending() const { 
    return m_adminCommandPending.load(std::memory_order_relaxed); 
  }
  std::optional<lilsim::ControlAsync> pollAsyncControl();
  bool isSyncClientConnected() const {
    return m_syncClientConnected.load(std::memory_order_relaxed);
  }
  std::shared_ptr<zmq::context_t> getContext() const { return m_context; }

private:
  std::shared_ptr<zmq::context_t> m_context;
  std::unique_ptr<zmq::socket_t> m_statePub;        // PUB socket for state updates
  std::unique_ptr<zmq::socket_t> m_metadataPub;     // PUB socket for metadata updates
  std::unique_ptr<zmq::socket_t> m_controlDealer;   // DEALER socket for control requests/replies (sync)
  std::unique_ptr<zmq::socket_t> m_controlAsyncSub; // SUB socket for async control
  std::unique_ptr<zmq::socket_t> m_adminRep;        // REP socket for admin commands
  
  std::atomic<bool> m_running{false};
  std::atomic<bool> m_adminCommandPending{false};
  std::atomic<bool> m_syncClientConnected{false};
};

/**
 * @brief Marker subscriber for viz module
 * 
 * Subscribes to marker messages from clients
 */
class MarkerSubscriber {
public:
  MarkerSubscriber();
  ~MarkerSubscriber();

  // Disable copy/move
  MarkerSubscriber(const MarkerSubscriber&) = delete;
  MarkerSubscriber& operator=(const MarkerSubscriber&) = delete;

  bool start();
  void stop();

  /**
   * @brief Check if the subscriber is running
   */
  bool isRunning() const { return m_running.load(std::memory_order_relaxed); }

  /**
   * @brief Message type indicator for polled messages
   */
  enum class MessageType {
    None,
    MarkerArray,
    MarkerCommand
  };

  /**
   * @brief Result of polling for a message
   */
  struct PollResult {
    MessageType type;
    std::optional<lilsim::MarkerArray> marker_array;
    std::optional<lilsim::MarkerCommand> marker_command;
  };

  PollResult poll();

private:
  std::unique_ptr<zmq::context_t> m_context;
  std::unique_ptr<zmq::socket_t> m_markerSub;
  std::atomic<bool> m_running{false};
};

} // namespace comm

