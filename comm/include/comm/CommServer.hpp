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
 * @brief Callback for handling admin commands
 * @param cmd The received admin command
 * @return AdminReply to send back to client
 */
using AdminCallback = std::function<lilsim::AdminReply(const lilsim::AdminCommand&)>;

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

  /**
   * @brief Initialize and bind all sockets
   * @return true if successful, false otherwise
   */
  bool start();

  /**
   * @brief Close all sockets
   */
  void stop();

  /**
   * @brief Check if the server is running
   */
  bool isRunning() const { return m_running.load(std::memory_order_relaxed); }

  /**
   * @brief Publish a state update (non-blocking)
   * @param update The state update to publish
   */
  void publishState(const lilsim::StateUpdate& update);
  /**
   * @brief Publish a metadata update (non-blocking).
   */
  void publishMetadata(const lilsim::ModelMetadata& metadata);

  /**
   * @brief Request control from client (blocking with timeout)
   * @param request The control request to send
   * @param reply Output parameter for the control reply
   * @param timeout_ms Timeout in milliseconds
   * @return true if reply received within timeout, false otherwise
   */
  bool requestControl(const lilsim::ControlRequest& request,
                      lilsim::ControlReply& reply,
                      int timeout_ms = 100);

  /**
   * @brief Probe if sync client is connected (lightweight heartbeat check)
   * @param timeout_ms Timeout in milliseconds
   * @return true if client responded, false otherwise
   */
  bool probeConnection(int timeout_ms = 50);

  /**
   * @brief Poll for admin commands (non-blocking)
   * @return Admin command if one is pending, std::nullopt otherwise
   */
  std::optional<lilsim::AdminCommand> pollAdminCommand();

  /**
   * @brief Reply to an admin command
   * @param reply The admin reply to send
   */
  void replyAdmin(const lilsim::AdminReply& reply);

  /**
   * @brief Check if there's a pending admin command waiting for reply
   */
  bool hasAdminCommandPending() const { 
    return m_adminCommandPending.load(std::memory_order_relaxed); 
  }

  /**
   * @brief Poll for async control (non-blocking)
   * @return ControlAsync if one is pending, std::nullopt otherwise
   */
  std::optional<lilsim::ControlAsync> pollAsyncControl();

  /**
   * @brief Check if sync client is connected
   * 
   * Connection state is automatically updated by requestControl() and probeConnection()
   */
  bool isSyncClientConnected() const {
    return m_syncClientConnected.load(std::memory_order_relaxed);
  }

  /**
   * @brief Shared ZeroMQ context for inproc sockets (viz GUI).
   */
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

  /**
   * @brief Initialize and connect subscriber
   * @return true if successful, false otherwise
   */
  bool start();

  /**
   * @brief Close the socket
   */
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

  /**
   * @brief Poll for any marker-related message (non-blocking)
   * @return PollResult containing the message type and data
   */
  PollResult poll();

private:
  std::unique_ptr<zmq::context_t> m_context;
  std::unique_ptr<zmq::socket_t> m_markerSub;
  std::atomic<bool> m_running{false};
};

} // namespace comm

