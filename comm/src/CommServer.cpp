#include "comm/CommServer.hpp"
#include "comm/endpoints.hpp"
#include "comm/zmq_helpers.hpp"
#include <spdlog/spdlog.h>

namespace comm {

// ============ CommServer ============

CommServer::CommServer() 
  : m_context(std::make_unique<zmq::context_t>(1)) {
}

CommServer::~CommServer() {
  stop();
}

bool CommServer::start() {
  if (m_running.load(std::memory_order_relaxed)) {
    spdlog::warn("[comm] CommServer already running");
    return true;
  }

  try {
    // Create state publisher (PUB)
    m_statePub = std::make_unique<zmq::socket_t>(*m_context, zmq::socket_type::pub);
    m_statePub->bind(endpoints::STATE_PUB);
    spdlog::info("[comm] State publisher bound to {}", endpoints::STATE_PUB);

    // Create control dealer (DEALER) - for synchronous mode
    m_controlDealer = std::make_unique<zmq::socket_t>(*m_context, zmq::socket_type::dealer);
    m_controlDealer->bind(endpoints::CONTROL_REQ);
    spdlog::info("[comm] Control dealer bound to {}", endpoints::CONTROL_REQ);

    // Create async control subscriber (SUB) - for asynchronous mode
    m_controlAsyncSub = std::make_unique<zmq::socket_t>(*m_context, zmq::socket_type::sub);
    m_controlAsyncSub->bind(endpoints::CONTROL_ASYNC_SUB);
    m_controlAsyncSub->set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
    spdlog::info("[comm] Async control subscriber bound to {}", endpoints::CONTROL_ASYNC_SUB);

    // Create admin replier (REP)
    m_adminRep = std::make_unique<zmq::socket_t>(*m_context, zmq::socket_type::rep);
    m_adminRep->bind(endpoints::ADMIN_REP);
    spdlog::info("[comm] Admin replier bound to {}", endpoints::ADMIN_REP);

    m_running.store(true, std::memory_order_relaxed);
    return true;

  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] ZMQ error during start: {}", e.what());
    stop();
    return false;
  }
}

void CommServer::stop() {
  if (!m_running.load(std::memory_order_relaxed)) {
    return;
  }

  m_running.store(false, std::memory_order_relaxed);

  try {
    // Close and reset all sockets
    if (m_statePub) {
      m_statePub->close();
      m_statePub.reset();
    }
    if (m_controlDealer) {
      m_controlDealer->close();
      m_controlDealer.reset();
    }
    if (m_controlAsyncSub) {
      m_controlAsyncSub->close();
      m_controlAsyncSub.reset();
    }
    if (m_adminRep) {
      m_adminRep->close();
      m_adminRep.reset();
    }
    
    // Don't shutdown/reset context here - let destructor handle it
    // This avoids issues with context lifecycle when stop/start is called multiple times
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] ZMQ error during stop: {}", e.what());
  }

  spdlog::info("[comm] CommServer stopped");
}

void CommServer::publishState(const lilsim::StateUpdate& update) {
  if (!m_running.load(std::memory_order_relaxed) || !m_statePub) {
    return;
  }

  try {
    sendProto(*m_statePub, update, zmq::send_flags::dontwait);
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error publishing state: {}", e.what());
  }
}

bool CommServer::requestControl(const lilsim::ControlRequest& request,
                                lilsim::ControlReply& reply,
                                int timeout_ms) {
  if (!m_running.load(std::memory_order_relaxed) || !m_controlDealer) {
    m_syncClientConnected.store(false, std::memory_order_relaxed);
    return false;
  }

  try {
    // Send control request (DEALER doesn't enforce strict ordering)
    if (!sendProto(*m_controlDealer, request, zmq::send_flags::dontwait)) {
      m_syncClientConnected.store(false, std::memory_order_relaxed);
      return false;
    }

    // Wait for reply with timeout
    bool success = recvProtoTimeout(*m_controlDealer, reply, timeout_ms);
    
    // Update connection state based on result
    m_syncClientConnected.store(success, std::memory_order_relaxed);
    
    return success;

  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error in control request: {}", e.what());
    m_syncClientConnected.store(false, std::memory_order_relaxed);
    return false;
  }
}

bool CommServer::probeConnection(int timeout_ms) {
  if (!m_running.load(std::memory_order_relaxed) || !m_controlDealer) {
    return false;
  }

  try {
    // Send a lightweight control request with tick=0 as heartbeat
    lilsim::ControlRequest probe;
    auto* header = probe.mutable_header();
    header->set_tick(0);  // Special tick=0 indicates heartbeat
    header->set_sim_time(0.0);
    header->set_version(1);

    // Send probe
    if (!sendProto(*m_controlDealer, probe, zmq::send_flags::dontwait)) {
      m_syncClientConnected.store(false, std::memory_order_relaxed);
      return false;
    }

    // Wait for reply
    lilsim::ControlReply reply;
    bool success = recvProtoTimeout(*m_controlDealer, reply, timeout_ms);
    
    // Update connection state
    m_syncClientConnected.store(success, std::memory_order_relaxed);
    
    return success;

  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error in connection probe: {}", e.what());
    m_syncClientConnected.store(false, std::memory_order_relaxed);
    return false;
  }
}

std::optional<lilsim::AdminCommand> CommServer::pollAdminCommand() {
  if (!m_running.load(std::memory_order_relaxed) || !m_adminRep) {
    return std::nullopt;
  }

  if (m_adminCommandPending.load(std::memory_order_relaxed)) {
    // Already have a pending command waiting for reply
    return std::nullopt;
  }

  try {
    lilsim::AdminCommand cmd;
    if (recvProto(*m_adminRep, cmd, zmq::recv_flags::dontwait)) {
      m_adminCommandPending.store(true, std::memory_order_relaxed);
      return cmd;
    }
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error polling admin command: {}", e.what());
  }

  return std::nullopt;
}

void CommServer::replyAdmin(const lilsim::AdminReply& reply) {
  if (!m_running.load(std::memory_order_relaxed) || !m_adminRep) {
    return;
  }

  if (!m_adminCommandPending.load(std::memory_order_relaxed)) {
    spdlog::warn("[comm] Attempted to reply to admin command when none pending");
    return;
  }

  try {
    sendProto(*m_adminRep, reply, zmq::send_flags::none);
    m_adminCommandPending.store(false, std::memory_order_relaxed);
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error replying to admin command: {}", e.what());
  }
}

std::optional<lilsim::ControlAsync> CommServer::pollAsyncControl() {
  if (!m_running.load(std::memory_order_relaxed) || !m_controlAsyncSub) {
    return std::nullopt;
  }

  try {
    lilsim::ControlAsync control;
    if (recvProto(*m_controlAsyncSub, control, zmq::recv_flags::dontwait)) {
      return control;
    }
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error polling async control: {}", e.what());
  }

  return std::nullopt;
}

// ============ MarkerSubscriber ============

MarkerSubscriber::MarkerSubscriber()
  : m_context(std::make_unique<zmq::context_t>(1)) {
}

MarkerSubscriber::~MarkerSubscriber() {
  stop();
}

bool MarkerSubscriber::start() {
  if (m_running.load(std::memory_order_relaxed)) {
    spdlog::warn("[comm] MarkerSubscriber already running");
    return true;
  }

  try {
    // Create marker subscriber (SUB)
    m_markerSub = std::make_unique<zmq::socket_t>(*m_context, zmq::socket_type::sub);
    m_markerSub->bind(endpoints::MARKER_SUB);
    m_markerSub->set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
    spdlog::info("[comm] Marker subscriber bound to {}", endpoints::MARKER_SUB);

    m_running.store(true, std::memory_order_relaxed);
    return true;

  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] ZMQ error during MarkerSubscriber start: {}", e.what());
    stop();
    return false;
  }
}

void MarkerSubscriber::stop() {
  if (!m_running.load(std::memory_order_relaxed)) {
    return;
  }

  m_running.store(false, std::memory_order_relaxed);

  try {
    if (m_markerSub) {
      m_markerSub->close();
      m_markerSub.reset();
    }
    
    // Explicitly shutdown context to avoid blocking on destruction
    if (m_context) {
      m_context->shutdown();
      m_context.reset();
    }
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] ZMQ error during MarkerSubscriber stop: {}", e.what());
  }

  spdlog::info("[comm] MarkerSubscriber stopped");
}

MarkerSubscriber::PollResult MarkerSubscriber::poll() {
  PollResult result;
  result.type = MessageType::None;
  
  if (!m_running.load(std::memory_order_relaxed) || !m_markerSub) {
    return result;
  }

  try {
    // Receive topic frame (non-blocking)
    zmq::message_t topic_msg;
    auto res = m_markerSub->recv(topic_msg, zmq::recv_flags::dontwait);
    if (!res) {
      return result;
    }
    
    std::string topic(static_cast<char*>(topic_msg.data()), topic_msg.size());
    
    // Receive data frame (non-blocking)
    zmq::message_t data_msg;
    res = m_markerSub->recv(data_msg, zmq::recv_flags::dontwait);
    if (!res) {
      spdlog::warn("[comm] Received topic frame but no data frame");
      return result;
    }
    
    // Parse based on topic
    if (topic == "MARKERS") {
      lilsim::MarkerArray markers;
      if (markers.ParseFromArray(data_msg.data(), static_cast<int>(data_msg.size()))) {
        result.type = MessageType::MarkerArray;
        result.marker_array = std::move(markers);
      } else {
        spdlog::error("[comm] Failed to parse MarkerArray");
      }
    } else if (topic == "COMMAND") {
      lilsim::MarkerCommand command;
      if (command.ParseFromArray(data_msg.data(), static_cast<int>(data_msg.size()))) {
        result.type = MessageType::MarkerCommand;
        result.marker_command = std::move(command);
      } else {
        spdlog::error("[comm] Failed to parse MarkerCommand");
      }
    } else {
      // spdlog::warn("[comm] Unknown marker message topic: {}", topic);
    }
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error polling marker messages: {}", e.what());
  }

  return result;
}

} // namespace comm

