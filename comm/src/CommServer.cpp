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

    // Create control requester (REQ) - for synchronous mode
    m_controlReq = std::make_unique<zmq::socket_t>(*m_context, zmq::socket_type::req);
    m_controlReq->bind(endpoints::CONTROL_REQ);
    spdlog::info("[comm] Control requester bound to {}", endpoints::CONTROL_REQ);

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
    if (m_statePub) {
      m_statePub->close();
      m_statePub.reset();
    }
    if (m_controlReq) {
      m_controlReq->close();
      m_controlReq.reset();
    }
    if (m_adminRep) {
      m_adminRep->close();
      m_adminRep.reset();
    }
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
  if (!m_running.load(std::memory_order_relaxed) || !m_controlReq) {
    return false;
  }

  try {
    // Send control request
    if (!sendProto(*m_controlReq, request, zmq::send_flags::none)) {
      return false;
    }

    // Wait for reply with timeout
    return recvProtoTimeout(*m_controlReq, reply, timeout_ms);

  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error in control request: {}", e.what());
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
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] ZMQ error during MarkerSubscriber stop: {}", e.what());
  }

  spdlog::info("[comm] MarkerSubscriber stopped");
}

std::optional<lilsim::MarkerArray> MarkerSubscriber::pollMarkers() {
  if (!m_running.load(std::memory_order_relaxed) || !m_markerSub) {
    return std::nullopt;
  }

  try {
    lilsim::MarkerArray markers;
    if (recvProto(*m_markerSub, markers, zmq::recv_flags::dontwait)) {
      return markers;
    }
  } catch (const zmq::error_t& e) {
    spdlog::error("[comm] Error polling markers: {}", e.what());
  }

  return std::nullopt;
}

} // namespace comm

