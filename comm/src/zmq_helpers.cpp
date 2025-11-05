#include "comm/zmq_helpers.hpp"
#include <spdlog/spdlog.h>

namespace comm {

bool sendProto(zmq::socket_t& socket, 
               const google::protobuf::MessageLite& message,
               zmq::send_flags flags) {
  std::string serialized;
  if (!message.SerializeToString(&serialized)) {
    spdlog::error("[comm] Failed to serialize protobuf message");
    return false;
  }

  zmq::message_t zmqMsg(serialized.data(), serialized.size());
  auto result = socket.send(zmqMsg, flags);
  
  if (!result.has_value()) {
    spdlog::error("[comm] Failed to send ZMQ message");
    return false;
  }
  
  return true;
}

bool recvProto(zmq::socket_t& socket, 
               google::protobuf::MessageLite& message,
               zmq::recv_flags flags) {
  zmq::message_t zmqMsg;
  auto result = socket.recv(zmqMsg, flags);
  
  if (!result.has_value()) {
    // No error message for dontwait flag - it's expected
    if (flags != zmq::recv_flags::dontwait) {
      spdlog::error("[comm] Failed to receive ZMQ message");
    }
    return false;
  }

  if (!message.ParseFromArray(zmqMsg.data(), static_cast<int>(zmqMsg.size()))) {
    spdlog::error("[comm] Failed to parse protobuf message");
    return false;
  }

  return true;
}

bool recvProtoTimeout(zmq::socket_t& socket,
                      google::protobuf::MessageLite& message,
                      int timeout_ms) {
  // Set socket receive timeout
  socket.set(zmq::sockopt::rcvtimeo, timeout_ms);
  
  bool success = recvProto(socket, message, zmq::recv_flags::none);
  
  // Reset timeout to no timeout
  socket.set(zmq::sockopt::rcvtimeo, -1);
  
  return success;
}

} // namespace comm

