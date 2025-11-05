/**
 * @file zmq_helpers.hpp
 * @brief Helper functions for ZeroMQ send/receive with protobuf messages
 */

#pragma once

#include <zmq.hpp>
#include <google/protobuf/message_lite.h>
#include <optional>
#include <string>

namespace comm {

/**
 * @brief Send a protobuf message over a ZeroMQ socket
 * @param socket The ZeroMQ socket to send on
 * @param message The protobuf message to send
 * @param flags Optional ZMQ send flags (default: none)
 * @return true if send succeeded, false otherwise
 */
bool sendProto(zmq::socket_t& socket, 
               const google::protobuf::MessageLite& message,
               zmq::send_flags flags = zmq::send_flags::none);

/**
 * @brief Receive a protobuf message from a ZeroMQ socket
 * @param socket The ZeroMQ socket to receive from
 * @param message The protobuf message to populate
 * @param flags Optional ZMQ receive flags (default: none)
 * @return true if receive and parse succeeded, false otherwise
 */
bool recvProto(zmq::socket_t& socket, 
               google::protobuf::MessageLite& message,
               zmq::recv_flags flags = zmq::recv_flags::none);

/**
 * @brief Receive a protobuf message with timeout
 * @param socket The ZeroMQ socket to receive from
 * @param message The protobuf message to populate
 * @param timeout_ms Timeout in milliseconds
 * @return true if received and parsed within timeout, false if timeout or error
 */
bool recvProtoTimeout(zmq::socket_t& socket,
                      google::protobuf::MessageLite& message,
                      int timeout_ms);

} // namespace comm

