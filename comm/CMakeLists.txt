find_package(Protobuf REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(cppzmq REQUIRED)

# Generate protobuf files
set(PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/messages.pb.cc")
set(PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/messages.pb.h")

add_custom_command(
  OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
  ARGS --experimental_allow_proto3_optional
       --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
       --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
       ${CMAKE_CURRENT_SOURCE_DIR}/messages.proto
  DEPENDS messages.proto
  COMMENT "Generating protobuf C++ sources"
)

add_library(comm STATIC
  ${PROTO_SRCS}
  src/zmq_helpers.cpp
  src/CommServer.cpp
)

target_include_directories(comm PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}  # For generated .pb.h
)

target_link_libraries(comm PUBLIC
  protobuf::libprotobuf
  cppzmq
  libzmq
  common
)

# Disable linting for generated protobuf code
set_source_files_properties(
  ${PROTO_SRCS}
  PROPERTIES
  SKIP_LINTING ON
)

