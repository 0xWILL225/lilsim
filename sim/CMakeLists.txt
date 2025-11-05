find_package(spdlog CONFIG REQUIRED)

add_library(sim STATIC
  src/simulator.cpp
)
target_include_directories(sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim PUBLIC scene common comm atomic spdlog::spdlog)
setup_warnings(sim)
setup_sanitizers(sim)
