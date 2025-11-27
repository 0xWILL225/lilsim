find_package(spdlog CONFIG REQUIRED)

add_library(sim STATIC
  src/simulator.cpp
  src/ModelLoader.cpp
)
target_include_directories(sim PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}
)
target_link_libraries(sim PUBLIC scene common comm atomic spdlog::spdlog ${CMAKE_DL_LIBS})
setup_warnings(sim)
setup_sanitizers(sim)
