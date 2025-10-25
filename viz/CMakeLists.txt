add_library(viz STATIC
  src/viz.cpp
)
target_include_directories(viz PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(viz PUBLIC scene common)
setup_warnings(viz)
setup_sanitizers(viz)

# wire wgpu/GLFW/ImGui here later