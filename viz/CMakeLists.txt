find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(X11 REQUIRED)

# Add wgpu-native
set(WGPU_NATIVE_DIR ${CMAKE_SOURCE_DIR}/third_party/wgpu-native)
add_library(wgpu_native SHARED IMPORTED)
set_target_properties(wgpu_native PROPERTIES
  IMPORTED_LOCATION ${WGPU_NATIVE_DIR}/lib/libwgpu_native.so
  INTERFACE_INCLUDE_DIRECTORIES ${WGPU_NATIVE_DIR}/include
)

# Add ImGuiFileDialog as a header-only library
add_library(ImGuiFileDialog INTERFACE)
target_include_directories(ImGuiFileDialog INTERFACE
  ${CMAKE_SOURCE_DIR}/third_party/ImGuiFileDialog
)

add_library(viz STATIC
  src/viz.cpp
  src/KeyBindings.cpp
  src/panels/SidePanel.cpp
  src/panels/ViewportPanel.cpp
  ${CMAKE_SOURCE_DIR}/third_party/imgui_wgpu/imgui_impl_wgpu.cpp
  ${CMAKE_SOURCE_DIR}/third_party/ImGuiFileDialog/ImGuiFileDialog.cpp
)

# Disable clang-tidy for third-party files
set_source_files_properties(
  ${CMAKE_SOURCE_DIR}/third_party/imgui_wgpu/imgui_impl_wgpu.cpp
  PROPERTIES
  SKIP_LINTING ON
  COMPILE_DEFINITIONS "IMGUI_IMPL_WEBGPU_BACKEND_WGPU"
)
set_source_files_properties(
  ${CMAKE_SOURCE_DIR}/third_party/ImGuiFileDialog/ImGuiFileDialog.cpp
  PROPERTIES
  SKIP_LINTING ON
)

target_include_directories(viz PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/third_party
  ${CMAKE_SOURCE_DIR}/third_party/wgpu-native/include
  ${CMAKE_SOURCE_DIR}/third_party/imgui_wgpu
  ${CMAKE_SOURCE_DIR}/sim/include
)
target_compile_definitions(viz PRIVATE IMGUI_IMPL_WEBGPU_BACKEND_WGPU)
target_link_libraries(viz PUBLIC 
  scene 
  sim
  common
  glfw
  imgui::imgui
  ImGuiFileDialog
  wgpu_native
  ${X11_LIBRARIES}
)
setup_warnings(viz)
setup_sanitizers(viz)