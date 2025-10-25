add_executable(lilsim
  src/main.cpp
)

find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

target_link_libraries(lilsim PRIVATE sim scene common viz glfw imgui::imgui OpenGL::GL)
setup_warnings(lilsim)
setup_sanitizers(lilsim)

# Generate compile_commands.json for clang-tidy/IDE
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
