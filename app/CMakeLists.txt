add_executable(lilsim
  src/main.cpp
)

target_link_libraries(lilsim PRIVATE sim scene viz)
setup_warnings(lilsim)
setup_sanitizers(lilsim)

# Generate compile_commands.json for clang-tidy/IDE
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
