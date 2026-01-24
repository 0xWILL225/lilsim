add_executable(lilsim
  src/main.cpp
)

target_link_libraries(lilsim PRIVATE sim scene viz)
setup_warnings(lilsim)
setup_sanitizers(lilsim)

# Generate compile_commands.json for clang-tidy/IDE
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Copy the default GUI config to the executable path
add_custom_command(
  TARGET lilsim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_SOURCE_DIR}/viz/resources/lilsim_gui_config.default.yaml
          $<TARGET_FILE_DIR:lilsim>/lilsim_gui_config.yaml
)
