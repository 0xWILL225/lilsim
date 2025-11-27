# Where built plugins end up relative to the build tree
set(LILSIM_CAR_PLUGIN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/models/cars")

# A convenience macro for each model
function(lilsim_add_car_model NAME SRC)
  add_library(${NAME} SHARED ${SRC})
  target_include_directories(${NAME}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  # Put all car plugins in the same folder (build tree)
  set_target_properties(${NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${LILSIM_CAR_PLUGIN_OUTPUT_DIR}"  # Windows .dll
    LIBRARY_OUTPUT_DIRECTORY "${LILSIM_CAR_PLUGIN_OUTPUT_DIR}"  # Linux .so / macOS .dylib
  )

  # Install rule so releases have a fixed relative path:
  install(TARGETS ${NAME}
    RUNTIME DESTINATION models/cars  # Windows
    LIBRARY DESTINATION models/cars  # Linux/macOS
  )
endfunction()

# Your built-in model(s)
lilsim_add_car_model(lilsim_model_kinematic_single_track
  src/kinematic_single_track.cpp
)

# Later:
# lilsim_add_car_model(lilsim_model_whatever src/whatever.cpp)
