add_library(scene STATIC
  src/scene.cpp
)
target_include_directories(scene PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(scene PUBLIC common)
setup_warnings(scene)
setup_sanitizers(scene)
