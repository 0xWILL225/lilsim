add_library(common STATIC
  src/SE2.cpp
)
target_include_directories(common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
setup_warnings(common)
setup_sanitizers(common)

if(ENABLE_PCH)
  target_precompile_headers(common PRIVATE <vector> <string> <cmath>)
endif()

# Example: Eigen is header-only via vcpkg
find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(common PUBLIC Eigen3::Eigen)
