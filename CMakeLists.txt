cmake_minimum_required(VERSION 3.23)

# ---- Project ----
project(lilsim
  VERSION 0.1.0
  LANGUAGES CXX
)

# ---- Fetch wgpu-native if not present ----
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/third_party/wgpu-native/lib")
  message(STATUS "Downloading wgpu-native...")
  include(FetchContent)
  
  # Detect platform
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(WGPU_URL "https://github.com/gfx-rs/wgpu-native/releases/download/v27.0.2.0/wgpu-linux-x86_64-release.zip")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
      set(WGPU_URL "https://github.com/gfx-rs/wgpu-native/releases/download/v27.0.2.0/wgpu-macos-aarch64-release.zip")
    else()
      set(WGPU_URL "https://github.com/gfx-rs/wgpu-native/releases/download/v27.0.2.0/wgpu-macos-x86_64-release.zip")
    endif()
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(WGPU_URL "https://github.com/gfx-rs/wgpu-native/releases/download/v27.0.2.0/wgpu-windows-x86_64-release.zip")
  else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
  endif()
  
  FetchContent_Declare(
    wgpu_native
    URL ${WGPU_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/third_party/wgpu-native
  )
  FetchContent_MakeAvailable(wgpu_native)
  message(STATUS "wgpu-native downloaded successfully")
endif()

# ---- Options ----
option(BUILD_TESTS "Build tests" OFF)
option(ENABLE_CLANG_TIDY "Run clang-tidy during builds" ON)
option(ENABLE_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(ENABLE_SANITIZERS "Enable ASan/UBSan in Debug/RelWithDebInfo" ON)
option(ENABLE_LTO "Enable interprocedural optimization (LTO) in Release" OFF)
option(ENABLE_PCH "Enable precompiled headers for faster builds" OFF)

# Prefer Release if not provided
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "RelWithDebInfo" "Release" "MinSizeRel")
endif()

# ---- C++ standard ----
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Always export compile_commands.json for IDE tooling and clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "" FORCE)

# ---- ccache (optional) ----
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# ---- Helper: warnings ----
function(setup_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
    if(ENABLE_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    if(ENABLE_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

# ---- Helper: sanitizers ----
function(setup_sanitizers target)
  if(ENABLE_SANITIZERS AND NOT MSVC)
    if(CMAKE_BUILD_TYPE MATCHES "Debug|RelWithDebInfo")
      target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
      target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endif()
  endif()
endfunction()

# ---- LTO ----
include(CheckIPOSupported)
if(ENABLE_LTO)
  check_ipo_supported(RESULT ipo_supported OUTPUT ipo_msg)
  if(ipo_supported)
    message(STATUS "IPO/LTO enabled for Release")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
  else()
    message(WARNING "IPO not supported: ${ipo_msg}")
  endif()
endif()

# ---- clang-tidy ----
if(ENABLE_CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES clang-tidy)
  if(CLANG_TIDY_EXE)
    set(CLANG_TIDY_EXTRA_ARGS "")
    
    # When using clang, clang-tidy doesn't need special gcc include paths
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      # Find g++ include paths for clang-tidy when using gcc
      execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -E -x c++ - -v
        INPUT_FILE /dev/null
        ERROR_VARIABLE GXX_INCLUDES
        OUTPUT_QUIET
      )
      string(REGEX MATCH "#include <...> search starts here:\n(.+)\nEnd of search list" _ ${GXX_INCLUDES})
      set(GXX_INCLUDE_PATHS ${CMAKE_MATCH_1})
      
      # Extract the include paths and add them as extra args
      string(REGEX MATCHALL " ([^\n]+)" GXX_INCLUDE_LINES ${GXX_INCLUDE_PATHS})
      foreach(INCLUDE_LINE ${GXX_INCLUDE_LINES})
        string(STRIP ${INCLUDE_LINE} INCLUDE_PATH)
        if(INCLUDE_PATH)
          list(APPEND CLANG_TIDY_EXTRA_ARGS "--extra-arg=-isystem${INCLUDE_PATH}")
        endif()
      endforeach()
      
      list(APPEND CLANG_TIDY_EXTRA_ARGS "--extra-arg-before=--driver-mode=g++")
    else()
      # When using clang, tell clang-tidy to use libc++ if that's what we're using
      if(CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
        list(APPEND CLANG_TIDY_EXTRA_ARGS "--extra-arg=-stdlib=libc++")
      endif()
    endif()
    
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}" ${CLANG_TIDY_EXTRA_ARGS})
    message(STATUS "clang-tidy enabled: ${CLANG_TIDY_EXE}")
  else()
    message(WARNING "clang-tidy not found; set ENABLE_CLANG_TIDY=OFF or install it")
  endif()
endif()


# ---- Subdirectories ----
add_subdirectory(common)
add_subdirectory(comm)
add_subdirectory(scene)
add_subdirectory(sim)
add_subdirectory(viz)
add_subdirectory(app)
add_subdirectory(models/cars)


# ---- Tests (optional) ----
if(BUILD_TESTS)
  enable_testing()
  # add_subdirectory(tests)  # add when ready
endif()
