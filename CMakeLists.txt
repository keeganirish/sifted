cmake_minimum_required(VERSION 3.21)

project(sifted
  VERSION 0.1.0
  DESCRIPTION "Deterministic multi-sensor track fusion"
  LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(SIFTED_BUILD_TESTS "Build unit and integration tests" ON)
option(SIFTED_BUILD_FUZZ  "Build libFuzzer targets (requires Clang)" OFF)
option(SIFTED_WERROR      "Treat warnings as errors" OFF)
option(SIFTED_CLANG_TIDY  "Run clang-tidy during the build" OFF)
option(SIFTED_FETCH_DEPS  "Download pinned dependencies if not found on the system" OFF)
set(SIFTED_SANITIZE "" CACHE STRING
  "Sanitizers to enable, e.g. 'address;undefined' or 'thread' (not combinable)")

include(SiftedOptions)       # defines the sifted_options interface target
include(SiftedDependencies)

add_subdirectory(src)

if(SIFTED_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(SIFTED_BUILD_FUZZ)
  add_subdirectory(fuzz)
endif()
