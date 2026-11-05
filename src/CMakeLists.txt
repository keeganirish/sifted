configure_file(sifted/version.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/generated/sifted/version.hpp" @ONLY)

# One library for now. Split along the module directories (wire, estimation,
# association, ...) only when a real dependency boundary needs enforcing.
add_library(sifted_core STATIC
  sifted/core/clock.cpp
  sifted/core/engine.cpp
  sifted/core/measurement.cpp
  sifted/wire/codec.cpp)
add_library(sifted::core ALIAS sifted_core)

target_include_directories(sifted_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(sifted_core
  PUBLIC  Eigen3::Eigen
  PRIVATE sifted_options)

add_subdirectory(apps)
