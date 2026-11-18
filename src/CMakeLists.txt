configure_file(sifted/version.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/generated/sifted/version.hpp" @ONLY)

# The deterministic core (no threads, sockets or clocks). Split further along
# the module directories only when a real dependency boundary needs enforcing.
add_library(sifted_core STATIC
  sifted/association/assignment.cpp
  sifted/association/gating.cpp
  sifted/association/lifecycle.cpp
  sifted/core/clock.cpp
  sifted/core/engine.cpp
  sifted/core/measurement.cpp
  sifted/core/outcome.cpp
  sifted/core/sensor.cpp
  sifted/estimation/bearing_only.cpp
  sifted/estimation/cv_filter.cpp
  sifted/estimation/geodetic.cpp
  sifted/estimation/measurement_model.cpp
  sifted/estimation/range_bearing.cpp
  sifted/net/dedup.cpp
  sifted/net/impairment.cpp
  sifted/net/ingest.cpp
  sifted/net/snapshot.cpp
  sifted/replay/crc32.cpp
  sifted/replay/event_log.cpp
  sifted/replay/replayer.cpp
  sifted/store/track_store.cpp
  sifted/wire/codec.cpp)

# The threaded, OS-facing edge (sockets, wall clock) is kept out of the
# deterministic core: only the daemon and its tests link it.
add_library(sifted_net STATIC
  sifted/net/durable_file.cpp
  sifted/net/sha1.cpp
  sifted/net/system_clock.cpp
  sifted/net/rate_limit.cpp
  sifted/net/udp.cpp
  sifted/net/websocket.cpp)
add_library(sifted::net ALIAS sifted_net)
find_package(Threads REQUIRED)
target_link_libraries(sifted_net PUBLIC sifted::core Threads::Threads PRIVATE sifted_options)
add_library(sifted::core ALIAS sifted_core)

target_include_directories(sifted_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(sifted_core
  PUBLIC  Eigen3::Eigen
  PRIVATE sifted_options)

add_subdirectory(apps)
