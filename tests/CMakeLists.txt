include(GoogleTest)

add_executable(sifted_tests
  association_test.cpp
  bearing_only_test.cpp
  clock_test.cpp
  engine_test.cpp
  estimation_test.cpp
  fault_injection_test.cpp
  event_log_test.cpp
  degraded_network_test.cpp
  gate_screen_test.cpp
  geodetic_test.cpp
  impairment_test.cpp
  ingest_test.cpp
  measurement_test.cpp
  multi_target_regression_test.cpp
  reorder_test.cpp
  scenario_support.cpp
  scenario_regression_test.cpp
  sensor_table_test.cpp
  track_store_test.cpp
  udp_flood_test.cpp
  websocket_test.cpp
  wire_codec_test.cpp)
target_link_libraries(sifted_tests PRIVATE sifted::core sifted::net sifted_options GTest::gtest_main)
target_compile_definitions(sifted_tests PRIVATE
  SIFTED_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
gtest_discover_tests(sifted_tests)

# Cross-language wire contract: the Python encoder must match the same golden
# fixture the C++ codec is tested against.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_FOUND)
  add_test(NAME sim_wire_python
    COMMAND "${Python3_EXECUTABLE}" -m unittest discover -s tests
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/sim")
  set_tests_properties(sim_wire_python PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_SOURCE_DIR}/sim")
endif()
