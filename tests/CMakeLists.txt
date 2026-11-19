include(GoogleTest)

add_executable(sifted_tests
  association_test.cpp
  bearing_only_test.cpp
  clock_test.cpp
  consistency_test.cpp
  engine_test.cpp
  estimation_test.cpp
  eval_test.cpp
  fault_injection_test.cpp
  event_log_test.cpp
  degraded_network_test.cpp
  gate_screen_test.cpp
  geodetic_test.cpp
  impairment_test.cpp
  ingest_test.cpp
  live_reliability_test.cpp
  measurement_test.cpp
  multi_target_regression_test.cpp
  realdata_test.cpp
  reorder_test.cpp
  scenario_regression_test.cpp
  sensor_table_test.cpp
  track_store_test.cpp
  udp_flood_test.cpp
  websocket_test.cpp
  wire_codec_test.cpp)
target_link_libraries(sifted_tests PRIVATE sifted::core sifted::net sifted::eval_support sifted_options
  GTest::gtest_main)
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

# Eval.* runs the sifted_eval binary on a small matrix and checks it against
# the scores the regression tests compute (tests/eval_test.cpp).
if(TARGET sifted_eval)
  add_dependencies(sifted_tests sifted_eval)
  target_compile_definitions(sifted_tests PRIVATE SIFTED_EVAL_BIN="$<TARGET_FILE:sifted_eval>")
endif()

# Real-data evaluation end to end (sim/tests/test_realeval.py EndToEnd): a
# synthetic OpenSky-format archive through sifted_sim.realeval, sifted_realeval
# and sifted_eval, twice, byte-identical. Offline; skipped by the plain Python
# run above, which has no binaries.
if(Python3_FOUND AND TARGET sifted_eval AND TARGET sifted_realeval)
  add_dependencies(sifted_tests sifted_realeval)
  add_test(NAME realdata_e2e_python
    COMMAND "${Python3_EXECUTABLE}" -m unittest tests.test_realeval.EndToEnd
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/sim")
  set_tests_properties(realdata_e2e_python PROPERTIES ENVIRONMENT
    "PYTHONPATH=${PROJECT_SOURCE_DIR}/sim;SIFTED_BIN_DIR=$<TARGET_FILE_DIR:sifted_realeval>")
endif()
