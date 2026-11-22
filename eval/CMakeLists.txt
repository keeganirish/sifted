# Ground-truth evaluation (docs/evaluation.md).
#
# sifted_eval_support: scenario loading, CLEAR-MOT style scoring, filter
# consistency, impairment levels and fault cases. The one implementation that
# both the tests and sifted_eval use. Deterministic and pure: it links only
# sifted::core (no sockets, threads or clocks), like the code it evaluates.
add_library(sifted_eval_support STATIC
  sifted/eval/conditions.cpp
  sifted/eval/consistency.cpp
  sifted/eval/gospa.cpp
  sifted/eval/realdata.cpp
  sifted/eval/scenario.cpp
  sifted/eval/stats.cpp
  sifted/eval/track_report.cpp)
add_library(sifted::eval_support ALIAS sifted_eval_support)
target_include_directories(sifted_eval_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sifted_eval_support PUBLIC sifted::core PRIVATE sifted_options)

# The scorecard tool. Developer tooling like the benchmarks, so the app preset
# (what install.sh builds) leaves it out:
#   cmake --preset release && cmake --build --preset release
#   build/release/eval/sifted_eval --out docs/evaluation.md
if(SIFTED_BUILD_EVAL)
  add_executable(sifted_eval sifted_eval.cpp)
  target_link_libraries(sifted_eval PRIVATE sifted::eval_support sifted_options)
  target_compile_definitions(sifted_eval PRIVATE
    SIFTED_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/tests/fixtures/scenarios")

  # Real-data evaluation A (withheld ADS-B reports; docs/real-data-evaluation.md),
  # driven by `python3 -m sifted_sim.realeval`, which also runs sifted_eval
  # for evaluation B.
  add_executable(sifted_realeval sifted_realeval.cpp)
  target_link_libraries(sifted_realeval PRIVATE sifted::eval_support sifted_options)

  # Legacy constant-velocity filter against the IMM (ADR 0014;
  # docs/motion-model-evaluation.md).
  add_executable(sifted_motion_eval sifted_motion_eval.cpp)
  target_link_libraries(sifted_motion_eval PRIVATE sifted::eval_support sifted_options)

  # Monte Carlo evaluation and baseline comparison (ADR 0015;
  # docs/evaluation-monte-carlo.md, docs/evaluation-baseline.md), driven by
  # `python3 -m sifted_sim.montecarlo report` and
  # benchmarks/baselines/stonesoup/compare.py.
  add_executable(sifted_montecarlo sifted_montecarlo.cpp)
  target_link_libraries(sifted_montecarlo PRIVATE sifted::eval_support sifted_options)
endif()
