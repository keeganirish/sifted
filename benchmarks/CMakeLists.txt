# Benchmarks (ADR 0011). Built in every preset so they cannot rot, but only
# meaningful from the release preset:
#   cmake --preset release && cmake --build --preset release
#   build/release/benchmarks/sifted_bench all > benchmarks/results/engine.csv
# Timing code lives here only; the core never reads a clock.
add_executable(sifted_bench bench_engine.cpp)
target_link_libraries(sifted_bench PRIVATE sifted::core sifted_options)
