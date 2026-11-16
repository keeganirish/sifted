add_executable(fuzz_decode fuzz_decode.cpp)
target_link_libraries(fuzz_decode PRIVATE sifted::core sifted_options)
target_link_options(fuzz_decode PRIVATE -fsanitize=fuzzer)

# Note: libFuzzer writes newly found inputs into the FIRST corpus directory it
# is given. Run with a scratch dir first, e.g.
#   mkdir -p build/fuzz-work && fuzz_decode build/fuzz-work fuzz/corpus
# and copy only interesting inputs into fuzz/corpus/ by hand.

add_executable(fuzz_event_log fuzz_event_log.cpp)
target_link_libraries(fuzz_event_log PRIVATE sifted::core sifted_options)
target_link_options(fuzz_event_log PRIVATE -fsanitize=fuzzer)
# Seed corpus: fuzz/corpus_log/ (a small recorded log). Same scratch-dir rule as above.

# M3: every remaining parser of untrusted or semi-trusted input, and the
# engine itself (structure-aware). Seed corpora: fuzz/corpus_<name>/.
foreach(target fuzz_sensor_table fuzz_log_header fuzz_websocket fuzz_engine)
  add_executable(${target} ${target}.cpp)
  target_link_libraries(${target} PRIVATE sifted::core sifted_options)
  target_link_options(${target} PRIVATE -fsanitize=fuzzer)
endforeach()
# The WebSocket parsers live in the OS-facing library.
target_link_libraries(fuzz_websocket PRIVATE sifted::net)
