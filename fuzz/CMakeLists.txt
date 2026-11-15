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
