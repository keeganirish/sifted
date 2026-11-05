add_executable(fuzz_decode fuzz_decode.cpp)
target_link_libraries(fuzz_decode PRIVATE sifted::core sifted_options)
target_link_options(fuzz_decode PRIVATE -fsanitize=fuzzer)

# Note: libFuzzer writes newly found inputs into the FIRST corpus directory it
# is given. Run with a scratch dir first, e.g.
#   mkdir -p build/fuzz-work && fuzz_decode build/fuzz-work fuzz/corpus
# and copy only interesting inputs into fuzz/corpus/ by hand.
