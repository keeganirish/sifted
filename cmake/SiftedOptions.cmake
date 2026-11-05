# Defines `sifted_options`: an INTERFACE target carrying the project's compiler
# flags (warnings, determinism flags, sanitizers). Link it PRIVATE to every
# target we own; never to third-party code.

add_library(sifted_options INTERFACE)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(sifted_options INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
    -Wnon-virtual-dtor -Woverloaded-virtual -Wcast-align -Wformat=2
    -Wnull-dereference -Wdouble-promotion)
  if(SIFTED_WERROR)
    target_compile_options(sifted_options INTERFACE -Werror)
  endif()

  # Determinism: replay must be bit-for-bit identical, so floating point must
  # not depend on the compiler's freedom to fuse or reassociate operations.
  # -ffast-math is never acceptable here; -ffp-contract=off forbids FMA
  # contraction, which differs between targets and compilers.
  target_compile_options(sifted_options INTERFACE -ffp-contract=off)
elseif(MSVC)
  target_compile_options(sifted_options INTERFACE /W4 /fp:strict)
  if(SIFTED_WERROR)
    target_compile_options(sifted_options INTERFACE /WX)
  endif()
endif()

# Sanitizers. Frame pointers keep stack traces readable.
if(SIFTED_SANITIZE)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "SIFTED_SANITIZE requires GCC or Clang")
  endif()
  string(REPLACE ";" "," _san "${SIFTED_SANITIZE}")
  message(STATUS "sifted: sanitizers enabled: ${_san}")
  target_compile_options(sifted_options INTERFACE
    -fsanitize=${_san} -fno-omit-frame-pointer -fno-sanitize-recover=all)
  target_link_options(sifted_options INTERFACE -fsanitize=${_san})
endif()

# Fuzzing: instrument everything we own for coverage, without linking in
# libFuzzer's main(); only fuzz targets link the engine.
if(SIFTED_BUILD_FUZZ)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "SIFTED_BUILD_FUZZ requires Clang")
  endif()
  target_compile_options(sifted_options INTERFACE -fsanitize=fuzzer-no-link)
endif()

if(SIFTED_CLANG_TIDY)
  find_program(SIFTED_CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
  set(CMAKE_CXX_CLANG_TIDY "${SIFTED_CLANG_TIDY_EXE}")
endif()
