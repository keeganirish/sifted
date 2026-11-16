# Dependencies: prefer the system's copy (reproducible via the distro and CI's
# apt step). If SIFTED_FETCH_DEPS is ON, fall back to downloading pinned tags.

# Eigen 3.4 or newer. Eigen 5 (Homebrew, newer distributions) only accepts a
# request for its own major version, and Eigen 3.4's package rejects version
# ranges that go past 3.x, so ask both ways.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_package(Eigen3 3.4...<100 QUIET NO_MODULE)
endif()
if(Eigen3_FOUND)
  message(STATUS "sifted: using Eigen ${Eigen3_VERSION}")
  # GCC reports a false "potential null pointer dereference" inside Eigen 5's
  # inlined functors at -O2/-O3 (-isystem does not cover warnings raised after
  # inlining). Keep the warning, but don't let it fail -Werror builds.
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND Eigen3_VERSION VERSION_GREATER_EQUAL 5)
    target_compile_options(sifted_options INTERFACE -Wno-error=null-dereference)
  endif()
endif()
if(NOT Eigen3_FOUND AND SIFTED_FETCH_DEPS)
  include(FetchContent)
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(eigen)
  # Treat the downloaded headers like a system install (-isystem), so the
  # project's strict warnings (and -Werror) apply only to code we own.
  get_target_property(_eigen_inc eigen INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(eigen PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_eigen_inc}")
elseif(NOT Eigen3_FOUND)
  message(FATAL_ERROR
    "Eigen 3.4 or newer not found. Install it (e.g. 'apt install libeigen3-dev', "
    "'brew install eigen') "
    "or configure with -DSIFTED_FETCH_DEPS=ON.")
endif()

if(SIFTED_BUILD_TESTS)
  find_package(GTest QUIET)
  if(NOT GTest_FOUND AND SIFTED_FETCH_DEPS)
    include(FetchContent)
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        v1.14.0
      GIT_SHALLOW    TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  elseif(NOT GTest_FOUND)
    message(FATAL_ERROR
      "GoogleTest not found. Install it (e.g. 'apt install libgtest-dev'), "
      "configure with -DSIFTED_FETCH_DEPS=ON, or pass -DSIFTED_BUILD_TESTS=OFF.")
  endif()
endif()
