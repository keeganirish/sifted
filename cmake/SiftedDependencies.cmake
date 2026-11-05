# Dependencies: prefer the system's copy (reproducible via the distro and CI's
# apt step). If SIFTED_FETCH_DEPS is ON, fall back to downloading pinned tags.

find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT Eigen3_FOUND AND SIFTED_FETCH_DEPS)
  include(FetchContent)
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(eigen)
elseif(NOT Eigen3_FOUND)
  message(FATAL_ERROR
    "Eigen 3.4+ not found. Install it (e.g. 'apt install libeigen3-dev') "
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
