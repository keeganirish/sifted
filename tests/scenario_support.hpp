#pragma once

// The tests' view of the shared evaluation library (eval/sifted/eval/): the
// scenario loading, CLEAR-MOT style scoring, consistency statistics,
// impairment levels and fault cases are defined there, once, for the tests
// and for sifted_eval alike. This header only adds the fixture lookup by
// name.

#include <string>

#include "sifted/eval/conditions.hpp"
#include "sifted/eval/consistency.hpp"
#include "sifted/eval/scenario.hpp"

namespace sifted::testing {

using namespace sifted::eval;  // NOLINT(google-build-using-namespace)

// tests/fixtures/scenarios/<name>.
inline Scenario load_scenario(const std::string& name) {
  return load_scenario_dir(std::string(SIFTED_FIXTURE_DIR) + "/scenarios/" + name);
}

}  // namespace sifted::testing
