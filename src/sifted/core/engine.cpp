#include "sifted/core/engine.hpp"

namespace sifted {

void Engine::on_measurement(const Measurement& /*m*/) noexcept { ++processed_; }

}  // namespace sifted
