#include "sifted/core/outcome.hpp"

namespace sifted {

std::string_view to_string(MeasurementOutcome o) noexcept {
  switch (o) {
    case MeasurementOutcome::Initialized:
      return "initialized";
    case MeasurementOutcome::Updated:
      return "updated";
    case MeasurementOutcome::Invalid:
      return "invalid";
    case MeasurementOutcome::UnknownSensor:
      return "unknown sensor";
    case MeasurementOutcome::KindMismatch:
      return "kind mismatch";
    case MeasurementOutcome::UnsupportedKind:
      return "unsupported kind";
    case MeasurementOutcome::Duplicate:
      return "duplicate";
    case MeasurementOutcome::OutOfOrder:
      return "out of order";
    case MeasurementOutcome::FilterRejected:
      return "filter rejected";
    case MeasurementOutcome::Unassociated:
      return "unassociated";
    case MeasurementOutcome::TrackLimit:
      return "track limit";
    case MeasurementOutcome::OutOfFrame:
      return "out of frame";
  }
  return "unknown outcome";
}

}  // namespace sifted
