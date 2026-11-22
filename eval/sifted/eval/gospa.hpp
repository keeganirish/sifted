#pragma once

// Generalized Optimal Sub-Pattern Assignment (GOSPA) metric with alpha = 2
// (Rahmathullah, Garcia-Fernandez and Svensson, "Generalized optimal
// sub-pattern assignment metric", FUSION 2017, arXiv:1601.05585).
//
// Definition 1 of the paper, for finite sets X, Y of points, a base metric d,
// cut-off c > 0, order 1 <= p < inf and 0 < alpha <= 2, with
// d^(c)(x, y) = min(d(x, y), c) and |X| <= |Y|:
//
//   d_p^(c,alpha)(X, Y) = ( min_pi sum_{i <= |X|} d^(c)(x_i, y_pi(i))^p
//                           + (c^p / alpha) (|Y| - |X|) )^(1/p),
//
// symmetric in X and Y. For alpha = 2 (Proposition 1) it is the minimum over
// one-to-one partial assignments gamma of X to Y:
//
//   d^p = min_gamma [ sum_{(i,j) in gamma} d(x_i, y_j)^p
//                     + (c^p / 2) (|X| + |Y| - 2 |gamma|) ],
//
// which splits into localisation (the sum over assigned pairs), missed
// targets (|X| - |gamma| of them, c^p/2 each) and false tracks
// (|Y| - |gamma|, c^p/2 each). A pair further apart than c is never worth
// assigning (it costs d^p >= c^p, leaving both out costs c^p), so the optimum
// is found by association::solve_assignment with pair cost d^p for d < c,
// pairs at d >= c forbidden, and c^p as the cost of leaving a row
// unassigned: total = sum_gamma (d^p - c^p) + (c^p/2)(|X| + |Y|), the same
// objective up to a constant. alpha = 2 is the value the paper recommends for
// tracking: it penalises missed and false targets, not only localisation.
//
// The base metric here is the Euclidean distance between 2-D positions.

#include <cstddef>
#include <vector>

namespace sifted::eval {

struct GospaParams {
  // Cut-off [m]: an error beyond it counts as one missed target plus one false
  // track. Default = kMatchDistanceM (scenario.hpp) so GOSPA's missed / false
  // decomposition uses the same "is this the same object" threshold as the
  // CLEAR-MOT matcher; sized by the same rule (about 5x the filtered radar
  // error at the edge of coverage, below the closest lane spacing).
  double c = 100.0;
  // Order: 2, like the RMSE, so large localisation errors weigh as in RMSE.
  double p = 2.0;
};

// True iff c is finite and > 0 and p is finite and >= 1.
bool is_valid(const GospaParams& g) noexcept;

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

// One evaluation. `total` is d (metres); the components are in d^p units
// (m^p) and add up to total^p exactly up to rounding.
struct Gospa {
  double total = 0.0;
  double localisation = 0.0;  // sum of d^p over assigned pairs
  double missed = 0.0;        // (c^p / 2) * missed_count
  double false_tracks = 0.0;  // (c^p / 2) * false_count
  std::size_t assigned = 0;
  std::size_t missed_count = 0;
  std::size_t false_count = 0;
  // Per truth point, the assigned estimate index, or -1.
  std::vector<int> assignment;
};

// GOSPA between truth set `x` and estimate set `y`. Either may be empty.
// Throws std::invalid_argument if !is_valid(params) or a coordinate is not
// finite.
Gospa gospa(const std::vector<Point2>& x, const std::vector<Point2>& y,
            const GospaParams& params = {});

// Sum of GOSPA evaluations over instants (and runs), for time averages.
struct GospaSum {
  std::size_t instants = 0;
  double sum_p = 0.0;  // sum of total^p
  double localisation = 0.0;
  double missed = 0.0;
  double false_tracks = 0.0;
  std::size_t missed_count = 0;
  std::size_t false_count = 0;

  void add(const Gospa& g, double p);
  void merge(const GospaSum& o);
  // (mean over instants of d^p)^(1/p): for p = 2 the RMS GOSPA [m]. 0 if empty.
  double mean_total(double p) const;
};

}  // namespace sifted::eval
