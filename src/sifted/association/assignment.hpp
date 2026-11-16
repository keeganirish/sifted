#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>

// Optimal (global nearest neighbour) assignment of measurements to tracks.
//
// Rows are measurements and columns are tracks. Each row is assigned to at most
// one column and each column to at most one row; a row may also stay
// unassigned at a fixed cost. The solver minimises the total cost exactly
// (Kuhn-Munkres / Hungarian with potentials, O(n^2 m)), so it resolves
// crossing and close-pass ambiguities that a greedy nearest-neighbour pass gets
// wrong. Forbidden pairs (outside the gate) have cost kForbidden.
//
// Determinism: the algorithm scans columns in index order and only replaces a
// candidate on a strictly smaller value, so among equal-cost solutions it
// picks the one favouring lower column indices. Callers order columns by track
// id, which makes "ties are broken by track id" hold for the assignment too.
namespace sifted::association {

inline constexpr double kForbidden = std::numeric_limits<double>::infinity();
inline constexpr int kUnassigned = -1;

// `cost` is row-major, rows x cols, each entry finite and >= 0 or kForbidden.
// `unassigned_cost` (finite, >= 0) is what leaving one row unassigned costs;
// a pair is only ever chosen if that lowers the total. Returns, for each row,
// the assigned column or kUnassigned.
std::vector<int> solve_assignment(const std::vector<double>& cost, std::size_t rows,
                                  std::size_t cols, double unassigned_cost);

// Greedy nearest neighbour, for comparison only (ADR 0004): repeatedly takes
// the cheapest remaining pair (ties: lowest row, then lowest column) while it
// costs less than leaving its row unassigned. Same arguments and result as
// solve_assignment; not optimal.
std::vector<int> solve_assignment_greedy(const std::vector<double>& cost, std::size_t rows,
                                         std::size_t cols, double unassigned_cost);

// Which solver the engine uses. Global nearest neighbour is the default and
// the only one meant for operation; Greedy exists so tests and benchmarks can
// show what the global solve buys.
enum class AssignmentMethod : std::uint8_t { GlobalNearestNeighbour, Greedy };
std::string_view to_string(AssignmentMethod m) noexcept;

// Splits a bipartite candidate graph into connected components so each can be
// solved independently (far cheaper than one large problem, same answer).
// `pairs` lists (row, col) candidate edges. Returns components as lists of
// rows and cols, each sorted ascending, ordered by their smallest row. Rows
// with no edge are not part of any component.
struct Component {
  std::vector<std::size_t> rows;
  std::vector<std::size_t> cols;
};
std::vector<Component> connected_components(
    const std::vector<std::pair<std::size_t, std::size_t>>& pairs, std::size_t rows,
    std::size_t cols);

}  // namespace sifted::association
