#include "sifted/association/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>

namespace sifted::association {

std::vector<int> solve_assignment(const std::vector<double>& cost, std::size_t rows,
                                  std::size_t cols, double unassigned_cost) {
  std::vector<int> result(rows, kUnassigned);
  if (rows == 0) {
    return result;
  }
  // Augment with one "stay unassigned" dummy column per row, so a full
  // assignment of rows always exists. Forbidden pairs get a finite cost large
  // enough that any solution using one is worse than leaving every row
  // unassigned.
  double max_cost = unassigned_cost;
  for (double c : cost) {
    if (c != kForbidden) {
      max_cost = std::max(max_cost, c);
    }
  }
  const double n_d = static_cast<double>(rows);
  const double big = (max_cost + 1.0) * (n_d + 1.0) * 2.0;
  const std::size_t m = cols + rows;
  auto at = [&](std::size_t r, std::size_t c) -> double {  // r in [0, rows), c in [0, m)
    if (c >= cols) {
      return unassigned_cost;
    }
    const double v = cost[r * cols + c];
    return v == kForbidden ? big : v;
  };

  // Hungarian algorithm with potentials (1-based arrays; index 0 is a sentinel).
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(rows + 1, 0.0);
  std::vector<double> v(m + 1, 0.0);
  std::vector<std::size_t> p(m + 1, 0);    // p[j]: row matched to column j
  std::vector<std::size_t> way(m + 1, 0);  // previous column on the augmenting path
  for (std::size_t i = 1; i <= rows; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::vector<double> minv(m + 1, inf);
    std::vector<char> used(m + 1, 0);
    do {
      used[j0] = 1;
      const std::size_t i0 = p[j0];
      double delta = inf;
      std::size_t j1 = 0;
      for (std::size_t j = 1; j <= m; ++j) {
        if (used[j] != 0) {
          continue;
        }
        const double cur = at(i0 - 1, j - 1) - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (std::size_t j = 0; j <= m; ++j) {
        if (used[j] != 0) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  for (std::size_t j = 1; j <= cols; ++j) {
    if (p[j] != 0 && cost[(p[j] - 1) * cols + (j - 1)] != kForbidden) {
      result[p[j] - 1] = static_cast<int>(j - 1);
    }
  }
  return result;
}

std::vector<Component> connected_components(
    const std::vector<std::pair<std::size_t, std::size_t>>& pairs, std::size_t rows,
    std::size_t cols) {
  // Union-find over rows [0, rows) and cols [rows, rows + cols).
  std::vector<std::size_t> parent(rows + cols);
  std::iota(parent.begin(), parent.end(), std::size_t{0});
  auto find = [&parent](std::size_t x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };
  std::vector<char> has_edge(rows + cols, 0);
  for (const auto& [r, c] : pairs) {
    const std::size_t a = find(r);
    const std::size_t b = find(rows + c);
    if (a != b) {
      parent[std::max(a, b)] = std::min(a, b);  // smallest index is the root
    }
    has_edge[r] = 1;
    has_edge[rows + c] = 1;
  }
  std::vector<Component> out;
  std::vector<std::size_t> slot(rows + cols, SIZE_MAX);
  for (std::size_t r = 0; r < rows; ++r) {
    if (has_edge[r] == 0) {
      continue;
    }
    const std::size_t root = find(r);
    if (slot[root] == SIZE_MAX) {
      slot[root] = out.size();
      out.emplace_back();
    }
    out[slot[root]].rows.push_back(r);
  }
  for (std::size_t c = 0; c < cols; ++c) {
    if (has_edge[rows + c] != 0) {
      out[slot[find(rows + c)]].cols.push_back(c);
    }
  }
  return out;
}

std::vector<int> solve_assignment_greedy(const std::vector<double>& cost, std::size_t rows,
                                         std::size_t cols, double unassigned_cost) {
  std::vector<int> out(rows, kUnassigned);
  std::vector<char> col_taken(cols, 0);
  while (true) {
    std::size_t best_r = rows;
    std::size_t best_c = cols;
    double best = unassigned_cost;
    for (std::size_t r = 0; r < rows; ++r) {
      if (out[r] != kUnassigned) {
        continue;
      }
      for (std::size_t c = 0; c < cols; ++c) {
        const double v = cost[r * cols + c];
        if (col_taken[c] == 0 && v < best) {
          best = v;
          best_r = r;
          best_c = c;
        }
      }
    }
    if (best_r == rows) {
      return out;
    }
    out[best_r] = static_cast<int>(best_c);
    col_taken[best_c] = 1;
  }
}

std::string_view to_string(AssignmentMethod m) noexcept {
  switch (m) {
    case AssignmentMethod::GlobalNearestNeighbour:
      return "gnn";
    case AssignmentMethod::Greedy:
      return "greedy";
  }
  return "unknown";
}

}  // namespace sifted::association
