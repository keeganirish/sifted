#include "sifted/eval/track_report.hpp"

#include <Eigen/Cholesky>
#include <array>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <system_error>

namespace sifted::eval {
namespace {

constexpr std::size_t kColumns = 18;
constexpr std::size_t kFirstCov = 8;
// Upper-triangle order of p00 .. p33.
constexpr std::array<std::pair<int, int>, 10> kCovIndex = {
    {{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 1}, {1, 2}, {1, 3}, {2, 2}, {2, 3}, {3, 3}}};

[[noreturn]] void fail_at(std::size_t line, const std::string& what) {
  throw TrackReportError("track report line " + std::to_string(line) + ": " + what);
}

template <class T>
T integer(std::string_view s, std::size_t line, const char* what) {
  T v{};
  const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (s.empty() || ec != std::errc{} || p != s.data() + s.size()) {
    fail_at(line, std::string("bad ") + what + ": '" + std::string(s) + "'");
  }
  return v;
}

double real(std::string_view s, std::size_t line, const char* what) {
  double v = 0.0;
  const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (s.empty() || ec != std::errc{} || p != s.data() + s.size() || !std::isfinite(v)) {
    fail_at(line, std::string("bad ") + what + ": '" + std::string(s) + "'");
  }
  return v;
}

std::string_view status_name(association::TrackStatus s) {
  switch (s) {
    case association::TrackStatus::Tentative:
      return "tentative";
    case association::TrackStatus::Confirmed:
      return "confirmed";
    case association::TrackStatus::Coasting:
      return "coasting";
    case association::TrackStatus::Dropped:
      break;
  }
  return "dropped";
}

// Checks one row's values (and its order after `prev`); shared by parse and
// format so both accept exactly the same reports.
void validate(const ReportedTrack& r, const ReportedTrack* prev, bool covariance,
              std::size_t line) {
  if (r.id == 0 || r.id >= (TrackId{1} << 63)) {
    fail_at(line, "track_id must be in [1, 2^63)");
  }
  if (r.status == association::TrackStatus::Dropped) {
    fail_at(line, "status must be confirmed, coasting or tentative");
  }
  if (r.time_ns < 0 || r.last_update_ns < 0 || r.last_update_ns > r.time_ns) {
    fail_at(line, "times must be >= 0 with last_update_ns <= time_ns");
  }
  if (prev != nullptr &&
      (r.time_ns < prev->time_ns || (r.time_ns == prev->time_ns && r.id <= prev->id))) {
    fail_at(line, "rows must be sorted by (time_ns, track_id) without duplicates");
  }
  for (int k = 0; k < 4; ++k) {
    const double bound = k < 2 ? kMaxTrackPositionM : kMaxTrackSpeedMps;
    if (!std::isfinite(r.state(k)) || std::abs(r.state(k)) > bound) {
      fail_at(line, "state component out of bounds");
    }
  }
  if (!covariance) {
    return;
  }
  for (int a = 0; a < 4; ++a) {
    for (int b = 0; b < 4; ++b) {
      const double v = r.covariance(a, b);
      if (!std::isfinite(v) || std::abs(v) > kMaxTrackCovariance || v != r.covariance(b, a)) {
        fail_at(line, "covariance must be finite, bounded and symmetric");
      }
    }
  }
  const Eigen::LLT<estimation::StateCovariance> llt(r.covariance);
  if (llt.info() != Eigen::Success) {
    fail_at(line, "covariance is not positive definite");
  }
}

std::string number(double v) {
  char buf[32];
  const auto res = std::to_chars(buf, buf + sizeof buf, v);
  return std::string(buf, res.ptr);
}

}  // namespace

TrackReport parse_track_report(std::string_view text) {
  if (text.size() > kMaxTrackReportBytes) {
    throw TrackReportError("track report larger than " + std::to_string(kMaxTrackReportBytes) +
                           " bytes");
  }
  TrackReport out;
  std::size_t line_no = 0;
  bool cov_known = false;
  while (!text.empty()) {
    const std::size_t nl = text.find('\n');
    std::string_view line = text.substr(0, nl);
    text.remove_prefix(nl == std::string_view::npos ? text.size() : nl + 1);
    ++line_no;
    if (line.size() > kMaxTrackReportLine) {
      fail_at(line_no, "line longer than " + std::to_string(kMaxTrackReportLine) + " bytes");
    }
    if (!line.empty() && line.back() == '\r') {
      line.remove_suffix(1);
    }
    if (line_no == 1) {
      if (line != kTrackReportHeader) {
        fail_at(1, "expected '" + std::string(kTrackReportHeader) + "'");
      }
      continue;
    }
    if (line_no == 2) {
      if (line != kTrackReportColumns) {
        fail_at(2, "unexpected column line");
      }
      continue;
    }
    if (line.empty()) {
      if (text.empty()) {
        break;  // the final newline
      }
      fail_at(line_no, "empty line");
    }
    if (out.rows.size() >= kMaxTrackReportRows) {
      fail_at(line_no, "more than " + std::to_string(kMaxTrackReportRows) + " rows");
    }
    std::array<std::string_view, kColumns> f;
    std::size_t n = 0;
    std::string_view rest = line;
    while (true) {
      if (n == kColumns) {
        fail_at(line_no, "expected " + std::to_string(kColumns) + " fields");
      }
      const std::size_t c = rest.find(',');
      f[n++] = rest.substr(0, c);
      if (c == std::string_view::npos) {
        break;
      }
      rest.remove_prefix(c + 1);
    }
    if (n != kColumns) {
      fail_at(line_no, "expected " + std::to_string(kColumns) + " fields");
    }
    ReportedTrack r;
    r.time_ns = integer<std::int64_t>(f[0], line_no, "time_ns");
    r.id = integer<TrackId>(f[1], line_no, "track_id");
    if (f[2] == "confirmed") {
      r.status = association::TrackStatus::Confirmed;
    } else if (f[2] == "coasting") {
      r.status = association::TrackStatus::Coasting;
    } else if (f[2] == "tentative") {
      r.status = association::TrackStatus::Tentative;
    } else {
      fail_at(line_no, "bad status: '" + std::string(f[2]) + "'");
    }
    r.last_update_ns = integer<std::int64_t>(f[3], line_no, "last_update_ns");
    for (int k = 0; k < 4; ++k) {
      r.state(k) = real(f[static_cast<std::size_t>(4 + k)], line_no, "state");
    }
    std::size_t empty = 0;
    for (std::size_t k = kFirstCov; k < kColumns; ++k) {
      empty += f[k].empty() ? 1U : 0U;
    }
    if (empty != 0 && empty != kCovIndex.size()) {
      fail_at(line_no, "covariance must be all ten fields or none");
    }
    const bool has_cov = empty == 0;
    if (!cov_known) {
      out.covariance = has_cov;
      cov_known = true;
    } else if (has_cov != out.covariance) {
      fail_at(line_no, "every row must have a covariance, or none");
    }
    if (has_cov) {
      for (std::size_t k = 0; k < kCovIndex.size(); ++k) {
        const double v = real(f[kFirstCov + k], line_no, "covariance");
        r.covariance(kCovIndex[k].first, kCovIndex[k].second) = v;
        r.covariance(kCovIndex[k].second, kCovIndex[k].first) = v;
      }
    }
    validate(r, out.rows.empty() ? nullptr : &out.rows.back(), has_cov, line_no);
    out.rows.push_back(r);
  }
  if (line_no < 2) {
    throw TrackReportError("track report: missing header or column line");
  }
  return out;
}

std::string format_track_report(const TrackReport& report) {
  std::string out;
  out.append(kTrackReportHeader).append("\n").append(kTrackReportColumns).append("\n");
  for (std::size_t i = 0; i < report.rows.size(); ++i) {
    const ReportedTrack& r = report.rows[i];
    validate(r, i ? &report.rows[i - 1] : nullptr, report.covariance, i + 3);
    out += std::to_string(r.time_ns) + "," + std::to_string(r.id) + "," +
           std::string(status_name(r.status)) + "," + std::to_string(r.last_update_ns);
    for (int k = 0; k < 4; ++k) {
      out += "," + number(r.state(k));
    }
    for (const auto& [a, b] : kCovIndex) {
      out += ",";
      if (report.covariance) {
        out += number(r.covariance(a, b));
      }
    }
    out += "\n";
  }
  return out;
}

TrackReport read_track_report_file(const std::string& path) {
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    throw TrackReportError("cannot read " + path + ": " + ec.message());
  }
  if (size > kMaxTrackReportBytes) {
    throw TrackReportError(path + ": larger than " + std::to_string(kMaxTrackReportBytes) +
                           " bytes");
  }
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  if (!f.good() && !f.eof()) {
    throw TrackReportError("cannot read " + path);
  }
  try {
    return parse_track_report(ss.str());
  } catch (const TrackReportError& e) {
    throw TrackReportError(path + ": " + e.what());
  }
}

void append_engine_tracks(TrackReport& report, std::int64_t t_ns, const Engine& engine) {
  report.covariance = true;
  for (const TrackEstimate& e : engine.tracks_at(Timestamp{t_ns})) {  // sorted by id
    ReportedTrack r;
    r.time_ns = t_ns;
    r.id = e.id;
    r.status = e.status;
    r.last_update_ns = e.last_update.ns;
    r.state = e.state;
    // The filter's covariance is symmetric up to rounding; the report holds
    // one triangle. Mirror the lower one: it is the triangle the NEES
    // Cholesky (consistency.cpp) reads, so scores from the file equal the
    // in-process ones bit for bit.
    r.covariance = e.covariance.selfadjointView<Eigen::Lower>();
    report.rows.push_back(r);
  }
}

Score score_track_report(const std::vector<TruthRow>& truth, const TrackReport& report,
                         double match_distance_m, const GospaParams& gospa) {
  Scorer scorer(truth, match_distance_m, gospa);
  const std::vector<std::int64_t>& times = scorer.instants();
  std::size_t next = 0;
  std::set<TrackId> ended;     // reported before, then missing at a truth instant
  std::set<TrackId> previous;  // present at the previous truth instant
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (next < report.rows.size() && report.rows[next].time_ns < times[i]) {
      throw TrackReportError("track report row " + std::to_string(next + 3) + ": time_ns " +
                             std::to_string(report.rows[next].time_ns) + " is not a truth instant");
    }
    std::vector<TrackEstimate> tracks;
    std::set<TrackId> present;
    for (; next < report.rows.size() && report.rows[next].time_ns == times[i]; ++next) {
      const ReportedTrack& r = report.rows[next];
      if (ended.contains(r.id)) {
        throw TrackReportError("track report row " + std::to_string(next + 3) + ": track id " +
                               std::to_string(r.id) + " reappears after it ended (reused id)");
      }
      present.insert(r.id);
      TrackEstimate e;
      e.id = r.id;
      e.status = r.status;
      e.time = Timestamp{r.time_ns};
      e.last_update = Timestamp{r.last_update_ns};
      e.state = r.state;
      e.covariance = r.covariance;
      tracks.push_back(std::move(e));
    }
    for (const TrackId id : previous) {
      if (!present.contains(id)) {
        ended.insert(id);
      }
    }
    previous = std::move(present);
    scorer.score(i, std::move(tracks), report.covariance);
  }
  if (next < report.rows.size()) {
    throw TrackReportError("track report row " + std::to_string(next + 3) + ": time_ns " +
                           std::to_string(report.rows[next].time_ns) +
                           " is not a truth instant (after the last one)");
  }
  return scorer.finish();
}

}  // namespace sifted::eval
