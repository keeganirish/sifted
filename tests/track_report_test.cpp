// Track reports (eval/sifted/eval/track_report.hpp): the interchange format
// external trackers are scored through. Round trip, strict rejection of
// malformed input, the truth-dependent checks, and the refactor's neutrality:
// Sifted's own tracks exported to a file and scored from it give exactly the
// in-process numbers for every metric both paths have.

#include "sifted/eval/track_report.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "scenario_support.hpp"

namespace sifted::eval {
namespace {

const std::string kHead =
    std::string(kTrackReportHeader) + "\n" + std::string(kTrackReportColumns) + "\n";
const std::string kNoCov = ",,,,,,,,,,";
const std::string kCov = ",4,0,0,0,4,0,0,9,0,9";

TrackReport sample(bool covariance) {
  TrackReport r;
  r.covariance = covariance;
  for (const std::int64_t t : {1'000'000'000LL, 2'000'000'000LL}) {
    for (const TrackId id : {TrackId{3}, TrackId{8}}) {
      ReportedTrack row;
      row.time_ns = t;
      row.id = id;
      row.status =
          id == 3 ? association::TrackStatus::Confirmed : association::TrackStatus::Coasting;
      row.last_update_ns = t - 250'000'000;
      row.state << 1203.5 + static_cast<double>(id), -0.1, 70.25, -1e-7;
      row.covariance.setIdentity();
      row.covariance(0, 1) = row.covariance(1, 0) = 0.3;
      r.rows.push_back(row);
    }
  }
  return r;
}

TEST(EvalTrackReport, RoundTripsWithAndWithoutCovariance) {
  for (const bool cov : {true, false}) {
    TrackReport r = sample(cov);
    if (!cov) {
      for (ReportedTrack& row : r.rows) {
        row.covariance.setIdentity();  // what the parser fills in
      }
    }
    const std::string text = format_track_report(r);
    const TrackReport back = parse_track_report(text);
    EXPECT_EQ(back, r);
    EXPECT_EQ(format_track_report(back), text);
  }
  // CR LF and a missing final newline are tolerated.
  const TrackReport crlf = parse_track_report(std::string(kTrackReportHeader) + "\r\n" +
                                              std::string(kTrackReportColumns) + "\r\n" +
                                              "0,1,tentative,0,0,0,0,0" + kNoCov);
  ASSERT_EQ(crlf.rows.size(), 1U);
  EXPECT_FALSE(crlf.covariance);
  EXPECT_EQ(parse_track_report(kHead).rows.size(), 0U);
}

TEST(EvalTrackReport, RejectsMalformedInput) {
  const std::string ok = "5,1,confirmed,5,0,0,0,0";
  const std::vector<std::string> bad = {
      "",                                                             // no header
      "sifted-tracks 2\n" + std::string(kTrackReportColumns) + "\n",  // version
      std::string(kTrackReportHeader) + "\ntime_ns,id\n",             // columns
      kHead + "5,1,confirmed,5,0,0,0\n",                              // too few fields
      kHead + ok + kNoCov + ",\n",                                    // too many fields
      kHead + "5,1,confirmed,5,nan,0,0,0" + kNoCov + "\n",            // non-finite
      kHead + "5,1,confirmed,5,inf,0,0,0" + kNoCov + "\n",
      kHead + "5,1,confirmed,5,1e300,0,0,0" + kNoCov + "\n",                   // out of bounds
      kHead + "5,1,confirmed,5,0x10,0,0,0" + kNoCov + "\n",                    // not decimal
      kHead + "5,1,confirmed,5, 1,0,0,0" + kNoCov + "\n",                      // whitespace
      kHead + "5,0,confirmed,5,0,0,0,0" + kNoCov + "\n",                       // id 0
      kHead + "5,-1,confirmed,5,0,0,0,0" + kNoCov + "\n",                      // negative id
      kHead + "5,1,dropped,5,0,0,0,0" + kNoCov + "\n",                         // status
      kHead + "5,1,confirmed,6,0,0,0,0" + kNoCov + "\n",                       // update after time
      kHead + "-5,1,confirmed,-5,0,0,0,0" + kNoCov + "\n",                     // negative time
      kHead + ok + kNoCov + "\n" + ok + kNoCov + "\n",                         // duplicate
      kHead + "6,1,confirmed,5,0,0,0,0" + kNoCov + "\n" + ok + kNoCov + "\n",  // unsorted
      kHead + "5,2,confirmed,5,0,0,0,0" + kNoCov + "\n" + ok + kNoCov + "\n",  // id order
      kHead + ok + ",4,0,0,0,4,0,0,9,0,\n",                                    // partial covariance
      kHead + ok + kCov + "\n" + "6,1,confirmed,5,0,0,0,0" + kNoCov + "\n",    // mixed
      kHead + ok + ",1,2,0,0,1,0,0,1,0,1\n",  // not positive definite
      kHead + ok + ",-1,0,0,0,1,0,0,1,0,1\n",
      kHead + "\n" + ok + kNoCov + "\n",                    // empty line
      kHead + ok + kNoCov + std::string(2000, ' ') + "\n",  // line too long
  };
  for (const std::string& text : bad) {
    EXPECT_THROW(parse_track_report(text), TrackReportError) << text;
  }
  EXPECT_NO_THROW(parse_track_report(kHead + ok + kCov + "\n"));
}

TEST(EvalTrackReport, FormatRejectsAnInvalidReport) {
  TrackReport r = sample(true);
  std::swap(r.rows[0], r.rows[1]);
  EXPECT_THROW(format_track_report(r), TrackReportError);
}

std::vector<TruthRow> truth_at(std::initializer_list<std::int64_t> times) {
  std::vector<TruthRow> t;
  for (const std::int64_t ns : times) {
    t.push_back({ns, 0.0, 0.0, 1, true, 0.0, 0.0});
  }
  return t;
}

TEST(EvalTrackReport, ScoringChecksTimesAndIdReuse) {
  constexpr std::int64_t kS = 1'000'000'000;
  const auto truth = truth_at({0, kS, 2 * kS, 3 * kS});
  auto row = [](std::int64_t t, TrackId id) {
    return std::to_string(t) + "," + std::to_string(id) + ",confirmed," + std::to_string(t) +
           ",1,1,0,0" + kNoCov + "\n";
  };
  // A truth instant with no row is an empty report.
  const Score s =
      score_track_report(truth, parse_track_report(kHead + row(kS, 1) + row(3 * kS, 2)));
  EXPECT_FALSE(s.engine_metrics);
  EXPECT_FALSE(s.covariance);
  EXPECT_EQ(s.reported_tracks, 2U);
  EXPECT_EQ(s.instants.size(), 4U);
  for (const std::string& text : {
           kHead + row(kS / 2, 1),               // not a truth instant
           kHead + row(4 * kS, 1),               // after the last one
           kHead + row(kS, 1) + row(3 * kS, 1),  // id 1 missing at 2 s, then back
       }) {
    EXPECT_THROW(score_track_report(truth, parse_track_report(text)), TrackReportError) << text;
  }
  // Truth before the first row is fine: id 1 present at 1 and 2 s.
  EXPECT_NO_THROW(
      score_track_report(truth, parse_track_report(kHead + row(kS, 1) + row(2 * kS, 1))));
}

void expect_same_shared_metrics(const Score& a, const Score& b, const std::string& what) {
  EXPECT_EQ(a.rmse, b.rmse) << what;
  EXPECT_EQ(a.rmse_n, b.rmse_n) << what;
  EXPECT_EQ(a.rmse_eligible, b.rmse_eligible) << what;
  EXPECT_EQ(a.velocity_rmse, b.velocity_rmse) << what;
  EXPECT_EQ(a.mean_rmse, b.mean_rmse) << what;
  EXPECT_EQ(a.max_rmse, b.max_rmse) << what;
  EXPECT_EQ(a.eligible, b.eligible) << what;
  EXPECT_EQ(a.missed, b.missed) << what;
  EXPECT_EQ(a.id_switches, b.id_switches) << what;
  EXPECT_EQ(a.false_tracks, b.false_tracks) << what;
  EXPECT_EQ(a.reported_tracks, b.reported_tracks) << what;
  EXPECT_EQ(a.gospa.instants, b.gospa.instants) << what;
  EXPECT_EQ(a.gospa.sum_p, b.gospa.sum_p) << what;
  EXPECT_EQ(a.gospa.localisation, b.gospa.localisation) << what;
  EXPECT_EQ(a.gospa.missed_count, b.gospa.missed_count) << what;
  EXPECT_EQ(a.gospa.false_count, b.gospa.false_count) << what;
  for (auto [x, y] : {std::pair{&a.nees.updated, &b.nees.updated},
                      std::pair{&a.nees.extrapolated, &b.nees.extrapolated}}) {
    EXPECT_EQ(x->full.sum, y->full.sum) << what;
    EXPECT_EQ(x->full.n, y->full.n) << what;
    EXPECT_EQ(x->position.sum, y->position.sum) << what;
  }
  ASSERT_EQ(a.instants.size(), b.instants.size()) << what;
  for (std::size_t i = 0; i < a.instants.size(); ++i) {
    EXPECT_EQ(a.instants[i].match, b.instants[i].match) << what << " instant " << i;
    EXPECT_EQ(a.instants[i].updated, b.instants[i].updated) << what << " instant " << i;
  }
}

TEST(Eval, FileScoringMatchesInProcessScoring) {
  for (const char* name : {"close_pass", "formation_split", "enter_leave"}) {
    for (const auto model :
         {estimation::MotionModel::Imm, estimation::MotionModel::ConstantVelocity}) {
      const testing::Scenario sc = testing::load_scenario(name);
      EngineConfig cfg;
      cfg.motion_model = model;
      TrackReport exported;
      const Score live = run_engine(
          sc, sc.measurements, cfg, kMatchDistanceM, {},
          [&](std::int64_t t, const Engine& e) { append_engine_tracks(exported, t, e); });
      EXPECT_TRUE(live.engine_metrics);
      const TrackReport file = parse_track_report(format_track_report(exported));
      EXPECT_EQ(file, exported) << "the text form must be exact";
      const Score from_file = score_track_report(sc.truth, file);
      EXPECT_FALSE(from_file.engine_metrics);
      EXPECT_TRUE(from_file.covariance);
      expect_same_shared_metrics(
          live, from_file, std::string(name) + " " + std::string(estimation::to_string(model)));
    }
  }
}

TEST(Eval, GospaIsReportedWithItsParameters) {
  const testing::Scenario sc = testing::load_scenario("close_pass");
  const Score s = testing::run_engine(sc);
  EXPECT_EQ(s.gospa_params.c, kMatchDistanceM);
  EXPECT_GT(s.gospa.instants, 0U);
  // Tracked throughout with metre-level errors: GOSPA close to the RMSE scale.
  EXPECT_LT(s.gospa.mean_total(2.0), 30.0);
  const Score tight = run_engine(sc, sc.measurements, {}, kMatchDistanceM, {.c = 5.0, .p = 2.0});
  EXPECT_GT(tight.gospa.missed_count, s.gospa.missed_count);
  EXPECT_EQ(tight.rmse, s.rmse) << "GOSPA parameters must not touch CLEAR-MOT metrics";
}

}  // namespace
}  // namespace sifted::eval
