// Degraded-network model (ADR 0008): determinism, statistics of each
// impairment, per-link independence, and the cross-language golden fixture
// shared with sim/sifted_sim/impair.py.

#include "sifted/net/impairment.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "scenario_support.hpp"

namespace sifted::net {
namespace {

std::vector<std::uint8_t> frame_from(SensorId sensor) {
  std::vector<std::uint8_t> f(72, 0);
  f[8] = static_cast<std::uint8_t>(sensor & 0xFF);
  f[9] = static_cast<std::uint8_t>(sensor >> 8);
  return f;
}

// n frames from one sensor, 10 ms apart.
std::vector<SentFrame> stream(std::size_t n, SensorId sensor = 1,
                              std::int64_t period = 10'000'000) {
  std::vector<SentFrame> out;
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back({Timestamp{static_cast<std::int64_t>(i) * period}, frame_from(sensor)});
  }
  return out;
}

TEST(SplitMix64, MatchesReferenceOutput) {
  // Reference values of SplitMix64 for seed 0 (Vigna's splitmix64.c).
  SplitMix64 r(0);
  EXPECT_EQ(r.next(), 0xE220A8397B1DCDAFULL);
  EXPECT_EQ(r.next(), 0x6E789E6AA1B965F4ULL);
  EXPECT_EQ(r.next(), 0x06C45D188009454FULL);
}

TEST(SplitMix64, BelowIsInRangeAndMatchesTheTopBits) {
  SplitMix64 a(42);
  for (int i = 0; i < 1000; ++i) {
    EXPECT_LT(a.below(1000), 1000U);
  }
  // below(2^k) is exactly the top k bits of next(); below(1) is always 0;
  // below(2^64 - 1) exercises the high cross terms of the 128-bit product.
  SplitMix64 c(7);
  SplitMix64 d(7);
  EXPECT_EQ(c.below(1ULL << 16), d.next() >> 48);
  EXPECT_EQ(SplitMix64(7).below(1), 0U);
  SplitMix64 e(9);
  SplitMix64 g(9);
  const std::uint64_t x = g.next();
  EXPECT_EQ(e.below(UINT64_MAX), x - (x != 0 ? 1U : 0U));
}

TEST(Impairment, NoImpairmentIsIdentity) {
  const auto frames = stream(100);
  ImpairmentStats st;
  const auto out = impair(frames, {}, &st);
  ASSERT_EQ(out.size(), frames.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_EQ(out[i].arrival, frames[i].send);
    EXPECT_EQ(out[i].source, i);
    EXPECT_FALSE(out[i].duplicate);
  }
  EXPECT_EQ(st.sent, 100U);
  EXPECT_EQ(st.delivered, 100U);
  EXPECT_EQ(st.reordered, 0U);
}

TEST(Impairment, SameSeedSameArrivalsOtherSeedDifferent) {
  ImpairmentConfig c;
  c.seed = 99;
  c.global.loss = 0.1;
  c.global.jitter_ns = 50'000'000;
  c.global.duplicate = 0.05;
  const auto frames = stream(2000);
  const auto a = impair(frames, c);
  const auto b = impair(frames, c);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].arrival, b[i].arrival);
    EXPECT_EQ(a[i].source, b[i].source);
  }
  c.seed = 100;
  const auto d = impair(frames, c);
  bool differs = d.size() != a.size();
  for (std::size_t i = 0; !differs && i < a.size(); ++i) {
    differs = a[i].arrival != d[i].arrival;
  }
  EXPECT_TRUE(differs);
}

TEST(Impairment, IndependentLossRateMatchesProbability) {
  ImpairmentConfig c;
  c.global.loss = 0.1;
  ImpairmentStats st;
  (void)impair(stream(100'000), c, &st);
  // Binomial(1e5, 0.1): sigma ~ 95 frames; 5 sigma bound.
  EXPECT_NEAR(static_cast<double>(st.lost_good), 10'000.0, 475.0);
  EXPECT_EQ(st.lost_bad, 0U);
  EXPECT_EQ(st.delivered + st.lost_good, st.sent);
}

TEST(Impairment, GilbertElliottLossComesInBursts) {
  // Good: no loss. Bad: always lost. Mean burst length 1 / p_bad_to_good = 5;
  // stationary share of Bad: p_gb / (p_gb + p_bg) = 0.02 / 0.22 ~ 0.0909.
  ImpairmentConfig c;
  c.global.p_good_to_bad = 0.02;
  c.global.p_bad_to_good = 0.2;
  c.global.loss = 0.0;
  c.global.loss_bad = 1.0;
  const auto frames = stream(200'000);
  ImpairmentStats st;
  const auto out = impair(frames, c, &st);
  EXPECT_EQ(st.lost_good, 0U);
  EXPECT_NEAR(static_cast<double>(st.lost_bad) / static_cast<double>(st.sent), 0.0909, 0.006);
  // Burst lengths from the gaps in the delivered sources.
  std::size_t bursts = 0;
  std::size_t lost = 0;
  for (std::size_t i = 1; i < out.size(); ++i) {
    const std::size_t gap = out[i].source - out[i - 1].source - 1;
    if (gap > 0) {
      ++bursts;
      lost += gap;
    }
  }
  ASSERT_GT(bursts, 0U);
  EXPECT_NEAR(static_cast<double>(lost) / static_cast<double>(bursts), 5.0, 0.3);
}

TEST(Impairment, DelayAndJitterAreBoundedAndCauseReordering) {
  ImpairmentConfig c;
  c.global.delay_ns = 20'000'000;
  c.global.jitter_ns = 100'000'000;  // 10x the 10 ms frame spacing
  const auto frames = stream(5000);
  ImpairmentStats st;
  const auto out = impair(frames, c, &st);
  ASSERT_EQ(out.size(), frames.size());
  std::int64_t min_d = INT64_MAX;
  std::int64_t max_d = 0;
  for (std::size_t i = 0; i < out.size(); ++i) {
    const std::int64_t d = out[i].arrival.ns - frames[out[i].source].send.ns;
    min_d = std::min(min_d, d);
    max_d = std::max(max_d, d);
    if (i > 0) {
      EXPECT_LE(out[i - 1].arrival, out[i].arrival);  // sorted by arrival
    }
  }
  EXPECT_GE(min_d, c.global.delay_ns);
  EXPECT_LE(max_d, c.global.delay_ns + c.global.jitter_ns);
  EXPECT_GT(max_d - min_d, 90'000'000);  // the whole jitter range is used
  EXPECT_GT(st.reordered, 1000U);
}

TEST(Impairment, DuplicatesGetTheirOwnDelay) {
  ImpairmentConfig c;
  c.global.duplicate = 0.2;
  c.global.jitter_ns = 1'000'000'000;
  ImpairmentStats st;
  const auto out = impair(stream(20'000), c, &st);
  EXPECT_NEAR(static_cast<double>(st.duplicated), 4000.0, 300.0);
  EXPECT_EQ(st.delivered, 20'000U + st.duplicated);
  std::size_t differ = 0;
  std::vector<std::int64_t> first(20'000, -1);
  for (const auto& a : out) {
    if (!a.duplicate) {
      first[a.source] = a.arrival.ns;
    }
  }
  for (const auto& a : out) {
    differ += a.duplicate && a.arrival.ns != first[a.source] ? 1U : 0U;
  }
  EXPECT_GT(differ, st.duplicated * 9 / 10);
}

TEST(Impairment, OutagesDropEverythingSentInsideThem) {
  ImpairmentConfig c;
  c.outages = {{1'000'000'000, 2'000'000'000}};
  LinkProfile p2;
  p2.outages = {{3'000'000'000, 3'500'000'000}};
  c.sensors = {{2, p2}};
  std::vector<SentFrame> frames = stream(500, 1);
  for (auto& f : stream(500, 2)) {
    frames.push_back(f);
  }
  ImpairmentStats st;
  const auto out = impair(frames, c, &st);
  for (const auto& a : out) {
    const auto& f = frames[a.source];
    EXPECT_FALSE(f.send.ns >= 1'000'000'000 && f.send.ns < 2'000'000'000);
    if (frame_sensor(f.bytes) == 2) {
      EXPECT_FALSE(f.send.ns >= 3'000'000'000 && f.send.ns < 3'500'000'000);
    }
  }
  EXPECT_EQ(st.lost_outage, 100U + 100U + 50U);
}

TEST(Impairment, LinksAreIndependentOfOtherSensorsTraffic) {
  ImpairmentConfig c;
  c.seed = 5;
  c.global.loss = 0.2;
  c.global.jitter_ns = 30'000'000;
  const auto alone = impair(stream(1000, 7), c);
  std::vector<SentFrame> mixed;
  const auto a = stream(1000, 7);
  const auto b = stream(1000, 8, 3'000'000);
  for (std::size_t i = 0; i < 1000; ++i) {
    mixed.push_back(a[i]);
    mixed.push_back(b[i]);
  }
  std::vector<std::int64_t> mixed7;
  for (const auto& x : impair(mixed, c)) {
    if (frame_sensor(x.bytes) == 7) {
      mixed7.push_back(x.arrival.ns);
    }
  }
  ASSERT_EQ(mixed7.size(), alone.size());
  for (std::size_t i = 0; i < alone.size(); ++i) {
    EXPECT_EQ(mixed7[i], alone[i].arrival.ns);
  }
}

TEST(Impairment, RejectsInvalidConfiguration) {
  ImpairmentConfig c;
  c.global.loss = 1.5;
  EXPECT_FALSE(is_valid(c));
  EXPECT_THROW((void)impair(stream(1), c), std::invalid_argument);
  c.global.loss = 0.0;
  c.global.jitter_ns = -1;
  EXPECT_FALSE(is_valid(c));
  c.global.jitter_ns = 0;
  c.outages = {{5, 5}};
  EXPECT_FALSE(is_valid(c));
  c.outages.clear();
  c.sensors = {{1, {}}, {1, {}}};
  EXPECT_FALSE(is_valid(c));
  c.sensors.pop_back();
  EXPECT_TRUE(is_valid(c));
}

// The configuration of sim/sifted_sim/impair.py GOLDEN_CONFIG.
ImpairmentConfig golden_config() {
  ImpairmentConfig c;
  c.seed = 20261115;
  c.global.loss = 0.05;
  c.global.p_good_to_bad = 0.02;
  c.global.p_bad_to_good = 0.25;
  c.global.loss_bad = 0.7;
  c.global.delay_ns = 30'000'000;
  c.global.jitter_ns = 400'000'000;
  c.global.duplicate = 0.03;
  LinkProfile cam;
  cam.loss = 0.1;
  cam.jitter_ns = 50'000'000;
  cam.outages = {{60'000'000'000, 62'500'000'000}};
  c.sensors = {{20, cam}};
  c.outages = {{30'000'000'000, 35'000'000'000}};
  return c;
}

TEST(Impairment, MatchesPythonGoldenFixture) {
  const auto sc = testing::load_scenario("crossing_pair");
  const auto frames = testing::sent_frames(sc.measurements);
  ImpairmentStats st;
  const auto out = impair(frames, golden_config(), &st);

  std::ifstream f(std::string(SIFTED_FIXTURE_DIR) + "/impairment/golden.txt");
  ASSERT_TRUE(f.good());
  std::string line;
  std::getline(f, line);
  std::ostringstream stats;
  stats << "# stats sent=" << st.sent << " lost_outage=" << st.lost_outage
        << " lost_good=" << st.lost_good << " lost_bad=" << st.lost_bad
        << " duplicated=" << st.duplicated << " delivered=" << st.delivered
        << " reordered=" << st.reordered;
  EXPECT_EQ(line, stats.str());
  std::size_t i = 0;
  while (std::getline(f, line)) {
    std::istringstream ls(line);
    std::int64_t arrival = 0;
    std::size_t source = 0;
    int dup = 0;
    ls >> arrival >> source >> dup;
    ASSERT_LT(i, out.size());
    EXPECT_EQ(out[i].arrival.ns, arrival) << "arrival " << i;
    EXPECT_EQ(out[i].source, source) << "arrival " << i;
    EXPECT_EQ(out[i].duplicate, dup != 0) << "arrival " << i;
    ++i;
  }
  EXPECT_EQ(i, out.size());
}

}  // namespace
}  // namespace sifted::net
