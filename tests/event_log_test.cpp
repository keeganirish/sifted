// Event log format (ADR 0005) and record -> replay bit-for-bit equality.

#include "sifted/replay/event_log.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "sifted/net/ingest.hpp"
#include "sifted/replay/crc32.hpp"
#include "sifted/replay/replayer.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::replay {
namespace {

RunHeader sample_header() {
  RunHeader h;
  EXPECT_FALSE(h.sensors.add({1, SensorKind::RangeBearing, {10.5, -3.25, 0.1}}));
  EXPECT_FALSE(h.sensors.add({7, SensorKind::BearingOnly, {-3000.0, 3000.0, -0.7853981633974483}}));
  h.config.engine.origin = estimation::GeoOrigin{51.4779, -0.0015, 12.5};
  h.config.engine.lifecycle.drop_after_s = 12.25;
  h.config.snapshot_interval_s = 0.25;
  return h;
}

std::vector<std::uint8_t> frame(std::uint32_t seq, std::int64_t t) {
  Measurement m;
  m.sensor_id = 1;
  m.kind = SensorKind::RangeBearing;
  m.seq = seq;
  m.time = Timestamp{t};
  m.z = {4000.0 + seq, 0.25, 0.0};
  m.sigma = {10.0, 0.0035, 0.0};
  return wire::encode(m);
}

std::string bytes(const std::vector<std::uint8_t>& v) { return {v.begin(), v.end()}; }

std::string read_fixture_file(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace

TEST(Crc32, KnownVector) {
  const std::string s = "123456789";
  EXPECT_EQ(crc32(std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(s.data()),
                                                s.size())),
            0xCBF43926U);
}

TEST(EventLog, HeaderRoundTripsExactly) {
  const RunHeader h = sample_header();
  const std::string text = format_header(h);
  const auto back = parse_header(text);
  ASSERT_TRUE(back.ok());
  EXPECT_TRUE(back.value() == h);
  EXPECT_EQ(format_header(back.value()), text);
  EXPECT_NE(text.find("origin 51.4779 -0.0015 12.5\n"), std::string::npos);
  EXPECT_NE(text.find("sensors\nsifted-sensors 1\n"), std::string::npos);
}

TEST(EventLog, HeaderIsStrict) {
  const std::string good = format_header(sample_header());
  auto bad = [](std::string s) { return !parse_header(s).ok(); };
  EXPECT_TRUE(bad("sifted-log 2\n" + good.substr(good.find('\n') + 1)));
  std::string missing = good;
  missing.erase(
      missing.find("param accel_psd"),
      missing.find('\n', missing.find("param accel_psd")) - missing.find("param accel_psd") + 1);
  EXPECT_TRUE(bad(missing));
  std::string unknown = good;
  unknown.insert(unknown.find("sensors\n"), "param mystery 1\n");
  EXPECT_TRUE(bad(unknown));
  std::string repeated = good;
  repeated.insert(repeated.find("sensors\n"), "param accel_psd 4\n");
  EXPECT_TRUE(bad(repeated));
  std::string invalid = good;
  invalid.replace(invalid.find("param gate_probability 0.999"), 28, "param gate_probability 1.5");
  EXPECT_TRUE(bad(invalid));
  EXPECT_TRUE(bad(good + "# trailing comment makes the table non-canonical\n"));
  EXPECT_TRUE(bad(good.substr(0, good.find("sensors\n"))));
}

TEST(EventLog, RecordsRoundTrip) {
  std::stringstream ss;
  EventLogWriter w(ss, sample_header());
  const auto f0 = frame(0, 1000);
  ASSERT_TRUE(w.write_frame(Timestamp{5}, f0));
  ASSERT_TRUE(w.write_tick(Timestamp{5}));
  ASSERT_TRUE(w.write_frame(Timestamp{9}, frame(1, 2000)));
  EXPECT_FALSE(w.write_frame(Timestamp{8}, frame(2, 3000)));  // time backwards: refused
  EXPECT_FALSE(w.write_frame(Timestamp{9}, std::vector<std::uint8_t>(2000, 0)));  // too large
  EXPECT_EQ(w.records(), 3U);

  auto r = EventLogReader::open(ss);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.value().header() == sample_header());
  auto a = r.value().next();
  ASSERT_TRUE(a.ok() && a.value().has_value());
  EXPECT_EQ(a.value()->type, RecordType::Frame);
  EXPECT_EQ(a.value()->recv, Timestamp{5});
  EXPECT_EQ(a.value()->payload, f0);
  auto b = r.value().next();
  ASSERT_TRUE(b.ok() && b.value().has_value());
  EXPECT_EQ(b.value()->type, RecordType::Tick);
  ASSERT_TRUE(r.value().next().ok());
  const auto end = r.value().next();
  ASSERT_TRUE(end.ok());
  EXPECT_FALSE(end.value().has_value());
}

TEST(EventLog, ReaderRejectsCorruption) {
  std::stringstream ss;
  EventLogWriter w(ss, sample_header());
  ASSERT_TRUE(w.write_frame(Timestamp{5}, frame(0, 1000)));
  const std::string good = ss.str();
  auto open_err = [](std::string s) {
    std::istringstream in(s);
    const auto r = EventLogReader::open(in);
    return r.ok() ? std::optional<LogError>{} : std::optional<LogError>{r.error()};
  };
  auto record_err = [](std::string s) -> std::optional<LogError> {
    std::istringstream in(s);
    auto r = EventLogReader::open(in);
    if (!r.ok()) {
      return r.error();
    }
    while (true) {
      auto n = r.value().next();
      if (!n.ok()) {
        return n.error();
      }
      if (!n.value()) {
        return std::nullopt;
      }
    }
  };
  EXPECT_FALSE(open_err(good).has_value());
  EXPECT_FALSE(record_err(good).has_value());
  std::string s = good;
  s[0] = 'X';
  EXPECT_EQ(open_err(s), LogError::BadMagic);
  s = good;
  s[8] = 2;
  EXPECT_EQ(open_err(s), LogError::UnsupportedVersion);
  s = good;
  s[15] = 0x7F;  // header length ~2 GiB: refused before any allocation
  EXPECT_EQ(open_err(s), LogError::HeaderTooLarge);
  s = good;
  s[20] ^= 1;  // inside the header text
  EXPECT_EQ(open_err(s), LogError::BadChecksum);
  EXPECT_EQ(open_err(good.substr(0, 10)), LogError::Truncated);
  s = good;
  s[s.size() - 10] ^= 1;  // inside the record payload
  EXPECT_EQ(record_err(s), LogError::BadChecksum);
  EXPECT_EQ(record_err(good.substr(0, good.size() - 3)), LogError::Truncated);
  // A record type that does not exist, and a time going backwards.
  const auto header = encode_header(sample_header());
  const std::string base(header.begin(), header.end());
  EXPECT_EQ(record_err(base + bytes(encode_record(static_cast<RecordType>(9), Timestamp{1}, {}))),
            LogError::BadRecord);
  EXPECT_EQ(record_err(base + bytes(encode_record(RecordType::Tick, Timestamp{1}, frame(0, 0)))),
            LogError::BadRecord);
  EXPECT_EQ(record_err(base + bytes(encode_record(RecordType::Tick, Timestamp{9}, {})) +
                       bytes(encode_record(RecordType::Tick, Timestamp{8}, {}))),
            LogError::TimeWentBackwards);
}

// The headline determinism property: record a run through the live-side path
// (pipeline + writer, with ticks, a duplicate and a corrupt frame mixed in),
// replay the log through a fresh pipeline, and compare the output bytes.
TEST(Replay, RecordedRunReplaysBitForBit) {
  const std::string dir = std::string(SIFTED_FIXTURE_DIR) + "/scenarios/crossing_pair/";
  const auto table = parse_sensor_table(read_fixture_file(dir + "sensors.cfg"));
  ASSERT_TRUE(table.ok());
  RunHeader header{table.value(), {}};
  header.config.engine.origin = estimation::GeoOrigin{51.4779, -0.0015, 0.0};

  std::vector<std::vector<std::uint8_t>> frames;
  std::istringstream hex(read_fixture_file(dir + "frames.hex"));
  for (std::string line; hex >> line;) {
    std::vector<std::uint8_t> b;
    for (std::size_t i = 0; i + 1 < line.size(); i += 2) {
      b.push_back(static_cast<std::uint8_t>(std::stoi(line.substr(i, 2), nullptr, 16)));
    }
    frames.push_back(b);
  }
  ASSERT_GT(frames.size(), 100U);

  std::string live_out;
  std::stringstream log;
  {
    EventLogWriter writer(log, header);
    net::Pipeline live(header.sensors, header.config,
                       [&](std::string_view l) { live_out.append(l); });
    std::int64_t recv = 1'700'000'000'000'000'000;
    for (std::size_t i = 0; i < frames.size(); ++i) {
      recv += 1'000'000 + static_cast<std::int64_t>(i % 7) * 13'000;  // jittery arrivals
      ASSERT_TRUE(writer.write_frame(Timestamp{recv}, frames[i]));
      live.on_frame(Timestamp{recv}, frames[i]);
      if (i % 17 == 3) {  // a repeated datagram
        ASSERT_TRUE(writer.write_frame(Timestamp{recv}, frames[i]));
        live.on_frame(Timestamp{recv}, frames[i]);
      }
      if (i % 29 == 5) {  // a corrupt one (semi-trusted log: decoded again on replay)
        std::vector<std::uint8_t> bad = frames[i];
        bad[4] ^= 0xFF;  // wire version byte
        ASSERT_TRUE(writer.write_frame(Timestamp{recv}, bad));
        live.on_frame(Timestamp{recv}, bad);
      }
      if (i % 11 == 0 && live.has_pending()) {  // input went quiet: tick
        ASSERT_TRUE(writer.write_tick(Timestamp{recv}));
        live.on_tick(Timestamp{recv});
      }
    }
    live.finish(Timestamp{recv});
    EXPECT_GT(live.duplicates(), 0U);
    EXPECT_GT(live.snapshots(), 10U);
  }

  std::string replay_out;
  auto reader = EventLogReader::open(log);
  ASSERT_TRUE(reader.ok());
  net::Pipeline again(reader.value().header().sensors, reader.value().header().config,
                      [&](std::string_view l) { replay_out.append(l); });
  const auto stats = replay(reader.value(), again);
  ASSERT_TRUE(stats.ok());
  EXPECT_GT(stats.value().ticks, 0U);

  ASSERT_FALSE(live_out.empty());
  EXPECT_EQ(live_out.size(), replay_out.size());
  EXPECT_TRUE(live_out == replay_out) << "replay output differs from the live run";
  EXPECT_NE(live_out.find("\nd "), std::string::npos);  // duplicates were exercised
  EXPECT_NE(live_out.find("\nx "), std::string::npos);  // decode errors were exercised
  EXPECT_NE(live_out.find("\ns {\"type\":\"sifted.tracks\""), std::string::npos);
}

}  // namespace sifted::replay
