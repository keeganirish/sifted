// sifted_fusiond: the fusion service.
//
//   live:    UDP ingest -> bounded queue -> engine thread (Pipeline) -> WebSocket
//            publisher, optionally recording every consumed input to an event log
//   replay:  event log -> the same Pipeline, driven by the recorded receive times
//
// Threading (docs/design.md section 2): one UDP receive thread (receive,
// validate, stamp, enqueue), the main thread as the single engine thread, and
// the WebSocket server's own thread, which only ever reads immutable
// snapshots. Only this file and the UDP thread read the machine clock, and
// every value read is written to the event log.

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include "sifted/core/sensor.hpp"
#include "sifted/net/bounded_queue.hpp"
#include "sifted/net/durable_file.hpp"
#include "sifted/net/ingest.hpp"
#include "sifted/net/system_clock.hpp"
#include "sifted/net/udp.hpp"
#include "sifted/net/websocket.hpp"
#include "sifted/replay/event_log.hpp"
#include "sifted/replay/replayer.hpp"
#include "sifted/version.hpp"

namespace {

using namespace sifted;

// An arbitrary, recognisable default for simulated local-frame scenarios
// (Royal Observatory, Greenwich). Override with --origin.
constexpr estimation::GeoOrigin kDefaultOrigin{51.4779, -0.0015, 0.0};

// How often the engine thread proves it is alive to WebSocket clients while it
// has nothing new to publish. The operator view raises its alarm after several
// missed beats (WATCHDOG_MS in web/assets/app.js).
constexpr std::chrono::seconds kHeartbeat{1};

volatile std::sig_atomic_t g_signal = 0;
extern "C" void on_signal(int sig) { g_signal = sig; }

constexpr const char* kUsage =
    "usage: sifted_fusiond --sensors FILE [options]      live: UDP in, tracks out\n"
    "       sifted_fusiond --replay LOG [replay options] replay a recorded run\n"
    "       sifted_fusiond --version | --help\n"
    "\n"
    "live options:\n"
    "  --sensors FILE          sensor table (sifted-sensors 1, ADR 0003)   [required]\n"
    "  --listen ADDR:PORT      UDP listen address               [0.0.0.0:5600]\n"
    "  --record LOG            write an event log of everything consumed\n"
    "  --origin LAT,LON[,ALT]  tangent-plane origin for geodetic input and map display\n"
    "                          (fixed for the run, recorded in the log) [51.4779,-0.0015,0]\n"
    "  --queue N               engine queue capacity (frames)    [4096]\n"
    "  --rate-limit R          datagrams/s per source address (0: off) [20000]\n"
    "  --rate-burst B          per-source burst allowance (datagrams)  [4096]\n"
    "  --rate-sources N        source addresses tracked (LRU beyond)   [1024]\n"
    "  --reorder-window S      hold frames S seconds and release them in measurement-time\n"
    "                          order (ADR 0009; 0: off, M2 behaviour)  [0.5]\n"
    "  --max-time-ahead S      reject frames stamped more than S seconds after their receive\n"
    "                          time (ADR 0010; needs wall-clock senders; 0: off) [0]\n"
    "  --max-time-jump S       quarantine frames more than S seconds off the current timeline,\n"
    "                          beyond the receive time elapsed (ADR 0012; needs no common\n"
    "                          timebase; 0: off)                 [the --coast-after value]\n"
    "  --timeline-confirm-frames N  distinct frames needed to adopt a new timeline  [3]\n"
    "  --timeline-confirm S    silence of the current timeline needed before a new one is\n"
    "                          adopted                           [the --coast-after value]\n"
    "  --stale-factor X        a sensor is stale after X times its median scan gap (ADR 0012;\n"
    "                          0: no feed status in snapshots)   [3]\n"
    "  --stale-after-min S     ... and never sooner than S seconds [the --coast-after value]\n"
    "  --idle-tick S           while silent, update the picture every S seconds; a silent run\n"
    "                          stops ticking once nothing can change (0: off) [1]\n"
    "  --record-flush S        hand the recording to the OS at least every S seconds [0.1]\n"
    "  --record-flush-records N  ... and at least every N records [64]\n"
    "  --record-sync S         force the recording to disk every S seconds (0: leave it to\n"
    "                          the OS)                           [1]\n"
    "  tracker parameters (defaults: docs/design.md section 8):\n"
    "  --motion-model imm|cv    estimator (ADR 0014): IMM of three nearly-constant-velocity\n"
    "                          modes, or the legacy single filter with NIS-switched\n"
    "                          process noise                     [imm]\n"
    "  --imm-accel-psd Q0,Q1,Q2  IMM quiet, manoeuvre, hard acceleration PSD [m^2/s^3],\n"
    "                          non-decreasing                    [0.5,25,1200]\n"
    "  --imm-sojourn T0,T1,T2  IMM mean time in each mode [s]     [60,30,15]\n"
    "  --imm-probability-floor P  lowest mode probability          [0.0001]\n"
    "  --imm-manoeuvre-on P  --imm-manoeuvre-off P  manoeuvre indication hysteresis on\n"
    "                          1 - P(quiet)                      [0.8, 0.4]\n"
    "  --accel-psd Q (legacy cv model only)  --initial-velocity-sigma V  --gate-probability P\n"
    "  --confirm-hits N  --tentative-timeout S  --coast-after S  --drop-after S\n"
    "  --max-position-sigma M  --max-tracks N  --snapshot-interval S\n"
    "  --snapshot-trace-events N (newest decisions per track per snapshot, 0-32; 0: none) [4]\n"
    "  --assignment gnn|greedy (greedy only for comparisons)\n"
    "\n"
    "common options:\n"
    "  --ws-port PORT          serve track snapshots over WebSocket (0: off) [8765]\n"
    "  --ws-bind ADDR          WebSocket bind address             [127.0.0.1]\n"
    "  --output FILE           write the deterministic output stream (outcomes and\n"
    "                          snapshots, one per line); identical for a run and its replay\n"
    "  --stats-interval S      print counters to stderr every S seconds (0: off) [5]\n"
    "\n"
    "replay options:\n"
    "  --pace X                replay at X times real time (0: as fast as possible) [0]\n"
    "  --hold                  keep serving the last snapshot after the replay ends\n"
    "  --recover-truncated     replay a crashed run's recording up to its torn final record\n"
    "                          and report what was lost (other damage is still an error)\n";

struct Options {
  std::string sensors_file;
  std::string replay_file;
  std::string record_file;
  std::string output_file;
  std::string listen_addr = "0.0.0.0";
  std::uint16_t listen_port = 5600;
  std::uint16_t ws_port = 8765;
  std::string ws_bind = "127.0.0.1";
  std::size_t queue = 4096;
  net::RateLimitConfig rate_limit{20000.0, 4096.0, 1024};
  double stats_interval_s = 5.0;
  double pace = 0.0;
  bool hold = false;
  bool recover_truncated = false;
  bool tuning_given = false;
  double idle_tick_s = 1.0;
  double record_flush_s = 0.1;
  std::size_t record_flush_records = 64;
  double record_sync_s = 1.0;
  net::PipelineConfig config;
};

template <class T>
bool parse_num(std::string_view s, T& out) {
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
  return ec == std::errc{} && ptr == s.data() + s.size() && !s.empty();
}

bool parse_origin(std::string_view s, estimation::GeoOrigin& o) {
  std::vector<std::string_view> parts;
  while (true) {
    const auto c = s.find(',');
    parts.push_back(s.substr(0, c));
    if (c == std::string_view::npos) {
      break;
    }
    s.remove_prefix(c + 1);
  }
  if (parts.size() < 2 || parts.size() > 3) {
    return false;
  }
  o = {};
  return parse_num(parts[0], o.lat_deg) && parse_num(parts[1], o.lon_deg) &&
         (parts.size() == 2 || parse_num(parts[2], o.alt_m)) && estimation::is_valid(o);
}

// "a,b,c": exactly one number per IMM mode.
bool parse_modes(std::string_view s, std::array<double, estimation::kImmModes>& out) {
  for (std::size_t i = 0; i < out.size(); ++i) {
    const auto c = s.find(',');
    if ((c == std::string_view::npos) != (i + 1 == out.size()) ||
        !parse_num(s.substr(0, c), out[i])) {
      return false;
    }
    s.remove_prefix(c == std::string_view::npos ? s.size() : c + 1);
  }
  return true;
}

bool parse_host_port(std::string_view s, std::string& host, std::uint16_t& port) {
  const auto c = s.rfind(':');
  if (c == std::string_view::npos) {
    return false;
  }
  host = std::string(s.substr(0, c));
  return parse_num(s.substr(c + 1), port);
}

// Returns an error message, or empty on success.
std::string parse_args(int argc, char** argv, Options& o) {
  o.config.engine.origin = kDefaultOrigin;
  // The daemon faces a real network: hold frames briefly so delay jitter does
  // not become out-of-order drops (ADR 0009). The library default stays 0.
  o.config.reorder_window_s = 0.5;
  // The daemon faces an untrusted network, and an operator: the timeline guard
  // and the feed status are on (ADR 0012). The library defaults stay off, which
  // is also how recordings made before them replay.
  o.config.feed_stale_factor = 3.0;
  std::optional<double> max_jump;
  std::optional<double> confirm;
  std::optional<double> stale_min;
  auto seconds = [](std::optional<double>& out, std::optional<std::string_view> v) {
    double d = 0.0;
    if (!v || !parse_num(*v, d) || !std::isfinite(d) || d < 0.0) {
      return false;
    }
    out = d;
    return true;
  };
  for (int i = 1; i < argc; ++i) {
    const std::string_view a = argv[i];
    auto value = [&]() -> std::optional<std::string_view> {
      if (i + 1 >= argc) {
        return std::nullopt;
      }
      return std::string_view(argv[++i]);
    };
    auto num = [&](auto& out) {
      const auto v = value();
      return v && parse_num(*v, out);
    };
    auto tuning = [&](auto& out) {
      o.tuning_given = true;
      return num(out);
    };
    bool ok = true;
    if (a == "--sensors") {
      const auto v = value();
      ok = v.has_value();
      o.sensors_file = v.value_or("");
    } else if (a == "--replay") {
      const auto v = value();
      ok = v.has_value();
      o.replay_file = v.value_or("");
    } else if (a == "--record") {
      const auto v = value();
      ok = v.has_value();
      o.record_file = v.value_or("");
    } else if (a == "--output") {
      const auto v = value();
      ok = v.has_value();
      o.output_file = v.value_or("");
    } else if (a == "--listen") {
      const auto v = value();
      ok = v && parse_host_port(*v, o.listen_addr, o.listen_port);
    } else if (a == "--ws-port") {
      ok = num(o.ws_port);
    } else if (a == "--ws-bind") {
      const auto v = value();
      ok = v.has_value();
      o.ws_bind = v.value_or("");
    } else if (a == "--origin") {
      const auto v = value();
      estimation::GeoOrigin g;
      ok = v && parse_origin(*v, g);
      o.config.engine.origin = g;
      o.tuning_given = true;
    } else if (a == "--queue") {
      ok = num(o.queue) && o.queue > 0;
    } else if (a == "--rate-limit") {
      ok = num(o.rate_limit.rate_per_s) && o.rate_limit.rate_per_s >= 0.0;
    } else if (a == "--rate-burst") {
      ok = num(o.rate_limit.burst);
    } else if (a == "--rate-sources") {
      ok = num(o.rate_limit.max_sources);
    } else if (a == "--reorder-window") {
      ok = tuning(o.config.reorder_window_s);
    } else if (a == "--max-time-ahead") {
      ok = tuning(o.config.max_time_ahead_s);
    } else if (a == "--max-time-jump") {
      o.tuning_given = true;
      ok = seconds(max_jump, value());
    } else if (a == "--timeline-confirm-frames") {
      ok = tuning(o.config.timeline_confirm_frames);
    } else if (a == "--timeline-confirm") {
      o.tuning_given = true;
      ok = seconds(confirm, value());
    } else if (a == "--stale-factor") {
      ok = tuning(o.config.feed_stale_factor);
    } else if (a == "--stale-after-min") {
      o.tuning_given = true;
      ok = seconds(stale_min, value());
    } else if (a == "--idle-tick") {
      ok = tuning(o.idle_tick_s) && std::isfinite(o.idle_tick_s) && o.idle_tick_s >= 0.0;
    } else if (a == "--record-flush") {
      ok = tuning(o.record_flush_s) && std::isfinite(o.record_flush_s) && o.record_flush_s > 0.0;
    } else if (a == "--record-flush-records") {
      ok = tuning(o.record_flush_records) && o.record_flush_records > 0;
    } else if (a == "--record-sync") {
      ok = tuning(o.record_sync_s) && std::isfinite(o.record_sync_s) && o.record_sync_s >= 0.0;
    } else if (a == "--recover-truncated") {
      o.recover_truncated = true;
    } else if (a == "--assignment") {
      const auto v = value();
      o.tuning_given = true;
      ok = v && (*v == "gnn" || *v == "greedy");
      o.config.engine.association.method =
          v && *v == "greedy" ? association::AssignmentMethod::Greedy
                              : association::AssignmentMethod::GlobalNearestNeighbour;
    } else if (a == "--stats-interval") {
      ok = num(o.stats_interval_s) && o.stats_interval_s >= 0.0;
    } else if (a == "--pace") {
      ok = num(o.pace) && o.pace >= 0.0;
    } else if (a == "--hold") {
      o.hold = true;
    } else if (a == "--motion-model") {
      const auto v = value();
      o.tuning_given = true;
      const auto m = v ? estimation::motion_model_from_string(*v) : std::nullopt;
      ok = m.has_value();
      o.config.engine.motion_model = m.value_or(o.config.engine.motion_model);
    } else if (a == "--imm-accel-psd" || a == "--imm-sojourn") {
      const auto v = value();
      o.tuning_given = true;
      auto& model = o.config.engine.imm.model;
      ok = v && parse_modes(*v, a == "--imm-accel-psd" ? model.accel_psd : model.sojourn_s);
    } else if (a == "--imm-probability-floor") {
      ok = tuning(o.config.engine.imm.model.probability_floor);
    } else if (a == "--imm-manoeuvre-on") {
      ok = tuning(o.config.engine.imm.manoeuvre_on);
    } else if (a == "--imm-manoeuvre-off") {
      ok = tuning(o.config.engine.imm.manoeuvre_off);
    } else if (a == "--accel-psd") {
      ok = tuning(o.config.engine.accel_psd);
    } else if (a == "--initial-velocity-sigma") {
      ok = tuning(o.config.engine.initial_velocity_sigma);
    } else if (a == "--gate-probability") {
      ok = tuning(o.config.engine.association.gate_probability);
    } else if (a == "--confirm-hits") {
      ok = tuning(o.config.engine.lifecycle.confirm_hits);
    } else if (a == "--tentative-timeout") {
      ok = tuning(o.config.engine.lifecycle.tentative_timeout_s);
    } else if (a == "--coast-after") {
      ok = tuning(o.config.engine.lifecycle.coast_after_s);
    } else if (a == "--drop-after") {
      ok = tuning(o.config.engine.lifecycle.drop_after_s);
    } else if (a == "--max-position-sigma") {
      ok = tuning(o.config.engine.lifecycle.max_position_sigma_m);
    } else if (a == "--max-tracks") {
      ok = tuning(o.config.engine.max_tracks);
    } else if (a == "--snapshot-interval") {
      ok = tuning(o.config.snapshot_interval_s);
    } else if (a == "--snapshot-trace-events") {
      ok = tuning(o.config.snapshot_trace_events);
    } else {
      return "unknown option: " + std::string(a);
    }
    if (!ok) {
      return "missing or invalid value for " + std::string(a);
    }
  }
  // A jump or a gap shorter than coast_after_s cannot even make a track coast,
  // and the value already says how long this deployment's sensors may pause.
  const double coast = o.config.engine.lifecycle.coast_after_s;
  o.config.max_time_jump_s = max_jump.value_or(coast);
  o.config.timeline_confirm_s = confirm.value_or(coast);
  o.config.feed_stale_min_s = stale_min.value_or(coast);
  if (o.replay_file.empty() == o.sensors_file.empty()) {
    return "give exactly one of --sensors (live) or --replay";
  }
  if (o.recover_truncated && o.replay_file.empty()) {
    return "--recover-truncated applies to --replay";
  }
  if (!o.replay_file.empty() && (o.tuning_given || !o.record_file.empty())) {
    return "--replay takes its parameters from the log; tracker options and --record do not apply";
  }
  if (o.replay_file.empty() && !net::is_valid(o.config)) {
    return "invalid tracker parameters (see docs/design.md section 6 for the constraints)";
  }
  if (!net::is_valid(o.rate_limit)) {
    return "invalid rate limit (--rate-burst must be >= 1 when --rate-limit is on)";
  }
  return {};
}

std::optional<std::string> read_small_file(const std::string& path, std::size_t limit) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return std::nullopt;
  }
  std::string s(limit + 1, '\0');
  f.read(s.data(), static_cast<std::streamsize>(s.size()));
  s.resize(static_cast<std::size_t>(f.gcount()));
  return s;  // callers' parsers enforce `limit` with a proper error
}

struct Outputs {
  std::unique_ptr<std::ofstream> file;
  std::unique_ptr<net::WebSocketServer> ws;

  net::Pipeline::LineSink line_sink() {
    if (!file) {
      return {};
    }
    std::ofstream* f = file.get();
    return [f](std::string_view line) {
      f->write(line.data(), static_cast<std::streamsize>(line.size()));
    };
  }
  net::Pipeline::SnapshotSink snapshot_sink() {
    if (!ws) {
      return {};
    }
    net::WebSocketServer* w = ws.get();
    return [w](const std::string& json) { w->publish(json); };
  }
};

// Shared set-up for both modes. Returns false (after printing why) on error.
bool open_outputs(const Options& o, Outputs& out) {
  if (!o.output_file.empty()) {
    out.file = std::make_unique<std::ofstream>(o.output_file, std::ios::binary | std::ios::trunc);
    if (!*out.file) {
      std::fprintf(stderr, "sifted_fusiond: cannot write %s\n", o.output_file.c_str());
      return false;
    }
  }
  if (o.ws_port != 0) {
    net::WebSocketConfig wc;
    wc.bind_address = o.ws_bind;
    wc.port = o.ws_port;
    auto ws = net::WebSocketServer::start(wc);
    if (!ws.ok()) {
      std::fprintf(stderr, "sifted_fusiond: %s\n", ws.error().c_str());
      return false;
    }
    out.ws = std::move(ws.value());
    std::fprintf(stderr, "sifted_fusiond: track snapshots on ws://%s:%u/\n", o.ws_bind.c_str(),
                 static_cast<unsigned>(out.ws->port()));
  }
  return true;
}

void print_pipeline_stats(const net::Pipeline& p) {
  const Engine& e = p.engine();
  std::fprintf(
      stderr,
      "sifted_fusiond: frames %llu, scans %llu, duplicates %llu, measurements %llu "
      "(updated %llu, started %llu, out-of-order %llu, track-limit %llu), live tracks %zu, "
      "snapshots %llu; reorder held %zu (max %zu), late %llu, forced %llu; clock-gate %llu; "
      "timeline rejected %llu, timeline held %zu, timeline switches %llu\n",
      static_cast<unsigned long long>(p.frames()), static_cast<unsigned long long>(p.scans()),
      static_cast<unsigned long long>(p.duplicates()),
      static_cast<unsigned long long>(e.measurements_processed()),
      static_cast<unsigned long long>(e.count(MeasurementOutcome::Updated)),
      static_cast<unsigned long long>(e.count(MeasurementOutcome::Initialized)),
      static_cast<unsigned long long>(e.count(MeasurementOutcome::OutOfOrder)),
      static_cast<unsigned long long>(e.count(MeasurementOutcome::TrackLimit)), e.tracks().size(),
      static_cast<unsigned long long>(p.snapshots()), p.held(), p.held_high_water(),
      static_cast<unsigned long long>(p.reorder_late()),
      static_cast<unsigned long long>(p.reorder_forced()),
      static_cast<unsigned long long>(p.future_rejected()),
      static_cast<unsigned long long>(p.timeline_rejected()), p.timeline_held(),
      static_cast<unsigned long long>(p.timeline_switches()));
}

int run_replay(const Options& o) {
  std::ifstream in(o.replay_file, std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "sifted_fusiond: cannot read %s\n", o.replay_file.c_str());
    return 1;
  }
  auto reader = replay::EventLogReader::open(
      in, o.recover_truncated ? replay::TailPolicy::RecoverTorn : replay::TailPolicy::Strict);
  if (!reader.ok()) {
    std::fprintf(stderr, "sifted_fusiond: %s: %s\n", o.replay_file.c_str(),
                 std::string(replay::to_string(reader.error())).c_str());
    return 1;
  }
  replay::EventLogReader r = std::move(reader.value());
  Outputs out;
  if (!open_outputs(o, out)) {
    return 1;
  }
  // Counted so a crashed run's output can be compared without the final flush,
  // which that run never reached.
  std::uint64_t lines = 0;
  std::uint64_t lines_before_finish = 0;
  net::Pipeline::LineSink file_sink = out.line_sink();
  net::Pipeline pipeline(
      r.header().sensors, r.header().config,
      [&](std::string_view line) {
        ++lines;
        if (file_sink) {
          file_sink(line);
        }
      },
      out.snapshot_sink());

  std::optional<Timestamp> first;
  const auto wall0 = std::chrono::steady_clock::now();
  auto pace = [&](Timestamp t) {
    if (o.pace <= 0.0 || g_signal != 0) {
      return;
    }
    if (!first) {
      first = t;
    }
    const double offset_s = static_cast<double>(t.ns - first->ns) * 1e-9 / o.pace;
    const auto until = wall0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>(offset_s));
    // A long recorded silence must not look like a hung service.
    while (g_signal == 0 && std::chrono::steady_clock::now() < until) {
      std::this_thread::sleep_until(
          std::min(until, std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(kHeartbeat))));
      if (out.ws && std::chrono::steady_clock::now() < until) {
        out.ws->republish();
      }
    }
  };
  const auto result = replay::replay(r, pipeline, pace, [&] { lines_before_finish = lines; });
  if (out.file) {
    out.file->flush();
  }
  print_pipeline_stats(pipeline);
  if (!result.ok()) {
    std::fprintf(stderr, "sifted_fusiond: replay stopped: %s%s\n",
                 std::string(replay::to_string(result.error())).c_str(),
                 result.error() == replay::LogError::Truncated
                     ? " (a crashed run? --recover-truncated replays what it recorded)"
                     : "");
    return 1;
  }
  if (result.value().truncated) {
    std::fprintf(stderr,
                 "sifted_fusiond: WARNING: the recording is truncated: the run ended abnormally; "
                 "%llu complete records replayed, %llu bytes of a torn record discarded\n",
                 static_cast<unsigned long long>(result.value().frames + result.value().ticks),
                 static_cast<unsigned long long>(result.value().torn_bytes));
  }
  if (o.recover_truncated) {
    std::fprintf(stderr, "sifted_fusiond: recovered replay: %llu output lines before final flush\n",
                 static_cast<unsigned long long>(lines_before_finish));
  }
  if (o.hold && out.ws) {
    std::fprintf(stderr,
                 "sifted_fusiond: replay finished; serving the last snapshot (Ctrl-C to exit)\n");
    while (g_signal == 0) {
      std::this_thread::sleep_for(kHeartbeat);
      out.ws->republish();
    }
  }
  return 0;
}

int run_live(const Options& o) {
  const auto text = read_small_file(o.sensors_file, kMaxSensorTableTextSize);
  if (!text) {
    std::fprintf(stderr, "sifted_fusiond: cannot read %s\n", o.sensors_file.c_str());
    return 1;
  }
  const auto table = parse_sensor_table(*text);
  if (!table.ok()) {
    std::fprintf(stderr, "sifted_fusiond: %s: %s\n", o.sensors_file.c_str(),
                 std::string(to_string(table.error())).c_str());
    return 1;
  }

  auto udp = net::UdpReceiver::bind(o.listen_addr, o.listen_port);
  if (!udp.ok()) {
    std::fprintf(stderr, "sifted_fusiond: %s\n", udp.error().c_str());
    return 1;
  }
  Outputs out;
  if (!open_outputs(o, out)) {
    return 1;
  }
  std::unique_ptr<net::DurableFile> log_file;
  std::unique_ptr<std::ostream> log_stream;
  std::unique_ptr<replay::EventLogWriter> log;
  const replay::RunHeader header{table.value(), o.config};
  // Reported once, the moment it happens; tracking carries on without the log.
  bool log_ok = true;
  auto check_log = [&] {
    if (log_ok && log_file && log_file->failed()) {
      log_ok = false;
      std::fprintf(stderr,
                   "sifted_fusiond: ERROR: event log write failed (%s): recording stopped, "
                   "tracking continues\n",
                   log_file->error().c_str());
    }
  };
  if (!o.record_file.empty()) {
    net::DurableFile::Policy policy;
    policy.max_records = o.record_flush_records;
    policy.max_age = std::chrono::milliseconds(std::llround(o.record_flush_s * 1e3));
    policy.sync_interval = std::chrono::milliseconds(std::llround(o.record_sync_s * 1e3));
    auto created = net::DurableFile::create(o.record_file, policy);
    if (!created.ok()) {
      std::fprintf(stderr, "sifted_fusiond: %s\n", created.error().c_str());
      return 1;
    }
    log_file = std::move(created.value());
    log_stream = std::make_unique<std::ostream>(log_file.get());
    log = std::make_unique<replay::EventLogWriter>(*log_stream, header);
    // On the disk before the first frame: however early a crash comes, the
    // recording is a valid, replayable log.
    (void)log_file->sync_to_device();
    check_log();
  }
  net::Pipeline pipeline(header.sensors, header.config, out.line_sink(), out.snapshot_sink());

  net::SystemClock clock;
  net::BoundedQueue<net::Frame> queue(o.queue);
  net::UdpStats udp_stats;
  std::atomic<bool> stop{false};
  net::UdpReceiver receiver = std::move(udp.value());
  receiver.set_rate_limit(o.rate_limit);
  std::fprintf(stderr, "sifted_fusiond: %zu sensors, listening on udp %s:%u\n",
               header.sensors.size(), o.listen_addr.c_str(),
               static_cast<unsigned>(receiver.port()));
  std::thread rx([&] { receiver.run(stop, clock, queue, udp_stats); });

  // Receive times are clamped to be non-decreasing in consumption order (a
  // tick stamped here can race a frame stamped on the UDP thread), so the log
  // is always in time order; the clamped value is what is recorded and used.
  std::optional<Timestamp> last;
  auto stamp = [&](Timestamp t) {
    if (last && t < *last) {
      t = *last;
    }
    last = t;
    return t;
  };
  using SteadyClock = std::chrono::steady_clock;
  auto consume = [&](const net::Frame& f) {
    const Timestamp t = stamp(f.recv);
    const std::span<const std::uint8_t> bytes(f.bytes.data(), f.size);
    if (log) {
      (void)log->write_frame(t, bytes);
      log_file->record_written(SteadyClock::now());
      check_log();
    }
    pipeline.on_frame(t, bytes);
  };

  const auto idle_period_ns = std::llround(o.idle_tick_s * 1e9);
  std::optional<Timestamp> last_idle_tick;
  auto idle_tick_due = [&](Timestamp wall) {
    if (idle_period_ns <= 0) {
      return false;
    }
    const auto due = pipeline.next_idle_tick(idle_period_ns);
    return due && wall >= *due &&
           (!last_idle_tick || wall.ns - last_idle_tick->ns >= idle_period_ns);
  };

  auto next_stats = SteadyClock::now();
  auto next_output_flush = SteadyClock::now();
  auto next_heartbeat = SteadyClock::now() + kHeartbeat;
  std::uint64_t published = 0;
  while (g_signal == 0) {
    const auto f = queue.pop_for(std::chrono::milliseconds(100));
    if (f) {
      consume(*f);
    }
    // A flush needs the input to have gone quiet. Idle duties do not: junk that
    // keeps arriving must not hide the fact that no real data does.
    const Timestamp wall = clock.now();
    const bool flush_due = !f && pipeline.has_pending();
    const bool idle_due = !flush_due && idle_tick_due(wall);
    if (flush_due || idle_due) {
      // Recorded, so the replay acts at the same point.
      const Timestamp t = stamp(wall);
      if (idle_due) {
        last_idle_tick = t;
      }
      if (log) {
        (void)log->write_tick(t);
        log_file->record_written(SteadyClock::now());
        check_log();
      }
      pipeline.on_tick(t);
    }

    const auto steady = SteadyClock::now();
    if (log_file) {
      log_file->poll(steady);
      check_log();
    }
    if (out.file && steady >= next_output_flush) {
      out.file->flush();
      next_output_flush = steady + std::chrono::milliseconds(100);
    }
    if (out.ws && pipeline.snapshots() != published) {
      published = pipeline.snapshots();
      next_heartbeat = steady + kHeartbeat;
    } else if (out.ws && steady >= next_heartbeat) {
      out.ws->republish();
      next_heartbeat = steady + kHeartbeat;
    }
    if (o.stats_interval_s > 0 && steady >= next_stats) {
      next_stats += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(o.stats_interval_s));
      std::fprintf(stderr,
                   "sifted_fusiond: udp received %llu, rejected %llu, oversize %llu, rate-limited "
                   "%llu, queue-full drops %llu; ws clients %zu\n",
                   static_cast<unsigned long long>(udp_stats.received.load()),
                   static_cast<unsigned long long>(udp_stats.rejected.load()),
                   static_cast<unsigned long long>(udp_stats.oversize.load()),
                   static_cast<unsigned long long>(udp_stats.rate_limited.load()),
                   static_cast<unsigned long long>(udp_stats.queue_full.load()),
                   out.ws ? out.ws->clients() : 0U);
      print_pipeline_stats(pipeline);
      if (!log_ok) {
        std::fprintf(stderr, "sifted_fusiond: WARNING: event log write failed\n");
      }
    }
  }

  // Shutdown: stop receiving, drain what was accepted, flush the last scan.
  stop = true;
  rx.join();
  queue.close();
  while (auto f = queue.pop_for(std::chrono::milliseconds(0))) {
    consume(*f);
  }
  if (last) {
    pipeline.finish(*last);  // the replay's end-of-log flush uses the same time
  }
  if (log_file) {
    (void)log_file->sync_to_device();
    check_log();
  }
  if (out.file) {
    out.file->flush();
  }
  print_pipeline_stats(pipeline);
  std::fprintf(stderr, "sifted_fusiond: stopped (signal %d)\n", static_cast<int>(g_signal));
  return log_ok ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string_view first = argc > 1 ? argv[1] : "--help";
  if (first == "--version") {
    std::printf("sifted_fusiond %s\n", sifted::kVersion);
    return 0;
  }
  if (first == "--help" || first == "-h") {
    std::fputs(kUsage, stdout);
    return 0;
  }
  Options o;
  if (const std::string err = parse_args(argc, argv, o); !err.empty()) {
    std::fprintf(stderr, "sifted_fusiond: %s\n\n%s", err.c_str(), kUsage);
    return 2;
  }
  struct sigaction sa {};
  sa.sa_handler = on_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  try {
    return o.replay_file.empty() ? run_live(o) : run_replay(o);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "sifted_fusiond: %s\n", e.what());
    return 1;
  }
}
