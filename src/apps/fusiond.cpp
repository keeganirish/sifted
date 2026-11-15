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
#include <atomic>
#include <charconv>
#include <chrono>
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
    "  tracker parameters (defaults: docs/design.md section 6):\n"
    "  --accel-psd Q  --initial-velocity-sigma V  --gate-probability P\n"
    "  --confirm-hits N  --tentative-timeout S  --coast-after S  --drop-after S\n"
    "  --max-position-sigma M  --max-tracks N  --snapshot-interval S\n"
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
    "  --hold                  keep serving the last snapshot after the replay ends\n";

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
  double stats_interval_s = 5.0;
  double pace = 0.0;
  bool hold = false;
  bool tuning_given = false;
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
    } else if (a == "--stats-interval") {
      ok = num(o.stats_interval_s) && o.stats_interval_s >= 0.0;
    } else if (a == "--pace") {
      ok = num(o.pace) && o.pace >= 0.0;
    } else if (a == "--hold") {
      o.hold = true;
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
    } else {
      return "unknown option: " + std::string(a);
    }
    if (!ok) {
      return "missing or invalid value for " + std::string(a);
    }
  }
  if (o.replay_file.empty() == o.sensors_file.empty()) {
    return "give exactly one of --sensors (live) or --replay";
  }
  if (!o.replay_file.empty() && (o.tuning_given || !o.record_file.empty())) {
    return "--replay takes its parameters from the log; tracker options and --record do not apply";
  }
  if (o.replay_file.empty() && !net::is_valid(o.config)) {
    return "invalid tracker parameters (see docs/design.md section 6 for the constraints)";
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
      "snapshots %llu\n",
      static_cast<unsigned long long>(p.frames()), static_cast<unsigned long long>(p.scans()),
      static_cast<unsigned long long>(p.duplicates()),
      static_cast<unsigned long long>(e.measurements_processed()),
      static_cast<unsigned long long>(e.count(MeasurementOutcome::Updated)),
      static_cast<unsigned long long>(e.count(MeasurementOutcome::Initialized)),
      static_cast<unsigned long long>(e.count(MeasurementOutcome::OutOfOrder)),
      static_cast<unsigned long long>(e.count(MeasurementOutcome::TrackLimit)), e.tracks().size(),
      static_cast<unsigned long long>(p.snapshots()));
}

int run_replay(const Options& o) {
  std::ifstream in(o.replay_file, std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "sifted_fusiond: cannot read %s\n", o.replay_file.c_str());
    return 1;
  }
  auto reader = replay::EventLogReader::open(in);
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
  net::Pipeline pipeline(r.header().sensors, r.header().config, out.line_sink(),
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
    std::this_thread::sleep_until(wall0 + std::chrono::duration<double>(offset_s));
  };
  const auto result = replay::replay(r, pipeline, pace);
  if (out.file) {
    out.file->flush();
  }
  print_pipeline_stats(pipeline);
  if (!result.ok()) {
    std::fprintf(stderr, "sifted_fusiond: replay stopped: %s\n",
                 std::string(replay::to_string(result.error())).c_str());
    return 1;
  }
  if (o.hold && out.ws) {
    std::fprintf(stderr,
                 "sifted_fusiond: replay finished; serving the last snapshot (Ctrl-C to exit)\n");
    while (g_signal == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
  std::unique_ptr<std::ofstream> log_file;
  std::unique_ptr<replay::EventLogWriter> log;
  const replay::RunHeader header{table.value(), o.config};
  if (!o.record_file.empty()) {
    log_file = std::make_unique<std::ofstream>(o.record_file, std::ios::binary | std::ios::trunc);
    if (!*log_file) {
      std::fprintf(stderr, "sifted_fusiond: cannot write %s\n", o.record_file.c_str());
      return 1;
    }
    log = std::make_unique<replay::EventLogWriter>(*log_file, header);
  }
  net::Pipeline pipeline(header.sensors, header.config, out.line_sink(), out.snapshot_sink());

  net::SystemClock clock;
  net::BoundedQueue<net::Frame> queue(o.queue);
  net::UdpStats udp_stats;
  std::atomic<bool> stop{false};
  net::UdpReceiver receiver = std::move(udp.value());
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
  bool log_ok = true;
  auto consume = [&](const net::Frame& f) {
    const Timestamp t = stamp(f.recv);
    const std::span<const std::uint8_t> bytes(f.bytes.data(), f.size);
    if (log) {
      log_ok = log->write_frame(t, bytes) && log_ok;
    }
    pipeline.on_frame(t, bytes);
  };

  auto next_stats = std::chrono::steady_clock::now();
  while (g_signal == 0) {
    if (auto f = queue.pop_for(std::chrono::milliseconds(100))) {
      consume(*f);
    } else if (pipeline.has_pending()) {
      // The input went quiet with a scan pending: flush it, and record that
      // we did, so the replay flushes at the same point.
      const Timestamp t = stamp(clock.now());
      if (log) {
        log_ok = log->write_tick(t) && log_ok;
      }
      pipeline.on_tick(t);
    }
    if (o.stats_interval_s > 0 && std::chrono::steady_clock::now() >= next_stats) {
      next_stats += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(o.stats_interval_s));
      std::fprintf(stderr,
                   "sifted_fusiond: udp received %llu, rejected %llu, oversize %llu, queue-full "
                   "drops %llu; ws clients %zu\n",
                   static_cast<unsigned long long>(udp_stats.received.load()),
                   static_cast<unsigned long long>(udp_stats.rejected.load()),
                   static_cast<unsigned long long>(udp_stats.oversize.load()),
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
    log_file->flush();
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
