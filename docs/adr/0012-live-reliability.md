# ADR 0012: Live reliability -- timeline guard, honest silence, crash-durable recording

Status: accepted

## Context
Black-box testing of a Release daemon found three defects that unit tests,
sanitizers and fuzzers could not see, because each is a property of the running
service rather than of one function:

1. **One forged datagram disabled tracking for good.** The engine drops anything
   older than its clock and advances the clock to whatever it accepts. One
   well-formed frame stamped `4e18` from a known sensor moved the clock there, and
   every genuine frame afterwards was OutOfOrder. The clock gate (ADR 0010) cannot
   be the default defence: it needs a common timebase, and simulated sensors have
   none.
2. **A total outage looked like a healthy picture.** The engine clock, the
   lifecycle and the snapshot stream only move when measurements arrive. After 25 s
   of silence the operator had received 0 messages and still saw every track as
   `confirmed`.
3. **A crash destroyed the recording.** The event log sat in a stream buffer until
   clean shutdown. `SIGKILL` left a 0-byte file, and a log with a torn tail could
   not be replayed at all.

## Decision

### 1. Timeline guard (`max_time_jump_s`, `timeline_confirm_frames`, `timeline_confirm_s`)
`net::Pipeline` judges each frame against the measurement times it has already
accepted, so no shared timebase is needed. Let *head* be the newest accepted time.
A frame is **on the timeline** if it is at most `max_time_jump_s` behind head, or
ahead of head by at most `max_time_jump_s` plus the receive time elapsed since head
was received. Elapsed time is what lets ADS-B gaps of minutes through unquestioned.

Anything else is **quarantined** before the duplicate filter, the reorder stage and
scan assembly, so a rejected frame cannot move the clock, split a scan, move the
reorder watermark or suppress a later genuine frame. Quarantined frames group into
at most four candidate timelines.

A candidate replaces the current timeline only when **both** hold:

- it has `timeline_confirm_frames` *distinct* frames that agree with each other
  (a repeated datagram is not a second witness), and
- `timeline_confirm_s` of receive time have passed since its first frame *and*
  since the last frame on the current timeline.

Any frame on the current timeline discards all candidates, so **a timeline that is
still being heard cannot be displaced**, however many forged frames arrive. Adopted
frames are released in measurement-time order, so a genuine change loses no data
(up to `reorder_capacity` frames per candidate). A move to an *earlier* timeline
restarts the tracker (`Engine::restart_timeline`; track ids keep counting up),
because the engine clock cannot run backwards.

At start-up there is no timeline to protect: the first candidate with enough frames
is adopted at once, and a lone frame after the confirmation time. If that choice was
forged, the genuine timeline replaces it by the same rule, so a run always converges.

Defaults: both durations equal `coast_after_s` (a jump that small cannot even make a
track coast, and the value already expresses how long this deployment's sensors may
pause), 3 frames. Off in the library and in older recordings.

**Recovery bound.** A genuine timeline change takes effect
`timeline_confirm_s` of receive time after the last frame of the old timeline, once
`timeline_confirm_frames` frames have arrived (2.5 s and 3 frames by default), plus
at most one idle tick.

### 2. Honest silence (`feed_stale_factor`, `feed_stale_min_s`)
- **Staleness follows each sensor's cadence.** A sensor is stale after
  `max(feed_stale_min_s, feed_stale_factor x median of its last five inter-scan
  receive gaps)`. The median ignores a single outage, and a poller's long gaps
  become its normal. Default: 3 x the median, at least `coast_after_s`, so a 1 Hz
  radar goes stale after 3 s. The launcher raises the floor for ADS-B until the
  polling cadence has been observed.
- **Snapshots carry a `feed` object** (`waiting`, `live`, `degraded`, `stale`,
  overall and per sensor). It is additive, the version stays 2, and it is absent
  from older recordings. No separate status message exists, so the WebSocket's
  latest-message slot always holds a complete picture for a new client.
- **Idle time reaches the pipeline as recorded ticks.** During total silence each
  tick publishes a snapshot predicted to the current receive time. Its tracks come
  from `Engine::tracks_if_silent_until`, which applies the lifecycle rules read-only:
  confirmed tracks past `coast_after_s` are shown coasting, and tracks past
  `drop_after_s` are no longer shown. **The engine's state does not change**, so ticks
  can change what is displayed but never what is tracked. Coast and drop behaviour,
  outage recovery and the scorecard are exactly what they were.
- **Ticks stop.** `Pipeline::next_idle_tick` asks for a tick only when one would
  change something, at most one per `--idle-tick` (1 s). Once every track has left
  the picture, none is due, so a silent run's log stops growing.
- **The service proves it is alive.** The engine thread repeats the latest snapshot
  unchanged every second (`WebSocketServer::republish`), which touches neither the
  log nor the output. The browser raises "service not responding" after 5 s without
  any message: only the client can detect a hung daemon.

**Staleness bound.** The operator sees a stale feed within the threshold above plus
one idle tick (at most 1 s): about 4 s for a 1 Hz radar. A hung service is flagged
within 6 s.

### 3. Crash-durable recording (`net::DurableFile`)
The durability policy is at the daemon edge; `sifted_core` still writes to an
`ostream`.

| Failure | What survives | Default bound on the loss |
|---|---|---|
| Process death (crash, `SIGKILL`, OOM kill) | everything handed to the OS | the newest 64 records and at most about 200 ms of input (`--record-flush-records`, `--record-flush` plus one 100 ms engine loop) |
| Power loss or kernel crash | everything `fdatasync` completed | about 1.2 s (`--record-sync` plus the above), if the device honours flushes |

The header is synced before the first frame, so any crash after start-up leaves a
valid log. `fdatasync` runs on its own thread so a slow disk never stalls the engine.

A process dies between whole `write` calls, so at most the final record is torn.
`TailPolicy::RecoverTorn` (`--recover-truncated`) replays the complete prefix and
reports the discarded bytes. The strict reader is unchanged and remains the default.
Damage anywhere but the final record stays a hard error.

A failed write (a full disk) is reported at once on stderr and in the operator view.
Tracking continues, recording stops, and the file remains a valid prefix.

**Verifying an incomplete run.** `live.txt` and the log are cut off independently by
a crash, and the replay's final flush never happened live. Verify therefore compares
the complete lines both hold, excluding that flush, and reports a distinct result:
"partial", with the number of lines checked. It never reports a crashed run as
identical.

## Alternatives rejected
- **Turning the clock gate on by default.** It needs a common timebase, and rejects
  every simulated scenario.
- **Median-of-sensors clock voting.** One sensor is a normal deployment, and a vote
  would still let a forger with several sensor ids win.
- **Advancing the engine clock on idle ticks.** A slow or accelerated sender would
  turn its own next frame OutOfOrder, and results would depend on wall-clock pacing.
- **A separate status message.** It would displace the snapshot in the latest-message
  slot, and older pages reject unknown types.
- **`fsync` on every record.** It costs milliseconds per record on the engine thread
  for a guarantee few deployments need.

## Consequences
+ A single forged, duplicated or malformed timestamp costs one counted frame.
+ Silence, degradation and a hung service are three different things on screen.
+ A crash costs a bounded tail of the recording, and the rest replays.
- Start-up holds the first frames until `timeline_confirm_frames` agree.
- A genuine jump larger than the allowance pauses output for `timeline_confirm_s`.
  It happens only for senders running faster than real time across a gap.
- One sensor on a different epoch from a healthy majority stays rejected, and
  counted, for as long as the others are heard. One engine has one clock.

## Residual risk (no authentication, see the threat model)
Anyone who can reach the UDP port and name a sensor can still:
- walk the timeline forward in steps smaller than `max_time_jump_s`, making genuine
  data late for as long as the attack continues;
- take over the timeline during a silence longer than `timeline_confirm_s` by sending
  enough consistent frames. It lasts until genuine data returns, which restarts the
  tracker;
- inject plausible measurements on the current timeline, which is spoofing and is
  out of scope.

`--max-time-ahead` with wall-clock senders caps the first two at the gate's limit.
