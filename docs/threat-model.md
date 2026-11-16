# Threat model

> Status: final for M3. Scope is a demo-grade system, but it parses network
> input, so the parser boundary is treated seriously. Every threat below has
> a final status: mitigated (with the test that shows it), or accepted (with
> the reasoning).

## Assets

- Integrity of the track picture (operators must not be misled by forged or garbled data).
- Availability of the fusion service under hostile or broken input.
- Memory safety of the process.

## Trust boundaries

| # | Boundary | Trust |
|---|---|---|
| B1 | UDP datagrams from sensors → ingest | **Untrusted.** Any host that can reach the port may send anything. |
| B2 | OpenSky API response → ADS-B adapter | **Untrusted.** Public third-party data; may be malformed, stale, or spoofed. |
| B3 | WebSocket clients → publisher | **Untrusted** (read-only subscribers; no commands accepted in v1). |
| B4 | Event log file → replayer | Semi-trusted; still parsed with the same strict decoder. |
| B5 | Sensor table file → `parse_sensor_table` | Trusted operator configuration (ADR 0003); still parsed strictly with size and count limits. Never accepted from the network. |

## Threats and mitigations

| Threat | Boundary | Mitigation | Status |
|---|---|---|---|
| Malformed/oversized packet causes crash or OOB read | B1 | Strict decoder, hard payload cap checked first, bounds-checked reads; oversize datagrams dropped before decoding; unit tests; libFuzzer + ASan/UBSan (`fuzz_decode`); malformed bursts against the live receive path (`UdpFlood.MalformedBurstIsRejectedAndNeverQueued`) | **mitigated** |
| Non-finite or absurd values poison a filter (NaN propagation) | B1, B2 | `is_valid()` rejects NaN/Inf/out-of-range; canonical encoding; the engine fuzz target checks every live track stays finite on arbitrary input (`fuzz_engine`) | **mitigated** |
| Replay of duplicate datagrams | B1 | per-sensor `(seq, time)` window in ingest (ADR 0006) plus the engine's last-`seq` check; duplicate floods tested live (`UdpFlood.DuplicateFloodFromOneSourceDoesNotStarveAnother`) | **mitigated** |
| Sensor restart mistaken for replay (seq reset) | B1 | dedup keys on `(seq, time)`; tested bit-for-bit (`Faults.SensorRestartIsTransparent`) | **mitigated** |
| Forged sensor geometry (lying about pose) | B1 | poses come only from the operator's sensor table, never from frames (ADR 0003) | **mitigated** |
| Measurement kind not matching the configured sensor | B1 | engine rejects kind mismatches and unknown sensor ids, and counts them | **mitigated** |
| Reordered / delayed / lost packets | B1 | bounded reorder stage in ingest (ADR 0009, daemon default 0.5 s); lifecycle coasting bridges gaps; degraded-network tests across four impairment levels and outages (`DegradedNetwork.*`, docs/design.md §9) | **mitigated** (bounded: data later than the window is dropped as OutOfOrder and counted) |
| Clock skew or time going backwards | B1 | `SimulatedClock` refuses to go back; engine drops measurements older than its clock and counts them; the daemon clamps receive times to be non-decreasing; **clock gate** rejects frames stamped too far ahead of their receive time (ADR 0010; `Faults.ClockFaultsAndTheirMitigation`) | **mitigated when configured** (`--max-time-ahead`, needs wall-clock senders); a slow clock only loses its own sensor's data |
| Degenerate geometry or bad values destabilise a filter | B1 | filter validates inputs, uses Cholesky + Joseph form, and commits only finite results; engine applies updates on a copy | **mitigated** |
| Flood / resource exhaustion | B1, B3 | per-source-address token bucket before decoding (ADR 0010; daemon default 20 000/s, burst 4 096, 1 024 sources LRU); bounded queue with drop counter; bounded track count, trace windows, dropped-track history, measurement log, reorder stage, dedup window; WebSocket client cap and per-client single outgoing frame (`UdpFlood.*`, `Faults.ClutterFloodRespectsTheTrackCapAndKeepsEstablishedTracks`, `fuzz_engine` invariants) | **mitigated** |
| Track-table exhaustion by clutter | B1 | hard `max_tracks` cap (TrackLimit counted); established tracks are associated before tentative ones, so real targets keep their tracks under a 300-false-alarm-per-scan flood | **mitigated**; a *new* target cannot start a track while the table is full (visible in the TrackLimit counter) -- accepted cost of a hard cap |
| Spoofed sensor identity | B1 | **Out of scope in v1** (no authentication). Would need HMAC/keys. | accepted |
| Spoofed source addresses defeat per-source limits | B1 | UDP source addresses are unauthenticated; a spoofer gets a fresh bucket per address. The bounded queue and its counter remain the backstop; the engine can never be stalled, only starved of input | accepted |
| Forged ADS-B (spoofing) | B2 | Out of scope; cross-sensor consistency checks are a possible later feature | accepted |
| Malformed OpenSky JSON crashes the adapter or yields invalid frames | B2 | adapter validates every field; property tests with random and mutated JSON (`sim/tests/test_adsb_fuzz.py`). M3 found and fixed one bug: a `time_position` whose nanosecond value overflows int64 passed validation and made `encode()` raise | **mitigated** |
| Malicious WebSocket client (bad handshake, data frames, floods) | B3 | strict handshake parser, read-only frame policy (control frames only, ≤ 125 bytes), request and frame size caps, client cap, stall timeout; both parsers fuzzed (`fuzz_websocket`) | **mitigated** |
| Corrupt or hostile event log | B4 | size checks before allocation, CRCs, strict header parser (canonical round trip), time order; frames re-decoded; fuzzed (`fuzz_event_log`, `fuzz_log_header`) | **mitigated** |
| Malformed sensor table | B5 | strict parser with size and count caps; fuzzed (`fuzz_sensor_table`) | **mitigated** |
| Data race on shared state | all | single engine thread, ownership rules (docs/design.md §2), TSan on the full test suite, including the live UDP flood tests with a concurrent consumer | **mitigated** |

## Sensor and input faults

What the system does under each injected fault (`tests/fault_injection_test.cpp`,
`tests/udp_flood_test.cpp`; the same faults can be injected live with
`python3 -m sifted_sim.send ... --fault SPEC`, see `sim/sifted_sim/faults.py`).
Numbers: GCC 13 release, the three M2 multi-target scenarios.

| Fault | Expected behaviour | Measured |
|---|---|---|
| Sensor dropout (radar silent 6 s) | tracks coast on prediction (camera bearings refine but cannot hold them) and are held again on the first scans back | recovered within 0-2 s; straight-flying targets kept their ids; close_pass's manoeuvring target was re-initiated |
| Intermittent sensor (camera off in 40% of 2 s periods) | fusion gain shrinks, nothing breaks | no ID switches or false tracks; max RMSE 9.5-12.6 m (baseline 8.6-12.2 m) |
| Sensor restart (seq reset, twice) | transparent | track picture bit-for-bit identical to the run without restarts; 0 duplicates |
| Clock fast by 2 s (camera) | unmitigated: the camera drags the engine clock ahead and the radar's data is dropped as OutOfOrder; with the clock gate: the camera's frames are refused, radar-only tracking | unmitigated: 223-379 radar measurements lost, 97-100% of target-instants missed; gated: all camera frames refused, 0 OutOfOrder, identity intact |
| Clock fast by 2 s (radar, the only track-starting sensor) | gated: no tracks at all (integrity over availability) | gated: 0 tracks started; unmitigated: tracks reported 2 s "in the future" (crossing_pair: none reported at all; others: RMSE 93-99 m) |
| Clock slow by 2 s | only the slow sensor's own data is lost | all camera measurements OutOfOrder; radar tracking unaffected |
| Clock drift 5000 ppm | absorbed by the reorder window while small; refused by the gate when larger | unmitigated: 100-225 measurements lost; with window 0.5 s + gate 0.5 s: 0 lost until the lead exceeds 0.5 s, then that sensor's frames are refused |
| Range bias 50 m (radar) | not detectable from one sensor (registration problem; out of scope); the picture shifts by the bias | radar alone: RMSE ≈ 50 m, identity intact; fused with the unbiased camera: 1-6 ID switches as the two disagree |
| Stuck radar (repeats one scan for 8 s) | frozen detections become stationary (false) tracks; real tracks coast and recover when the radar does | ≤ 1 false track per frozen detection (2-4), recovered within 2 s, cap never reached |
| Clutter flood (300 false alarms/scan, cap 64 tracks) | cap holds, every bounded buffer holds, established tracks keep their targets | live tracks ≤ 64 at every scan; > 1000 TrackLimit outcomes; missed ≤ 5% where all targets were confirmed before the flood |
| Duplicate flood (20 000 copies of one datagram) | rate-limited per source; admitted copies suppressed as duplicates; other sources unaffected | > 50% rate-limited; every honest datagram that reached the socket was processed; every datagram counted exactly once |
| Malformed burst (5 000 random datagrams) | rejected before the queue | 0 enqueued; all counted as oversize or rejected |
| Engine queue full (no consumer) | queue holds at capacity; the rest counted | exactly `capacity` enqueued, the rest `queue_full` |

## Out of scope

Confidentiality, authentication, and physical attacks. This is a demonstration
system running on public data and simulations.
