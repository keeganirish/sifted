# Threat model

> Status: skeleton for M0. Scope is a demo-grade system, but it parses
> network input, so the parser boundary is treated seriously.

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

## Threats and mitigations

| Threat | Boundary | Mitigation | Status |
|---|---|---|---|
| Malformed/oversized packet causes crash or OOB read | B1 | Strict decoder, hard payload cap checked first, bounds-checked reads, unit tests, libFuzzer + ASan/UBSan | **in place** |
| Non-finite or absurd values poison a filter (NaN propagation) | B1, B2 | `is_valid()` rejects NaN/Inf/out-of-range; canonical encoding | **in place** |
| Replay of duplicate datagrams | B1 | per-sensor `seq` field; duplicate suppression in ingest | TODO (M2) |
| Reordered / delayed / lost packets | B1 | timestamp + seq handling; degraded-network tests | TODO (M3) |
| Clock skew or time going backwards | B1 | `SimulatedClock` refuses to go back; engine policy for stale data | partial |
| Flood / resource exhaustion | B1, B3 | bounded queue with drop counter; per-source rate limiting; bounded track count | TODO (M2) |
| Spoofed sensor identity | B1 | **Out of scope in v1** (no authentication). Documented limitation; would need HMAC/keys. | accepted |
| Forged ADS-B (spoofing) | B2 | Out of scope; cross-sensor consistency checks are a possible later feature | accepted |
| Data race on shared state | all | single engine thread, ownership rules, TSan in CI | TODO (M2) |

## Out of scope

Confidentiality, authentication, and physical attacks. This is a demonstration
system running on public data and simulations.
