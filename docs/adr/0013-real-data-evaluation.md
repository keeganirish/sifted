# ADR 0013: Real-data evaluation -- withheld-report scoring, downloaded not committed, one scorer

Status: accepted

## Context
Every accuracy figure in docs/evaluation.md comes from the project's own
simulator, whose noise matches the filter's assumptions exactly and whose
targets fly straight legs and constant-rate turns. Real ADS-B has realistic
motion and realistic errors, but no independent truth, a licence that forbids
redistribution, and aircraft identities that must not leak into the tracker or
into published results. The historical OpenSky state-vector dumps are no
longer published; what remains openly downloadable (2026-11) is a handful of
single-aircraft incident extracts.

## Decision
- **Two evaluations on the same cleaned flights.** A: real reports through the
  real codec, decoder and engine, scored predictively by withholding scheduled
  runs of each aircraft's reports and comparing the tracker's operator picture
  with them at their times, against holding and dead-reckoning the last
  report. B: interpolated real paths as truth for the existing seeded radar
  and camera simulators, as standard scenario directories, with a
  constant-velocity control over the same sensors and seeds.
- **One scorer.** A's scoring is `score_holdout` in `sifted_eval_support`
  (eval/sifted/eval/realdata.hpp), reading NIS with the same trace helper as
  the simulator scorer; B is scored by the unchanged `sifted_eval`. The
  simulator's 100 m match distance became a per-evaluation parameter whose
  default leaves docs/evaluation.md byte-identical.
- **Identity is a scoring label only.** Addresses become integer labels in
  files the engine never reads; frames, their order and the withholding
  schedule do not depend on the address (tested by renaming aircraft).
- **Data is downloaded, pinned, never committed.** A committed manifest holds
  URL, size, SHA-256, retrieval date, licence and descriptive fields; the
  archives live in git-ignored `recordings/`; tests use synthetic inputs only.
  The network is touched only by the explicit `fetch` step.
- **Rules before results.** Cleaning thresholds, windows, schedule, sensor
  geometry and the tuning/held-out split are named constants printed in the
  report; the only data-driven choices (interpolation method, a diagnostic
  sigma) are made on the tuning flights; defaults of the tracker and adapter
  are not changed on the strength of these results.

## Consequences
- The report shows unfavourable results as they are (prediction worse than
  dead reckoning, an inconsistent ADS-B noise assumption).
- One aircraft per published extract means identity swaps and dense traffic
  are not measured on real data; the metrics exist and are tested on
  synthetic multi-aircraft input, ready for a multi-aircraft recording.
- Anyone reproducing the report needs the archives from OpenSky, under its
  terms; if they disappear, the manifest still documents exactly what was used.
