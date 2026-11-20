# estimation

Kalman-family track filters (Eigen). One filter instance per track, holding the state vector and covariance. Predict/update take the measurement and the time delta explicitly; no clocks, no globals.

- `cv_filter.hpp`: `ConstantVelocityFilter`, a constant-velocity Kalman filter in the local x/y frame (`predict(dt)`, linear `update(z, R)`, the shared Joseph-form `update_with_innovation`, read-only `mahalanobis2` for gating, `set_accel_psd` for the legacy manoeuvre switch).
- `imm_filter.hpp` (ADR 0014): `ImmFilter`, the Interacting Multiple Model estimator. Three CV modes (quiet / manoeuvre / hard process noise) in the same state, a continuous-time Markov chain over mean sojourn times (`transition_matrix`, exact identity at `dt = 0`), mixing with the spread-of-means term, mode-matched Joseph-form updates through a per-mode linearisation callback, log-space mode likelihoods with a probability floor, and the moment-matched output. All or nothing: a failed mode rejects the update.
- `track_filter.hpp` (ADR 0014): `TrackFilter`, what a track holds: the CV filter (legacy, `motion_model = cv`) or the IMM. It gives the engine one interface: the combined estimate, the Gaussian components behind it (for the gate screen), the gating statistic (`gate`: exactly the CV `d²`, or the IMM's mixture statistic), and `update(ctx, l)`, which re-linearises the measurement at every IMM mode.
- `range_bearing.hpp`: the range-bearing measurement model for the EKF: `h(x)`, its Jacobian, the inverse model used to start tracks, `wrap_angle`, `range_bearing_innovation` and `update_range_bearing`.
- `bearing_only.hpp` (M2): the bearing-only (camera) model: `predict_bearing`, `bearing_jacobian`, wrapped `bearing_innovation`, `update_bearing_only`. Cannot start a track.
- `geodetic.hpp` (M2): WGS-84 geodetic / ECEF / ENU conversions and the projection of geodetic position measurements (with noise) into the tracking frame.
- `measurement_model.hpp`, `linearized.hpp` (M2): one `(y, H, R)` shape for every sensor kind, used by gating and updates, and the gate screen with its proof (for one Gaussian and for the IMM's mixture).

Design notes and tuning: `docs/design.md` sections 6-7, ADR 0014. M1, extended in M2 and by ADR 0014.
