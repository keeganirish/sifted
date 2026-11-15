# estimation

Kalman-family track filters (Eigen). One filter instance per track, holding the state vector and covariance. Predict/update take the measurement and the time delta explicitly; no clocks, no globals.

- `cv_filter.hpp`: `ConstantVelocityFilter`, a constant-velocity Kalman filter in the local x/y frame (`predict(dt)`, linear `update(z, R)`, the shared Joseph-form `update_with_innovation`, read-only `mahalanobis2` for gating, `set_accel_psd` for manoeuvre handling).
- `range_bearing.hpp`: the range-bearing measurement model for the EKF: `h(x)`, its Jacobian, the inverse model used to start tracks, `wrap_angle`, `range_bearing_innovation` and `update_range_bearing`.
- `bearing_only.hpp` (M2): the bearing-only (camera) model: `predict_bearing`, `bearing_jacobian`, wrapped `bearing_innovation`, `update_bearing_only`. Cannot start a track.
- `geodetic.hpp` (M2): WGS-84 geodetic / ECEF / ENU conversions and the projection of geodetic position measurements (with noise) into the tracking frame.
- `measurement_model.hpp`, `linearized.hpp` (M2): one `(y, H, R)` shape for every sensor kind, used by gating and updates.

Design notes and tuning: `docs/design.md` sections 6-7. M1, extended in M2.
