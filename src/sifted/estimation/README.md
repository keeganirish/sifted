# estimation

Kalman-family track filters (Eigen). One filter instance per track, holding the state vector and covariance. Predict/update take the measurement and the time delta explicitly; no clocks, no globals.

- `cv_filter.hpp`: `ConstantVelocityFilter`, a constant-velocity Kalman filter in the local x/y frame (`predict(dt)`, linear `update(z, R)`, and the shared Joseph-form `update_with_innovation`).
- `range_bearing.hpp`: the range-bearing measurement model for the EKF: `h(x)`, its Jacobian, the inverse model used to start tracks, `wrap_angle`, and `update_range_bearing`.

Design notes and tuning: `docs/design.md` sections 6-7. Implemented in M1.
