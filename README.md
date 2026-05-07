# hakoniwa-drone-control-adapter
Common control adapter interfaces for connecting drone flight controllers to the Hakoniwa simulation runtime.

## Scope

This repository contains public control adapter interfaces only.

- No PX4-specific source code
- No Hakoniwa runtime orchestration
- No backend implementations yet

The initial interfaces are `IRateControlBackend`,
`IAttitudeControlBackend`, `IAltitudeControlBackend`, and
`IHorizontalPositionControlBackend`, all designed as PX4-first single-step
control contracts that can also be adapted to the current Hakoniwa native
controllers.

## Current Interface

Header:

- `include/hakoniwa/drone/control_adapter/rate_control_backend.hpp`
- `include/hakoniwa/drone/control_adapter/attitude_control_backend.hpp`
- `include/hakoniwa/drone/control_adapter/altitude_control_backend.hpp`
- `include/hakoniwa/drone/control_adapter/horizontal_position_control_backend.hpp`

Key design points:

- one `run()` call represents one control-step evaluation
- rate-control input includes angular rate, angular acceleration, target
  rate, `dt`, landed flag, and saturation feedback
- rate-control output is body torque only
- attitude-control input is quaternion attitude plus target quaternion and
  yaw-rate feed-forward
- attitude-control output is a body-frame angular-rate target
- altitude-control input is mode-aware and includes vertical position,
  velocity, acceleration, plus either target altitude or target vertical
  velocity
- altitude-control output is normalized vertical thrust
- horizontal-position-control input is mode-aware and includes x/y position,
  velocity, acceleration, current yaw, plus either target x/y position or
  target x/y velocity
- horizontal-position-control output is a target roll/pitch tilt pair
- horizontal-position-control tilt output is a derived adapter contract for
  Hakoniwa compatibility, not a direct PX4 public-core type
- mixer/control allocation is intentionally out of scope
