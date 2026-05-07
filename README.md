# hakoniwa-drone-control-adapter
Common control adapter interfaces for connecting drone flight controllers to the Hakoniwa simulation runtime.

## Scope

This repository contains public control adapter interfaces only.

- No PX4-specific source code
- No Hakoniwa runtime orchestration
- No backend implementations yet

The initial interfaces are `IRateControlBackend` and
`IAttitudeControlBackend`, both designed as PX4-first single-step control
contracts that can also be adapted to the current Hakoniwa native
controllers.

## Current Interface

Header:

- `include/hakoniwa/drone/control_adapter/rate_control_backend.hpp`
- `include/hakoniwa/drone/control_adapter/attitude_control_backend.hpp`

Key design points:

- one `run()` call represents one control-step evaluation
- rate-control input includes angular rate, angular acceleration, target
  rate, `dt`, landed flag, and saturation feedback
- rate-control output is body torque only
- attitude-control input is quaternion attitude plus target quaternion and
  yaw-rate feed-forward
- attitude-control output is a body-frame angular-rate target
- mixer/control allocation is intentionally out of scope
