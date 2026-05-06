# hakoniwa-drone-control-adapter
Common control adapter interfaces for connecting drone flight controllers to the Hakoniwa simulation runtime.

## Scope

This repository contains public control adapter interfaces only.

- No PX4-specific source code
- No Hakoniwa runtime orchestration
- No backend implementations yet

The initial interface is `IRateControlBackend`, designed as a PX4-first,
single-step control contract that can also be adapted to the current
Hakoniwa native rate controller.

## Current Interface

Header:

- `include/hakoniwa/drone/control_adapter/rate_control_backend.hpp`

Key design points:

- one `run()` call represents one control-step evaluation
- input includes angular rate, angular acceleration, target rate, `dt`,
  landed flag, and saturation feedback
- output is body torque only
- mixer/control allocation is intentionally out of scope
