# hakoniwa-drone-control-adapter
Common control adapter interfaces for connecting drone flight controllers to the Hakoniwa simulation runtime.

## Why This Exists

Hakoniwa already has its own internal drone controllers inside the drone
simulator stack.

Related public simulator core:

- `hakoniwa-drone-core`
  - https://github.com/toppers/hakoniwa-drone-core

That internal control path is useful for simulation, but connecting a real
flight-controller stack such as PX4 through external TCP/UDP communication is
not ideal for large-scale PID autotuning.

Typical problems with an external runtime connection are:

- process boundary overhead
- TCP/UDP communication overhead
- runtime scheduling overhead outside Hakoniwa
- difficulty running large parameter sweeps quickly

This repository exists to cut the drone control boundary out as a public,
backend-facing interface.

The goal is:

- keep Hakoniwa's fast trial/evaluation/autotune workflow
- replace only the control core behind a stable interface
- make external flight-controller logic reusable without requiring a full
  out-of-process integration path for every tuning run

## Why It Matters

With this adapter boundary, PX4-based control logic can be executed inside a
Hakoniwa-oriented tuning workflow instead of only through an external networked
integration.

That makes it possible to:

- run PX4-based PID autotuning at high volume
- perform large parameter sweeps efficiently
- reuse Hakoniwa replay and regression style workflows
- compare native Hakoniwa control and external controller cores through the
  same public contract

This is not just a connectivity layer.
It is the control boundary that makes high-throughput autotuning practical for
external flight-controller implementations.

## Related Repositories

- `hakoniwa-drone-core`
  - public simulator core
  - https://github.com/toppers/hakoniwa-drone-core
- `hakoniwa-drone-control-adapter-px4`
  - PX4-specific backend implementation of this adapter interface
  - located alongside this repository in the current workspace under
    `work/hakoniwa-drone-control-adapter-px4`

## Scope

This repository contains public control adapter interfaces only.

- No PX4-specific source code
- No Hakoniwa runtime orchestration
- No backend implementations in this repository

The initial interfaces are `IRateControlBackend`,
`IAttitudeControlBackend`, `IAltitudeControlBackend`,
`IHorizontalPositionControlBackend`, and `IControlAllocationBackend`, all
designed as PX4-first or PX4-compatible single-step control contracts that can
also be adapted to the current Hakoniwa native controllers.

This repository also carries adapter-side shared test sources that can be
concretized by backend repositories.

Those shared test sources are intended to serve as:

- executable usage examples
- contract-level backend conformance checks

Concrete execution remains the responsibility of backend repositories.

## Current Interface

Header:

- `include/hakoniwa/drone/control_adapter/rate_control_backend.hpp`
- `include/hakoniwa/drone/control_adapter/attitude_control_backend.hpp`
- `include/hakoniwa/drone/control_adapter/altitude_control_backend.hpp`
- `include/hakoniwa/drone/control_adapter/horizontal_position_control_backend.hpp`
- `include/hakoniwa/drone/control_adapter/control_allocation_backend.hpp`

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
- control-allocation input includes thrust, body torque, actuator geometry,
  actuator limits, trim, and linearization point
- control-allocation output includes actuator commands plus minimal allocation
  status

## Implementation Notes

This repository defines the public control contract.

Backend-specific implementation, configuration loading, converter tooling, and
executable validation belong in backend repositories such as:

- `hakoniwa-drone-control-adapter-px4`

The intended architecture is:

1. Hakoniwa simulator and tuning workflow on one side
2. public control adapter interface in this repository
3. concrete backend implementation in a backend-specific repository

That split keeps simulator-specific know-how and backend-specific integration
details separated while preserving a reusable public control boundary.
