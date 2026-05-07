#pragma once

namespace hakoniwa::drone::control_adapter {

struct VerticalPositionState {
    double z{0.0};
};

struct VerticalVelocityState {
    double vz{0.0};
};

struct VerticalAccelerationState {
    double az{0.0};
};

struct AltitudeControlInput {
    VerticalPositionState position{};
    VerticalVelocityState velocity{};
    VerticalAccelerationState acceleration{};
    double target_altitude{0.0};
};

struct NormalizedVerticalThrustCommand {
    double body_z{0.0};
};

class IAltitudeControlBackend {
public:
    virtual ~IAltitudeControlBackend() = default;

    virtual void reset() = 0;

    virtual NormalizedVerticalThrustCommand run(const AltitudeControlInput& input, double dt_sec) = 0;
};

}  // namespace hakoniwa::drone::control_adapter
