#pragma once

namespace hakoniwa::drone::control_adapter {

enum class HorizontalControlMode {
    Position,
    Velocity
};

struct HorizontalPositionState {
    double x{0.0};
    double y{0.0};
};

struct HorizontalVelocityState {
    double vx{0.0};
    double vy{0.0};
};

struct HorizontalAccelerationState {
    double ax{0.0};
    double ay{0.0};
};

struct HorizontalPositionTarget {
    double x{0.0};
    double y{0.0};
};

struct HorizontalVelocityTarget {
    double vx{0.0};
    double vy{0.0};
};

struct HorizontalTiltTarget {
    double roll_rad{0.0};
    double pitch_rad{0.0};
};

struct HorizontalPositionControlInput {
    HorizontalControlMode mode{HorizontalControlMode::Position};
    HorizontalPositionState position{};
    HorizontalVelocityState velocity{};
    HorizontalAccelerationState acceleration{};
    double yaw_rad{0.0};
    HorizontalPositionTarget target_position{};
    HorizontalVelocityTarget target_velocity{};
};

class IHorizontalPositionControlBackend {
public:
    virtual ~IHorizontalPositionControlBackend() = default;

    virtual void reset() = 0;

    virtual HorizontalTiltTarget run(const HorizontalPositionControlInput& input, double dt_sec) = 0;
};

}  // namespace hakoniwa::drone::control_adapter
