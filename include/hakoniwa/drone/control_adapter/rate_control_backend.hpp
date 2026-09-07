#pragma once

#include "hakoniwa/drone/control_adapter/control_adapter_context.hpp"

#include <cstdint>

namespace hakoniwa::drone::control_adapter {

/**
 * @brief Current angular velocity resolved in the body FRD frame [rad/s].
 *
 * The public body-frame contract is FRD (Forward-Right-Down):
 *   p : angular velocity about +X (Forward)
 *   q : angular velocity about +Y (Right)
 *   r : angular velocity about +Z (Down)
 *
 * These are body angular-velocity components. They are not the Euler-angle
 * derivatives roll_dot / pitch_dot / yaw_dot.
 */
struct AngularRateState {
    double p{0.0};
    double q{0.0};
    double r{0.0};
};

struct AngularAccelerationState {
    double p_dot{0.0};
    double q_dot{0.0};
    double r_dot{0.0};
};

struct AngularRateTarget {
    double p{0.0};
    double q{0.0};
    double r{0.0};
};

struct AxisSaturationFlags {
    bool positive{false};
    bool negative{false};
};

struct RateControlSaturation {
    AxisSaturationFlags roll{};
    AxisSaturationFlags pitch{};
    AxisSaturationFlags yaw{};
};

struct RateControlInput {
    AngularRateState rate{};
    AngularAccelerationState angular_accel{};
    AngularRateTarget target{};
    double dt_sec{0.0};
    bool landed{false};
    RateControlSaturation saturation{};
    const IControlAdapterContext* context{nullptr};
};

struct BodyTorqueCommand {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

class IRateControlBackend {
public:
    virtual ~IRateControlBackend() = default;

    virtual void reset() = 0;

    virtual BodyTorqueCommand run(const RateControlInput& input) = 0;
};

}  // namespace hakoniwa::drone::control_adapter
