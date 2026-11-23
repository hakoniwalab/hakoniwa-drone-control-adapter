#pragma once

#include "hakoniwa/drone/control_adapter/control_adapter_context.hpp"

#include <cstddef>
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

#ifdef HAKO_ENABLE_GYRO_SAMPLE_INPUT
struct GyroSample {
    std::uint64_t time_usec{0};
    double p{0.0};
    double q{0.0};
    double r{0.0};
};
#endif

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
#ifdef HAKO_ENABLE_GYRO_SAMPLE_INPUT
    // The gyro values the host produced since the previous run, oldest first (body FRD, rad/s, simulation time in
    // us); the last one equals `rate`. Owned by the host and valid only during run(). A backend may filter them at
    // the sample rate (e.g. PX4's VehicleAngularVelocity); with no samples it uses `rate` and `angular_accel`.
    const GyroSample* gyro_samples{nullptr};
    std::size_t gyro_sample_count{0};
    // The rotors' current speeds in rotor order (rad/s, magnitude), e.g. for an RPM-driven notch filter. 0: none.
    const double* rotor_rad_per_sec{nullptr};
    std::size_t rotor_count{0};
#endif
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
