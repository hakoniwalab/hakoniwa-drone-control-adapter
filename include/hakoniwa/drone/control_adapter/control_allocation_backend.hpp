#pragma once

#include "hakoniwa/drone/control_adapter/control_adapter_context.hpp"
#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"

#include <array>
#include <cstddef>

namespace hakoniwa::drone::control_adapter {

constexpr std::size_t kMaxActuatorCount = 16;

struct CollectiveThrustCommand {
    double body_z{0.0};
};

struct ThrustTorqueCommand {
    CollectiveThrustCommand thrust{};
    double torque_x{0.0};
    double torque_y{0.0};
    double torque_z{0.0};
};

/**
 * @brief Current Euler attitude using the public NED/FRD coordinate contract.
 *
 * The local/world reference frame is NED (North-East-Down) and the aircraft
 * body frame is FRD (Forward-Right-Down). The values represent the current
 * orientation of the body FRD frame with respect to the local NED frame.
 *
 * roll_rad  (phi)   : roll angle about the FRD +X (Forward) axis [rad]
 * pitch_rad (theta) : pitch angle about the FRD +Y (Right) axis [rad]
 * yaw_rad   (psi)   : yaw angle about the FRD +Z (Down) axis [rad]
 */
struct EulerAttitudeState {
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double yaw_rad{0.0};
};

struct ActuatorAxis {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct ActuatorPosition {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct RotorActuatorGeometry {
    ActuatorPosition position{};
    ActuatorAxis axis{0.0, 0.0, -1.0};
    double thrust_coefficient{0.0};
    double moment_ratio{0.0};
};

struct ActuatorLimitRange {
    double min{0.0};
    double max{1.0};
};

struct RotorActuatorConfig {
    RotorActuatorGeometry geometry{};
    ActuatorLimitRange limit{};
    double trim{0.0};
    double linearization_point{0.0};
};

struct ControlAllocationInput {
    ThrustTorqueCommand command{};
    std::array<RotorActuatorConfig, kMaxActuatorCount> actuators{};
    std::size_t actuator_count{0};
    const IControlAdapterContext* context{nullptr};

    // Current vehicle state used by state-dependent allocation backends.
    // These fields are intentionally appended so existing aggregate callers
    // that initialize the original command/actuators/count/context prefix keep
    // their source-level field mapping.
    EulerAttitudeState attitude{};
    AngularRateState angular_rate{};
};

struct ActuatorCommandArray {
    std::array<double, kMaxActuatorCount> values{};
    std::size_t count{0};
};

struct ControlAllocationStatus {
    bool clipped{false};
    double unallocated_torque_x{0.0};
    double unallocated_torque_y{0.0};
    double unallocated_torque_z{0.0};
    double unallocated_thrust_body_z{0.0};
};

struct ControlAllocationOutput {
    ActuatorCommandArray actuator_commands{};
    ControlAllocationStatus status{};
};

class IControlAllocationBackend {
public:
    virtual ~IControlAllocationBackend() = default;

    virtual void reset() = 0;

    virtual ControlAllocationOutput run(const ControlAllocationInput& input) = 0;
};

}  // namespace hakoniwa::drone::control_adapter
