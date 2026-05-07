#pragma once

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
