#include "hakoniwa/drone/control_adapter/control_allocation_backend.hpp"

#include <array>
#include <cassert>

int main()
{
    using namespace hakoniwa::drone::control_adapter;

    ControlAllocationInput input{};
    assert(input.attitude.roll_rad == 0.0);
    assert(input.attitude.pitch_rad == 0.0);
    assert(input.attitude.yaw_rad == 0.0);
    assert(input.angular_rate.p == 0.0);
    assert(input.angular_rate.q == 0.0);
    assert(input.angular_rate.r == 0.0);

    input.attitude = EulerAttitudeState{0.1, -0.2, 0.3};
    input.angular_rate = AngularRateState{1.0, -2.0, 3.0};

    assert(input.attitude.roll_rad == 0.1);
    assert(input.attitude.pitch_rad == -0.2);
    assert(input.attitude.yaw_rad == 0.3);
    assert(input.angular_rate.p == 1.0);
    assert(input.angular_rate.q == -2.0);
    assert(input.angular_rate.r == 3.0);

    // Preserve source compatibility for callers using the original aggregate
    // prefix: command, actuators, actuator_count, context.
    ControlAllocationInput legacy_prefix{
        ThrustTorqueCommand{},
        std::array<RotorActuatorConfig, kMaxActuatorCount>{},
        0,
        nullptr,
    };
    assert(legacy_prefix.attitude.roll_rad == 0.0);
    assert(legacy_prefix.angular_rate.p == 0.0);

    return 0;
}
