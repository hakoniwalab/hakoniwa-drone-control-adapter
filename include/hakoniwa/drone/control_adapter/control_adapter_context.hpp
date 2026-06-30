#pragma once

#include <cstdint>
#include <vector>

namespace hakoniwa::drone::control_adapter {

struct ControlAdapterActuatorFault {
    std::uint32_t actuator_id{0};
    double command_scale{1.0};
};

struct ControlAdapterFaultData {
    std::vector<ControlAdapterActuatorFault> actuators;
};

class IControlAdapterContext {
public:
    virtual ~IControlAdapterContext() = default;

    virtual void update_fault_data(const ControlAdapterFaultData& fault_data) = 0;
};

}  // namespace hakoniwa::drone::control_adapter
