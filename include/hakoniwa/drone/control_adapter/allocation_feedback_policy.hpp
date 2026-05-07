#pragma once

#include "hakoniwa/drone/control_adapter/control_allocation_backend.hpp"
#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"

namespace hakoniwa::drone::control_adapter {

struct AllocationFeedbackPolicyInput {
    ControlAllocationStatus allocation_status{};
};

class IAllocationFeedbackPolicy {
public:
    virtual ~IAllocationFeedbackPolicy() = default;

    virtual void reset() = 0;

    virtual RateControlSaturation run(const AllocationFeedbackPolicyInput& input) = 0;
};

}  // namespace hakoniwa::drone::control_adapter
