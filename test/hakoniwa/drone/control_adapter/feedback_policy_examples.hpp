#pragma once

#include "hakoniwa/drone/control_adapter/allocation_feedback_policy.hpp"

namespace hakoniwa::drone::control_adapter::test {

class IFeedbackPolicyExampleContext {
public:
    virtual ~IFeedbackPolicyExampleContext() = default;

    virtual IAllocationFeedbackPolicy& feedback_policy() = 0;
    virtual void reset_all() = 0;
};

int run_feedback_policy_examples(IFeedbackPolicyExampleContext& context);

}  // namespace hakoniwa::drone::control_adapter::test
