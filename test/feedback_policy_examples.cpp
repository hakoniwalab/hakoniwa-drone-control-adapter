#include "hakoniwa/drone/control_adapter/feedback_policy_examples.hpp"

#include <stdexcept>

namespace hakoniwa::drone::control_adapter::test {

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void run_zero_feedback_usage(IFeedbackPolicyExampleContext& context)
{
    context.reset_all();

    const RateControlSaturation saturation = context.feedback_policy().run(
        AllocationFeedbackPolicyInput{
            ControlAllocationStatus{}
        });

    require(!saturation.roll.positive && !saturation.roll.negative,
            "feedback usage: zero roll feedback must not saturate");
    require(!saturation.pitch.positive && !saturation.pitch.negative,
            "feedback usage: zero pitch feedback must not saturate");
    require(!saturation.yaw.positive && !saturation.yaw.negative,
            "feedback usage: zero yaw feedback must not saturate");
}

void run_sign_feedback_usage(IFeedbackPolicyExampleContext& context)
{
    context.reset_all();

    const RateControlSaturation saturation = context.feedback_policy().run(
        AllocationFeedbackPolicyInput{
            ControlAllocationStatus{
                false,
                0.4,
                -0.3,
                0.0,
                0.0
            }
        });

    require(saturation.roll.positive && !saturation.roll.negative,
            "feedback usage: positive roll feedback must saturate positive roll");
    require(!saturation.pitch.positive && saturation.pitch.negative,
            "feedback usage: negative pitch feedback must saturate negative pitch");
    require(!saturation.yaw.positive && !saturation.yaw.negative,
            "feedback usage: zero yaw feedback must not saturate");
}

}  // namespace

int run_feedback_policy_examples(IFeedbackPolicyExampleContext& context)
{
    run_zero_feedback_usage(context);
    run_sign_feedback_usage(context);
    return 0;
}

}  // namespace hakoniwa::drone::control_adapter::test
