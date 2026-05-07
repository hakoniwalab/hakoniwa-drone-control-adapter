#pragma once

#include "hakoniwa/drone/control_adapter/altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/attitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"

namespace hakoniwa::drone::control_adapter::test {

class IUsageExampleContext {
public:
    virtual ~IUsageExampleContext() = default;

    virtual IRateControlBackend& rate_backend() = 0;
    virtual IAttitudeControlBackend& attitude_backend() = 0;
    virtual IAltitudeControlBackend& altitude_backend() = 0;
    virtual IHorizontalPositionControlBackend& horizontal_backend() = 0;

    virtual double rate_dt_sec() const = 0;
    virtual double attitude_dt_sec() const = 0;
    virtual double altitude_dt_sec() const = 0;
    virtual double horizontal_dt_sec() const = 0;

    virtual void reset_all() = 0;
};

int run_usage_examples(IUsageExampleContext& context);

}  // namespace hakoniwa::drone::control_adapter::test
