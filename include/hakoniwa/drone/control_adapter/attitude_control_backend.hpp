#pragma once

#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"

namespace hakoniwa::drone::control_adapter {

struct AttitudeQuaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct AttitudeControlInput {
    AttitudeQuaternion attitude{};
    AttitudeQuaternion target_attitude{};
    double target_yaw_rate_rad_sec{0.0};
};

class IAttitudeControlBackend {
public:
    virtual ~IAttitudeControlBackend() = default;

    virtual void reset() = 0;

    virtual AngularRateTarget run(const AttitudeControlInput& input) = 0;
};

}  // namespace hakoniwa::drone::control_adapter
