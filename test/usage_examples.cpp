#include "hakoniwa/drone/control_adapter/usage_examples.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace hakoniwa::drone::control_adapter::test {

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool finite(double value)
{
    return std::isfinite(value);
}

void run_attitude_rate_pipeline_usage(IUsageExampleContext& context)
{
    context.reset_all();

    const AngularRateTarget rate_target = context.attitude_backend().run(AttitudeControlInput{
        AttitudeQuaternion{},
        AttitudeQuaternion{
            std::cos(0.1 / 2.0),
            0.0,
            std::sin(0.1 / 2.0),
            0.0
        },
        0.0
    });

    require(finite(rate_target.p) && finite(rate_target.q) && finite(rate_target.r),
            "attitude usage: rate target must be finite");

    const BodyTorqueCommand torque = context.rate_backend().run(RateControlInput{
        {},
        {},
        rate_target,
        context.rate_dt_sec(),
        false,
        {}
    });

    require(finite(torque.x) && finite(torque.y) && finite(torque.z),
            "attitude usage: torque output must be finite");

    std::cout << "usage.attitude_rate.torque=[" << torque.x << ", " << torque.y << ", " << torque.z << "]\n";
}

void run_altitude_position_usage(IUsageExampleContext& context)
{
    context.reset_all();

    const NormalizedVerticalThrustCommand thrust = context.altitude_backend().run(
        AltitudeControlInput{
            AltitudeControlMode::Position,
            {0.0},
            {0.0},
            {0.0},
            -0.5,
            {}
        },
        context.altitude_dt_sec());

    require(finite(thrust.body_z), "altitude position usage: thrust must be finite");
    std::cout << "usage.altitude_position.body_z=" << thrust.body_z << "\n";
}

void run_altitude_velocity_usage(IUsageExampleContext& context)
{
    context.reset_all();

    const NormalizedVerticalThrustCommand thrust = context.altitude_backend().run(
        AltitudeControlInput{
            AltitudeControlMode::Velocity,
            {0.0},
            {0.0},
            {0.0},
            0.0,
            {-0.3}
        },
        context.altitude_dt_sec());

    require(finite(thrust.body_z), "altitude velocity usage: thrust must be finite");
    std::cout << "usage.altitude_velocity.body_z=" << thrust.body_z << "\n";
}

void run_horizontal_velocity_usage(IUsageExampleContext& context)
{
    context.reset_all();

    const HorizontalTiltTarget tilt = context.horizontal_backend().run(
        HorizontalPositionControlInput{
            HorizontalControlMode::Velocity,
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            0.0,
            {},
            {1.0, 0.0}
        },
        context.horizontal_dt_sec());

    require(finite(tilt.roll_rad) && finite(tilt.pitch_rad),
            "horizontal velocity usage: tilt output must be finite");
    std::cout << "usage.horizontal_velocity.tilt=[" << tilt.roll_rad << ", " << tilt.pitch_rad << "]\n";
}

void run_horizontal_position_usage(IUsageExampleContext& context)
{
    context.reset_all();

    const HorizontalTiltTarget tilt = context.horizontal_backend().run(
        HorizontalPositionControlInput{
            HorizontalControlMode::Position,
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            0.0,
            {1.0, 0.5},
            {}
        },
        context.horizontal_dt_sec());

    require(finite(tilt.roll_rad) && finite(tilt.pitch_rad),
            "horizontal position usage: tilt output must be finite");
    std::cout << "usage.horizontal_position.tilt=[" << tilt.roll_rad << ", " << tilt.pitch_rad << "]\n";
}

}  // namespace

int run_usage_examples(IUsageExampleContext& context)
{
    run_attitude_rate_pipeline_usage(context);
    run_altitude_position_usage(context);
    run_altitude_velocity_usage(context);
    run_horizontal_velocity_usage(context);
    run_horizontal_position_usage(context);
    return 0;
}

}  // namespace hakoniwa::drone::control_adapter::test
