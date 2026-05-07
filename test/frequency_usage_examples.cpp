#include "hakoniwa/drone/control_adapter/frequency_usage_examples.hpp"

#include <algorithm>
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

int expected_count(double duration_sec, double dt_sec)
{
    return static_cast<int>(std::llround(duration_sec / dt_sec));
}

}  // namespace

int run_frequency_usage_examples(IUsageExampleContext& context)
{
    context.reset_all();

    const double rate_dt = context.rate_dt_sec();
    const double attitude_dt = context.attitude_dt_sec();
    const double altitude_dt = context.altitude_dt_sec();
    const double horizontal_dt = context.horizontal_dt_sec();
    const double duration_sec = 1.0;
    const double base_dt = std::min(std::min(rate_dt, attitude_dt), std::min(altitude_dt, horizontal_dt));

    require(base_dt > 0.0, "frequency usage: base dt must be positive");
    require(attitude_dt > 0.0, "frequency usage: attitude dt must be positive");

    double rate_acc = 0.0;
    double attitude_acc = 0.0;
    double altitude_acc = 0.0;
    double horizontal_acc = 0.0;

    int rate_count = 0;
    int attitude_count = 0;
    int altitude_count = 0;
    int horizontal_count = 0;

    for (double t = 0.0; t < duration_sec - (base_dt * 0.5); t += base_dt) {
        rate_acc += base_dt;
        attitude_acc += base_dt;
        altitude_acc += base_dt;
        horizontal_acc += base_dt;

        if (attitude_acc + 1e-12 >= attitude_dt) {
            const AngularRateTarget rate_target = context.attitude_backend().run(AttitudeControlInput{
                AttitudeQuaternion{},
                AttitudeQuaternion{
                    std::cos(0.05 / 2.0),
                    0.0,
                    std::sin(0.05 / 2.0),
                    0.0
                },
                0.0
            });
            require(finite(rate_target.p) && finite(rate_target.q) && finite(rate_target.r),
                    "frequency usage: attitude output must be finite");
            attitude_acc -= attitude_dt;
            ++attitude_count;
        }

        if (altitude_acc + 1e-12 >= altitude_dt) {
            const NormalizedVerticalThrustCommand thrust = context.altitude_backend().run(
                AltitudeControlInput{
                    AltitudeControlMode::Velocity,
                    {0.0},
                    {0.0},
                    {0.0},
                    0.0,
                    {-0.2}
                },
                altitude_dt);
            require(finite(thrust.body_z), "frequency usage: altitude output must be finite");
            altitude_acc -= altitude_dt;
            ++altitude_count;
        }

        if (horizontal_acc + 1e-12 >= horizontal_dt) {
            const HorizontalTiltTarget tilt = context.horizontal_backend().run(
                HorizontalPositionControlInput{
                    HorizontalControlMode::Velocity,
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    0.0,
                    {},
                    {0.5, 0.0}
                },
                horizontal_dt);
            require(finite(tilt.roll_rad) && finite(tilt.pitch_rad),
                    "frequency usage: horizontal output must be finite");
            horizontal_acc -= horizontal_dt;
            ++horizontal_count;
        }

        if (rate_acc + 1e-12 >= rate_dt) {
            const BodyTorqueCommand torque = context.rate_backend().run(RateControlInput{
                {},
                {},
                {0.1, 0.0, 0.0},
                rate_dt,
                false,
                {}
            });
            require(finite(torque.x) && finite(torque.y) && finite(torque.z),
                    "frequency usage: rate output must be finite");
            rate_acc -= rate_dt;
            ++rate_count;
        }
    }

    require(rate_count == expected_count(duration_sec, rate_dt),
            "frequency usage: unexpected rate execution count");
    require(attitude_count == expected_count(duration_sec, attitude_dt),
            "frequency usage: unexpected attitude execution count");
    require(altitude_count == expected_count(duration_sec, altitude_dt),
            "frequency usage: unexpected altitude execution count");
    require(horizontal_count == expected_count(duration_sec, horizontal_dt),
            "frequency usage: unexpected horizontal execution count");

    std::cout << "usage.frequency.counts="
              << " rate:" << rate_count
              << " attitude:" << attitude_count
              << " altitude:" << altitude_count
              << " horizontal:" << horizontal_count
              << "\n";
    return 0;
}

}  // namespace hakoniwa::drone::control_adapter::test
