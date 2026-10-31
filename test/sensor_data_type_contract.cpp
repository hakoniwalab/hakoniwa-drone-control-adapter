#include "hakoniwa/drone/control_adapter/adapter_plugin.hpp"
#include "hakoniwa/drone/control_adapter/ekf_adapter.hpp"
#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/sensor_data_type.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <type_traits>

using namespace hakoniwa::drone::control_adapter;

int main()
{
    static_assert(std::is_same_v<decltype(EkfImuInput::xacc_mps2), double>);
    static_assert(std::is_same_v<decltype(EkfImuInput::xgyro_rad_s), double>);
    static_assert(std::is_same_v<decltype(EkfHilGpsInput::lat_deg), double>);
    static_assert(std::is_same_v<decltype(EkfHilGpsInput::vn_mps), double>);
    static_assert(std::is_same_v<decltype(RateControlInput::rate), AngularRateState>);
    const bool names_are_stable =
        std::string(sensor_data_type::kExact) == "exact"
        && std::string(sensor_data_type::kImuFifoInt16) == "imu_fifo_int16"
        && std::string(sensor_data_type::kMavlinkHilGps) == "mavlink_hil_gps";
    if (!names_are_stable) {
        std::cerr << "sensor data type name contract changed\n";
        return EXIT_FAILURE;
    }
#if defined(HAKO_ENABLE_GOTO_SPEED_LIMIT) && defined(HAKO_ENABLE_SENSOR_MODEL_V2)
    constexpr const char* expected_revision = "2026-10-31.1-flight-phase-goto-speed-baro-pressure";
#elif defined(HAKO_ENABLE_GOTO_SPEED_LIMIT)
    constexpr const char* expected_revision = "2026-10-22.1-flight-phase-goto-speed";
#elif defined(HAKO_ENABLE_SENSOR_MODEL_V2)
    constexpr const char* expected_revision = "2026-10-31.1-flight-phase-baro-pressure";
#else
    constexpr const char* expected_revision = "2026-10-11.1-flight-phase";
#endif
    if (std::string(plugin::kInterfaceRevision) != expected_revision) {
        std::cerr << "sensor data type names must not change the interface revision\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
