#include "hakoniwa/drone/control_adapter/adapter_plugin.hpp"
#include "hakoniwa/drone/control_adapter/position_control_3d_backend.hpp"

#include <cstdlib>
#include <iostream>
#include <string_view>

using namespace hakoniwa::drone::control_adapter;

int main()
{
    PositionControl3DPositionInput input{};
    if (input.flight_phase != FlightPhase::Normal) {
        std::cerr << "flight_phase must default to Normal\n";
        return EXIT_FAILURE;
    }
#ifdef HAKO_ENABLE_GOTO_SPEED_LIMIT
    input.horizontal_speed_limit_m_s = 2.0;
    HorizontalPositionControlInput horizontal_input{};
    horizontal_input.horizontal_speed_limit_m_s = 3.0;
    if (!input.horizontal_speed_limit_m_s || !horizontal_input.horizontal_speed_limit_m_s) {
        std::cerr << "goto-speed fields must be available in the enabled build\n";
        return EXIT_FAILURE;
    }
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    constexpr std::string_view expected_revision = "2026-10-31.1-flight-phase-goto-speed-baro-pressure";
#else
    constexpr std::string_view expected_revision = "2026-10-22.1-flight-phase-goto-speed";
#endif
#elif defined(HAKO_ENABLE_SENSOR_MODEL_V2)
    constexpr std::string_view expected_revision = "2026-10-31.1-flight-phase-baro-pressure";
#else
    constexpr std::string_view expected_revision = "2026-10-11.1-flight-phase";
#endif
    if (std::string_view{plugin::kInterfaceRevision} != expected_revision) {
        std::cerr << "flight-phase build has the wrong interface revision\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
