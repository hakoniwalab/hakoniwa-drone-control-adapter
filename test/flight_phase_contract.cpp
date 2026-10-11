#include "hakoniwa/drone/control_adapter/adapter_plugin.hpp"
#include "hakoniwa/drone/control_adapter/position_control_3d_backend.hpp"

#include <cstdlib>
#include <iostream>
#include <string_view>

using namespace hakoniwa::drone::control_adapter;

int main()
{
#ifdef HAKO_ENABLE_FLIGHT_PHASE
    PositionControl3DPositionInput input{};
    if (input.flight_phase != FlightPhase::Normal) {
        std::cerr << "flight_phase must default to Normal\n";
        return EXIT_FAILURE;
    }
    if (std::string_view{plugin::kInterfaceRevision} != "2026-10-11.1-flight-phase") {
        std::cerr << "flight-phase build has the wrong interface revision\n";
        return EXIT_FAILURE;
    }
#else
    if (std::string_view{plugin::kInterfaceRevision} != "2026-10-09.1") {
        std::cerr << "legacy build has the wrong interface revision\n";
        return EXIT_FAILURE;
    }
#endif
    return EXIT_SUCCESS;
}
