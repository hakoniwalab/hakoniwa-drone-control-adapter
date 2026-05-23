#pragma once

#include <array>
#include <cstdint>

namespace hakoniwa::drone::control_adapter {

struct EkfImuInput {
    std::uint64_t time_usec{0};

    double xacc_mps2{0.0};
    double yacc_mps2{0.0};
    double zacc_mps2{0.0};

    double xgyro_rad_s{0.0};
    double ygyro_rad_s{0.0};
    double zgyro_rad_s{0.0};
};

struct EkfMagInput {
    std::uint64_t time_usec{0};

    double xmag_gauss{0.0};
    double ymag_gauss{0.0};
    double zmag_gauss{0.0};
};

struct EkfBaroInput {
    std::uint64_t time_usec{0};

    double pressure_alt_m{0.0};
};

/**
 * Public Hakoniwa-facing sensor input that mirrors the fields currently sent in
 * HIL_GPS. The missing EKF speed accuracy term (`sacc`) is supplied through
 * GPS quality config rather than through this runtime message.
 */
struct EkfHilGpsInput {
    std::uint64_t time_usec{0};

    int fix_type{0};

    double lat_deg{0.0};
    double lon_deg{0.0};
    double alt_m{0.0};

    double vel_mps{0.0};
    double vn_mps{0.0};
    double ve_mps{0.0};
    double vd_mps{0.0};

    double cog_deg{0.0};
    int satellites_visible{0};
    double eph_m{0.0};
    double epv_m{0.0};
};

/**
 * Additional GPS metadata needed by the EKF but not currently carried by
 * HIL_GPS.
 */
struct EkfGpsQualityConfig {
    double sacc_mps{0.5};
};

struct EkfAdapterConfig {
    EkfGpsQualityConfig gps_quality{};
};

struct EkfEstimatedState {
    std::uint64_t time_usec{0};

    std::array<double, 4> attitude_quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> velocity_ned_mps{0.0, 0.0, 0.0};
    std::array<double, 3> position_local_ned_m{0.0, 0.0, 0.0};

    double lat_deg{0.0};
    double lon_deg{0.0};
    double alt_m_amsl{0.0};

    bool attitude_valid{false};
    bool local_position_valid{false};
    bool global_position_valid{false};
};

/**
 * Public EKF adapter contract.
 *
 * Intended implementation policy:
 * - runtime inputs are Hakoniwa-owned sensor data split by sensor type
 * - the implementation converts those into EKF samples
 * - GPS speed accuracy (`sacc`) comes from GPS config, not from HIL_GPS
 */
class IEkfAdapter {
public:
    virtual ~IEkfAdapter() = default;

    virtual void reset() = 0;

    virtual void set_config(const EkfAdapterConfig& config) = 0;
    virtual void set_in_air_status(bool in_air) = 0;
    virtual void set_vehicle_at_rest(bool at_rest) = 0;

    virtual void push_imu(const EkfImuInput& input, double dt_sec) = 0;
    virtual void push_mag(const EkfMagInput& input) = 0;
    virtual void push_baro(const EkfBaroInput& input) = 0;
    virtual void push_gps(const EkfHilGpsInput& input) = 0;

    virtual void update() = 0;

    virtual EkfEstimatedState get_estimated_state() const = 0;
};

}  // namespace hakoniwa::drone::control_adapter
