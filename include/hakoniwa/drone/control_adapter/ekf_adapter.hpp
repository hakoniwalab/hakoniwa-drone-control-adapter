#pragma once

#include <array>
#include <cstdint>
#include <optional>

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
 * HIL_GPS plus the speed accuracy term needed by PX4 EKF.
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
    double sacc_mps{0.5};
};

struct EkfAdapterConfig {
    struct Px4EkfParams {
        std::optional<int> ekf2_gps_ctrl{};
        std::optional<int> ekf2_gps_check{};
        std::optional<double> ekf2_req_eph{};
        std::optional<double> ekf2_req_epv{};
        std::optional<double> ekf2_req_sacc{};
        std::optional<int> ekf2_req_nsats{};
        std::optional<double> ekf2_req_pdop{};
        std::optional<int> ekf2_req_fix{};
        std::optional<double> ekf2_gps_p_noise{};
        std::optional<double> ekf2_gps_v_noise{};
        std::optional<double> ekf2_gps_p_gate{};
        std::optional<double> ekf2_gps_v_gate{};
        std::optional<int> ekf2_hgt_ref{};
        std::optional<int> ekf2_baro_ctrl{};
        std::optional<double> ekf2_baro_noise{};
        std::optional<double> ekf2_baro_gate{};
        std::optional<double> ekf2_mag_decl{};
        std::optional<int> ekf2_decl_type{};
        std::optional<int> ekf2_mag_type{};
        std::optional<double> ekf2_head_noise{};
        std::optional<double> ekf2_hdg_gate{};
        std::optional<double> ekf2_mag_noise{};
        std::optional<double> ekf2_mag_gate{};
    };

    double mag_declination_deg{0.0};
    Px4EkfParams px4_params{};
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

    int active_horizontal_aiding_sources{0};
    int active_horizontal_position_aiding_sources{0};
    int active_horizontal_velocity_aiding_sources{0};
    int active_vertical_position_aiding_sources{0};
    int active_vertical_velocity_aiding_sources{0};

    bool gnss_pos_fused{false};
    bool gnss_vel_fused{false};
    bool gps_hgt_fused{false};

    double horizontal_velocity_innovation_test_ratio{0.0};
    double vertical_velocity_innovation_test_ratio{0.0};
    double horizontal_position_innovation_test_ratio{0.0};
    double vertical_position_innovation_test_ratio{0.0};
};

/**
 * Public EKF adapter contract.
 *
 * Intended implementation policy:
 * - runtime inputs are Hakoniwa-owned sensor data split by sensor type
 * - the implementation converts those into EKF samples
 * - GPS speed accuracy (`sacc`) is carried with the GPS runtime sample
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
