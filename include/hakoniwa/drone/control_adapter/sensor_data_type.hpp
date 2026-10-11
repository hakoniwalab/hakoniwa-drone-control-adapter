#pragma once

/**
 * Names of the sensor data types an adapter can take its sensor inputs as.
 *
 * The interface structs (EkfImuInput, EkfHilGpsInput, RateControlInput, ...) always carry exact values
 * (double). An autopilot receives its sensors in its own types (a 16-bit IMU FIFO, MAVLink's integer
 * fields), and those types quantize: a rotation slower than one gyro step reads 0, a GNSS velocity below
 * 1 cm/s reads 0. The interface names the types only; each adapter implementation decides what a name
 * means for its autopilot (scale, rounding, decoding) and converts the exact values inside itself. An
 * adapter rejects a name it does not support. Selected in the adapter's own configuration; absent means
 * kExact (no conversion).
 */
namespace hakoniwa::drone::control_adapter::sensor_data_type {

/// The values as given (no conversion).
inline constexpr const char* kExact = "exact";

/// Gyro and accelerometer through the autopilot's 16-bit IMU FIFO (its scale and rounding are the
/// adapter's). For the rate controller's gyro and the EKF's IMU alike.
inline constexpr const char* kImuFifoInt16 = "imu_fifo_int16";

/// GNSS through MAVLink HIL_GPS: lat/lon int32 degE7, alt int32 mm, velocity int16 cm/s, eph/epv
/// uint16 cm, then decoded as the autopilot's simulator does.
inline constexpr const char* kMavlinkHilGps = "mavlink_hil_gps";

}  // namespace hakoniwa::drone::control_adapter::sensor_data_type
