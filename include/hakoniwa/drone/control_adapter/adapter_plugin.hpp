#pragma once

#include <cstddef>
#include <cstdint>

#include "hakoniwa/drone/control_adapter/allocation_feedback_policy.hpp"
#include "hakoniwa/drone/control_adapter/altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/attitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/control_allocation_backend.hpp"
#include "hakoniwa/drone/control_adapter/ekf_adapter.hpp"
#include "hakoniwa/drone/control_adapter/horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/position_control_3d_backend.hpp"
#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"

/**
 * Control adapter plugin: an adapter built as a shared library (.so/.dylib/.dll)
 * that a host loads at run time instead of linking it.
 *
 * The library exports one C function, HAKO_CONTROL_ADAPTER_PLUGIN_ENTRY_V1, that
 * returns the plugin table below. Backends cross the boundary only as the
 * interfaces of this repository; the host and the plugin must be built with the
 * same compiler, C++ standard library and interface revision (the host checks
 * abi_version and interface_revision when it loads the table).
 *
 * Rules for every table function:
 * - no exception crosses the boundary: a failure returns false / nullptr and
 *   writes a message into error (error_size bytes, always NUL-terminated);
 * - an object the plugin creates is destroyed by the plugin (destroy_backend,
 *   destroy_context), never with the host's delete.
 */
namespace hakoniwa::drone::control_adapter::plugin {

/// Version of ControlAdapterPluginV1. A host rejects a table of another version.
inline constexpr std::uint32_t kPluginAbiVersion = 1;

/// Layout revision of the interface headers. Bumped with every change that
/// alters an interface or a struct the interfaces pass (a host and a plugin
/// built from different revisions must not exchange objects).
inline constexpr const char* kInterfaceRevision = "2026-10-09.1";

enum class BackendKind : std::uint32_t {
    RateControl = 1,               // IRateControlBackend
    AttitudeControl = 2,           // IAttitudeControlBackend
    AltitudeControl = 3,           // IAltitudeControlBackend
    HorizontalPositionControl = 4, // IHorizontalPositionControlBackend
    PositionControl3D = 5,         // IPositionControl3DBackend
    ControlAllocation = 6,         // IControlAllocationBackend
    AllocationFeedbackPolicy = 7,  // IAllocationFeedbackPolicy
    Ekf = 8,                       // IEkfAdapter
};

struct ControlAdapterPluginV1 {
    /// kPluginAbiVersion of the headers the plugin was built with.
    std::uint32_t abi_version;
    /// kInterfaceRevision of the headers the plugin was built with.
    const char* interface_revision;
    /// Short adapter name, e.g. "ardupilot" (logs and contract reports).
    const char* adapter_id;

    /// Reads the adapter's own configuration file and returns its state; every
    /// backend of one vehicle is created from the same context.
    void* (*create_context)(const char* config_path, char* error, std::size_t error_size);
    void (*destroy_context)(void* context);

    /// Applies the host's Hakoniwa controller parameters (the PID_* text, with
    /// tuning overrides) to the adapter's own parameters. Called before the
    /// backends are created.
    bool (*apply_hakoniwa_params)(
        void* context, const char* param_text, char* error, std::size_t error_size);

    /// Creates the backend of one kind, returned as a pointer to that kind's
    /// interface (static_cast to the interface, then to void*). For Ekf,
    /// argument points to the EkfAdapterConfig; otherwise it is nullptr.
    /// A kind the adapter does not provide returns nullptr with an empty error.
    void* (*create_backend)(
        void* context, BackendKind kind, const void* argument, char* error, std::size_t error_size);
    void (*destroy_backend)(BackendKind kind, void* backend);

    /// The adapter's capability declaration (JSON, the contract checker's
    /// capability format). Valid while the context lives.
    const char* (*capabilities_json)(void* context);
};

}  // namespace hakoniwa::drone::control_adapter::plugin

/// The symbol the host looks up; it returns the plugin table (static storage).
#define HAKO_CONTROL_ADAPTER_PLUGIN_ENTRY_V1 "hako_control_adapter_plugin_v1"

extern "C" {
using HakoControlAdapterPluginEntryV1 =
    const hakoniwa::drone::control_adapter::plugin::ControlAdapterPluginV1* (*)();
}
