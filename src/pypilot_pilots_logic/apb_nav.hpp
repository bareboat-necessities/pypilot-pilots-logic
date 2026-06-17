#pragma once

#include "types.hpp"

namespace pypilot_pilots_logic {

inline Real pypilot_apb_xte_gain_or_default(Real gain_deg_per_nmi) {
    // PyPilot sensors.py APB default: 300 deg/nmi, meaning 30 degrees for 0.1 nmi.
    return gain_deg_per_nmi == Real(0) ? Real(300) : gain_deg_per_nmi;
}

inline Real pypilot_apb_nav_heading_command(Real track_deg,
                                            Real xte_nmi,
                                            Real gain_deg_per_nmi) {
    return track_deg + pypilot_apb_xte_gain_or_default(gain_deg_per_nmi) * xte_nmi;
}

inline bool pypilot_apb_mode_hint_blocks_nav_command(const DataModel& model) {
    // The NMEA APB parser records sender_id when it has an APB mode field.
    // PyPilot applies APB steering only for GPS-style APB mode, not magnetic/compass mode.
    // Other sources, for example SignalK target heading, may not populate sender_id, so they
    // are not blocked by the default-initialized mode_hint value.
    return model.navigation.apb.sender_id[0] != '\0' &&
           model.navigation.apb.mode_hint.value != pypilot_data_model::AutopilotMode::gps;
}

inline bool pypilot_apb_nav_command_available(const DataModel& model) {
    return model.ap.enabled.value &&
           model.ap.mode.value == pypilot_data_model::AutopilotMode::nav &&
           model.navigation.apb.source.value != pypilot_data_model::SensorSource::none &&
           model.navigation.apb.track_deg.valid &&
           !pypilot_apb_mode_hint_blocks_nav_command(model);
}

// Apply PyPilot APB/NAV command behavior:
//   heading_command = apb.track + apb.xte_gain * apb.xte
// Returns true only when heading_command_deg was changed.
bool apply_apb_nav_heading_command(DataModel& model, uint64_t now_us);

} // namespace pypilot_pilots_logic
