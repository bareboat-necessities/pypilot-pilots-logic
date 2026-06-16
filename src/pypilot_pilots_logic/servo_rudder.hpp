#pragma once

#include <stdint.h>
#include <pypilot_servo_protocol.hpp>

namespace pypilot_pilots_logic {

inline bool pypilot_servo_faulted(uint32_t flags) {
    return pypilot_servo_protocol::is_fault_flag(static_cast<uint16_t>(flags & 0xffffu));
}

inline bool pypilot_servo_allows_command(uint32_t flags) {
    return !pypilot_servo_faulted(flags);
}

template<typename Real = float>
inline bool pypilot_rudder_feedback_valid(bool angle_valid, bool angle_live, Real range_deg) {
    return angle_valid && angle_live && range_deg > Real(0);
}

template<typename Real = float>
inline Real pypilot_rudder_position_command(Real command_norm, Real range_deg) {
    return command_norm * range_deg;
}

template<typename Real = float>
inline Real pypilot_reset_command() {
    return Real(0);
}

} // namespace pypilot_pilots_logic
