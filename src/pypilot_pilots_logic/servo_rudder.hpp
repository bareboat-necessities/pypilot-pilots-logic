#pragma once

#include <stdint.h>

#if defined(__has_include)
#  if __has_include(<pypilot_servo_protocol.hpp>)
#    include <pypilot_servo_protocol.hpp>
#    define PYPILOT_PILOTS_LOGIC_HAS_SERVO_PROTOCOL 1
#  else
#    define PYPILOT_PILOTS_LOGIC_HAS_SERVO_PROTOCOL 0
#  endif
#else
#  define PYPILOT_PILOTS_LOGIC_HAS_SERVO_PROTOCOL 0
#endif

namespace pypilot_pilots_logic {

inline bool pypilot_servo_faulted(uint32_t flags) {
#if PYPILOT_PILOTS_LOGIC_HAS_SERVO_PROTOCOL
    return pypilot_servo_protocol::is_fault_flag(static_cast<uint16_t>(flags & 0xffffu));
#else
    const uint32_t fault_mask =
        2u |     // OVERTEMP_FAULT
        4u |     // OVERCURRENT_FAULT
        16u |    // INVALID_PACKET_FLAG
        32u |    // PORT_PIN_FAULT
        64u |    // STARBOARD_PIN_FAULT
        128u |   // BADVOLTAGE_FAULT
        256u |   // MIN_RUDDER_FAULT
        512u |   // MAX_RUDDER_FAULT
        2048u;   // BAD_FUSES_FLAG
    return (flags & fault_mask) != 0;
#endif
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
