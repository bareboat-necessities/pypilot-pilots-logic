#pragma once

#include <pypilot_syslib.hpp>
#include "tack.hpp"

#include "absolute_pilot.hpp"
#include "basic_pilot.hpp"
#include "gain.hpp"
#include "gps_pilot.hpp"
#include "pilot_select.hpp"
#include "rate_pilot.hpp"
#include "simple_pilot.hpp"
#include "source_arbitration.hpp"
#include "stale.hpp"
#include "types.hpp"
#include "vmg_pilot.hpp"
#include "wind_pilot.hpp"

namespace pypilot_pilots_logic {

class PilotsLogic {
public:
    PilotsLogic();

    void set_logger(pypilot_syslib::Logger* logger);
    pypilot_syslib::Logger* logger() const;

    void begin_tack(PypilotTackDirection direction);
    void cancel_tack();
    PypilotTackState tack_state() const;
    PypilotTackDirection tack_direction() const;
    void set_tack_config(Real delay_s, Real angle_deg, Real rate_deg_s, Real threshold_percent);

    bool update_inputs(DataModel& model, uint64_t now_us);
    bool compute_command(DataModel& model, uint64_t now_us);
    bool step(DataModel& model, uint64_t now_us);

    const char* last_error() const;

private:
    const char* last_error_;
    Real last_wind_speed_kn_;
    bool has_last_wind_speed_;
    Real last_heading_command_deg_;
    bool has_last_heading_command_;
    uint64_t last_heading_error_int_us_;
    Real gps_speed_filter_kn_;
    Real stored_preferred_command_deg_;
    uint64_t stored_preferred_command_us_;
    bool has_stored_preferred_command_;
    PypilotTackState tack_state_;
    PypilotTackDirection tack_direction_;
    PypilotTackDirection tack_current_direction_;
    Real tack_delay_s_;
    Real tack_angle_deg_;
    Real tack_rate_deg_s_;
    Real tack_threshold_percent_;
    uint64_t tack_state_start_us_;
    pypilot_syslib::Logger* logger_;
};

} // namespace pypilot_pilots_logic
