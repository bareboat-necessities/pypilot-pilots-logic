#include <pypilot_pilots_logic/gps_pilot.hpp>

#include <pypilot_pilots_logic/gain.hpp>
#include <pypilot_pilots_logic/math_helpers.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_gps_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;


    Real p = compute_gain(model.ap.heading_error_deg.value, model.pilots.gps.P.gain.value);
    Real d = compute_gain(model.imu.heading_rate_lowpass_deg_s.value, model.pilots.gps.D.gain.value);
    Real dd = compute_gain(model.imu.heading_rate_rate_lowpass_deg_s2.value,
                           model.pilots.gps.DD.gain.value);
    Real ff = compute_gain(model.ap.heading_command_rate_deg_s.value, model.pilots.gps.FF.gain.value);

    model.pilots.gps.P.contribution.set(p, now_us);
    model.pilots.gps.D.contribution.set(d, now_us);
    model.pilots.gps.DD.contribution.set(dd, now_us);
    model.pilots.gps.FF.contribution.set(ff, now_us);

    result.command_norm = p + d + dd + ff;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
