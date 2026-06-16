#include <pypilot_pilots_logic/simple_pilot.hpp>
#include <pypilot_pilots_logic/gain.hpp>
#include <pypilot_pilots_logic/math_helpers.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_simple_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    Real p = compute_gain(model.ap.heading_error_deg.value, model.pilots.simple.P.gain.value);
    Real i = compute_gain(model.ap.heading_error_int_deg.value, model.pilots.simple.I.gain.value);
    Real d = compute_gain(model.imu.heading_rate_lowpass_deg_s.value, model.pilots.simple.D.gain.value);

    model.pilots.simple.P.contribution.set(p, now_us);
    model.pilots.simple.I.contribution.set(i, now_us);
    model.pilots.simple.D.contribution.set(d, now_us);

    result.command_norm = p + i + d;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
