#include <pypilot_pilots_logic/rate_pilot.hpp>
#include <pypilot_pilots_logic/gain.hpp>
#include <pypilot_pilots_logic/math_helpers.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_rate_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    Real target_rate = clamp(model.ap.heading_error_deg.value *
                                 model.pilots.rate.turn_rate_rate_deg_s2.value,
                             -model.pilots.rate.max_turn_rate_deg_s.value,
                             model.pilots.rate.max_turn_rate_deg_s.value);
    Real rate_error = model.imu.heading_rate_lowpass_deg_s.value - target_rate;

    Real d = compute_gain(rate_error, model.pilots.rate.D.gain.value);
    Real dd = compute_gain(model.imu.heading_rate_rate_lowpass_deg_s2.value,
                           model.pilots.rate.DD.gain.value);
    Real ff = compute_gain(model.ap.heading_command_rate_deg_s.value,
                           model.pilots.rate.FF.gain.value);

    model.pilots.rate.D.contribution.set(d, now_us);
    model.pilots.rate.DD.contribution.set(dd, now_us);
    model.pilots.rate.FF.contribution.set(ff, now_us);

    result.command_norm = clamp(d + dd + ff, -1.0f, 1.0f);
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
