#include <pypilot_pilots_logic/basic_pilot.hpp>
#include <pypilot_pilots_logic/gain.hpp>
#include <pypilot_pilots_logic/math_helpers.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_basic_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    Real heading_error = model.ap.heading_error_deg.value;
    Real heading_rate = model.imu.heading_rate_lowpass_deg_s.value;
    Real heading_rate_rate = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    Real command_rate = model.ap.heading_command_rate_deg_s.value;

    Real p = compute_gain(heading_error, model.pilots.basic.P.gain.value);
    Real d = compute_gain(heading_rate, model.pilots.basic.D.gain.value);
    Real dd = compute_gain(heading_rate_rate, model.pilots.basic.DD.gain.value);
    Real pr = compute_gain(signed_sqrt_abs(heading_error), model.pilots.basic.PR.gain.value);
    Real ff = compute_gain(command_rate, model.pilots.basic.FF.gain.value);

    model.pilots.basic.P.contribution.set(p, now_us);
    model.pilots.basic.D.contribution.set(d, now_us);
    model.pilots.basic.DD.contribution.set(dd, now_us);
    model.pilots.basic.PR.contribution.set(pr, now_us);
    model.pilots.basic.FF.contribution.set(ff, now_us);

    result.command_norm = clamp(p + d + dd + pr + ff, -1.0f, 1.0f);
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
