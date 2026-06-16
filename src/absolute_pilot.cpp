#include <pypilot_pilots_logic/absolute_pilot.hpp>
#include <pypilot_pilots_logic/gain.hpp>
#include <pypilot_pilots_logic/math_helpers.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_absolute_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    if (!model.rudder.angle_deg.valid) {
        model.ap.pilot.value = pypilot_data_model::PilotName::basic;
        return result;
    }

    Real p = compute_gain(model.ap.heading_error_deg.value, model.pilots.absolute.P.gain.value);
    Real i = compute_gain(model.ap.heading_error_int_deg.value, model.pilots.absolute.I.gain.value);
    Real d = compute_gain(model.imu.heading_rate_lowpass_deg_s.value, model.pilots.absolute.D.gain.value);
    Real ff = compute_gain(model.ap.heading_command_rate_deg_s.value, model.pilots.absolute.FF.gain.value);

    model.pilots.absolute.P.contribution.set(p, now_us);
    model.pilots.absolute.I.contribution.set(i, now_us);
    model.pilots.absolute.D.contribution.set(d, now_us);
    model.pilots.absolute.FF.contribution.set(ff, now_us);

    Real command = clamp(p + i + d + ff, -1.0f, 1.0f);
    result.position_command_deg = command * model.rudder.range_deg.value;
    result.use_position_command = true;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.position_command_deg.set_internal_command(result.position_command_deg, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
