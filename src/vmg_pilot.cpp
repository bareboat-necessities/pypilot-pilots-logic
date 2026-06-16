#include <pypilot_pilots_logic/vmg_pilot.hpp>

#include <pypilot_pilots_logic/gain.hpp>
#include <pypilot_pilots_logic/math_helpers.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_vmg_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    if (model.wind.apparent.source.value == pypilot_data_model::SensorSource::none) {
        model.ap.pilot.value = pypilot_data_model::PilotName::basic;
        return result;
    }

    Real p = compute_gain(model.ap.heading_error_deg.value, model.pilots.vmg.P.gain.value);
    Real d = compute_gain(model.imu.heading_rate_lowpass_deg_s.value, model.pilots.vmg.D.gain.value);
    Real dd = compute_gain(model.imu.heading_rate_rate_lowpass_deg_s2.value,
                           model.pilots.vmg.DD.gain.value);

    model.pilots.vmg.P.contribution.set(p, now_us);
    model.pilots.vmg.D.contribution.set(d, now_us);
    model.pilots.vmg.DD.contribution.set(dd, now_us);

    result.command_norm = p + d + dd;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
