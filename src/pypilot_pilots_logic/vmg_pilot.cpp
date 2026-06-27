#include <pypilot_pilots_logic/vmg_pilot.hpp>

#include <pypilot_algorithms/vmg_pilot.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_vmg_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    if (model.wind.apparent.source.value == pypilot_data_model::SensorSource::none) {
        model.ap.pilot.value = pypilot_data_model::PilotName::basic;
        return result;
    }

    pypilot_algorithms::VmgPilotInput<Real> input;
    input.heading_error_deg = model.ap.heading_error_deg.value;
    input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    input.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;

    pypilot_algorithms::VmgPilotGains<Real> gains;
    gains.P = model.pilots.vmg.P.gain.value;
    gains.D = model.pilots.vmg.D.gain.value;
    gains.DD = model.pilots.vmg.DD.gain.value;

    auto output = pypilot_algorithms::compute_vmg_pilot(input, gains, pypilot_algorithms::CommandClamp::raw);

    model.pilots.vmg.P.contribution.set(output.Pgain, now_us);
    model.pilots.vmg.D.contribution.set(output.Dgain, now_us);
    model.pilots.vmg.DD.contribution.set(output.DDgain, now_us);

    result.command_norm = output.command_norm;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
        model.servo.position_command_deg.set_internal_command(Real(0), now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
