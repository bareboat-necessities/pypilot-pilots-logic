#include <pypilot_pilots_logic/basic_pilot.hpp>

#include <pypilot_algorithms/basic_pilot.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_basic_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    pypilot_algorithms::BasicPilotInput<Real> input;
    input.heading_error_deg = model.ap.heading_error_deg.value;
    input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    input.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    input.heading_command_rate_deg_s = model.ap.heading_command_rate_deg_s.value;

    pypilot_algorithms::BasicPilotGains<Real> gains;
    gains.P = model.pilots.basic.P.gain.value;
    gains.D = model.pilots.basic.D.gain.value;
    gains.DD = model.pilots.basic.DD.gain.value;
    gains.PR = model.pilots.basic.PR.gain.value;
    gains.FF = model.pilots.basic.FF.gain.value;

    pypilot_algorithms::BasicPilotOutput<Real> output =
        pypilot_algorithms::compute_basic_pilot(input,
                                                gains,
                                                pypilot_algorithms::CommandClamp::raw);

    model.pilots.basic.P.contribution.set(output.Pgain, now_us);
    model.pilots.basic.D.contribution.set(output.Dgain, now_us);
    model.pilots.basic.DD.contribution.set(output.DDgain, now_us);
    model.pilots.basic.PR.contribution.set(output.PRgain, now_us);
    model.pilots.basic.FF.contribution.set(output.FFgain, now_us);

    result.command_norm = output.command_norm;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
        model.servo.position_command_deg.set_internal_command(Real(0), now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
