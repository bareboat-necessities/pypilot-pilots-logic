#include <pypilot_pilots_logic/simple_pilot.hpp>

#include <pypilot_algorithms/simple_pilot.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_simple_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    pypilot_algorithms::SimplePilotInput<Real> input;
    input.heading_error_deg = model.ap.heading_error_deg.value;
    input.heading_error_int_deg = model.ap.heading_error_int_deg.value;
    input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;

    pypilot_algorithms::SimplePilotGains<Real> gains;
    gains.P = model.pilots.simple.P.gain.value;
    gains.I = model.pilots.simple.I.gain.value;
    gains.D = model.pilots.simple.D.gain.value;

    pypilot_algorithms::SimplePilotOutput<Real> output =
        pypilot_algorithms::compute_simple_pilot(input,
                                                 gains,
                                                 pypilot_algorithms::CommandClamp::raw);

    model.pilots.simple.P.contribution.set(output.Pgain, now_us);
    model.pilots.simple.I.contribution.set(output.Igain, now_us);
    model.pilots.simple.D.contribution.set(output.Dgain, now_us);

    result.command_norm = output.command_norm;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
