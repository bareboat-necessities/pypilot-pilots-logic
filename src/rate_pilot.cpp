#include <pypilot_pilots_logic/rate_pilot.hpp>

#include <pypilot_algorithms/rate_pilot.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_rate_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    pypilot_algorithms::RatePilotInput<Real> input;
    input.heading_error_deg = model.ap.heading_error_deg.value;
    input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    input.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    input.heading_command_rate_deg_s = model.ap.heading_command_rate_deg_s.value;

    pypilot_algorithms::RatePilotGains<Real> gains;
    gains.D = model.pilots.rate.D.gain.value;
    gains.DD = model.pilots.rate.DD.gain.value;
    gains.FF = model.pilots.rate.FF.gain.value;
    gains.max_turn_rate_deg_s = model.pilots.rate.max_turn_rate_deg_s.value;
    gains.turn_rate_rate_deg_s2 = model.pilots.rate.turn_rate_rate_deg_s2.value;

    pypilot_algorithms::RatePilotOutput<Real> output =
        pypilot_algorithms::compute_rate_pilot(input,
                                               gains,
                                               pypilot_algorithms::CommandClamp::raw);

    model.pilots.rate.D.contribution.set(output.Dgain, now_us);
    model.pilots.rate.DD.contribution.set(output.DDgain, now_us);
    model.pilots.rate.FF.contribution.set(output.FFgain, now_us);

    result.command_norm = output.command_norm;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
