#include <pypilot_pilots_logic/absolute_pilot.hpp>

#include <pypilot_algorithms/absolute_pilot.hpp>
#include <pypilot_algorithms/servo_rudder.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_absolute_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    bool rudder_live = model.rudder.angle_deg.valid && !model.rudder.angle_deg.stale(now_us, 8000000ULL);
    if (!pypilot_algorithms::pypilot_rudder_feedback_valid(model.rudder.angle_deg.valid,
                                                           rudder_live,
                                                           model.rudder.range_deg.value)) {
        model.ap.pilot.value = pypilot_data_model::PilotName::basic;
        return result;
    }

    pypilot_algorithms::AbsolutePilotInput<Real> input;
    input.heading_error_deg = model.ap.heading_error_deg.value;
    input.heading_error_int_deg = model.ap.heading_error_int_deg.value;
    input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    input.heading_command_rate_deg_s = model.ap.heading_command_rate_deg_s.value;

    pypilot_algorithms::AbsolutePilotGains<Real> gains;
    gains.P = model.pilots.absolute.P.gain.value;
    gains.I = model.pilots.absolute.I.gain.value;
    gains.D = model.pilots.absolute.D.gain.value;
    gains.FF = model.pilots.absolute.FF.gain.value;

    pypilot_algorithms::AbsolutePilotOutput<Real> output =
        pypilot_algorithms::compute_absolute_pilot(input,
                                                   gains,
                                                   pypilot_algorithms::CommandClamp::raw);

    model.pilots.absolute.P.contribution.set(output.Pgain, now_us);
    model.pilots.absolute.I.contribution.set(output.Igain, now_us);
    model.pilots.absolute.D.contribution.set(output.Dgain, now_us);
    model.pilots.absolute.FF.contribution.set(output.FFgain, now_us);

    result.position_command_deg =
        pypilot_algorithms::pypilot_rudder_position_command(output.command_norm,
                                                            model.rudder.range_deg.value);
    result.use_position_command = true;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.position_command_deg.set_internal_command(result.position_command_deg, now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
