#include <pypilot_pilots_logic/gps_pilot.hpp>

#include <pypilot_algorithms/gps_pilot.hpp>

namespace pypilot_pilots_logic {

PilotResult compute_gps_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;

    pypilot_algorithms::GpsPilotInput<Real> input;
    input.heading_error_deg = model.ap.heading_error_deg.value;
    input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    input.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    input.heading_command_rate_deg_s = model.ap.heading_command_rate_deg_s.value;

    pypilot_algorithms::GpsPilotGains<Real> gains;
    gains.P = model.pilots.gps.P.gain.value;
    gains.D = model.pilots.gps.D.gain.value;
    gains.DD = model.pilots.gps.DD.gain.value;
    gains.PR = Real(0);
    gains.FF = model.pilots.gps.FF.gain.value;

    auto output = pypilot_algorithms::compute_gps_pilot(input, gains, pypilot_algorithms::CommandClamp::raw);

    model.pilots.gps.P.contribution.set(output.Pgain, now_us);
    model.pilots.gps.D.contribution.set(output.Dgain, now_us);
    model.pilots.gps.DD.contribution.set(output.DDgain, now_us);
    model.pilots.gps.FF.contribution.set(output.FFgain, now_us);

    result.command_norm = output.command_norm;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
        model.servo.position_command_deg.set_internal_command(Real(0), now_us);
    }

    return result;
}

} // namespace pypilot_pilots_logic
