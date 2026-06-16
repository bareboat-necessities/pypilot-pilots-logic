#include <pypilot_pilots_logic/wind_pilot.hpp>
#include <pypilot_algorithms/wind_pilot.hpp>
namespace pypilot_pilots_logic {
static PilotResult run_wind(DataModel& model, uint64_t now_us, Real delta_kn) {
    PilotResult result;
    if (model.wind.apparent.source.value == pypilot_data_model::SensorSource::none) {
        model.ap.pilot.value = pypilot_data_model::PilotName::basic;
        return result;
    }
    if (model.wind.apparent.direction_deg.valid && model.wind.apparent.direction_deg.value < 0.0f) {
        delta_kn = -delta_kn;
    }
    pypilot_algorithms::WindPilotInput<Real> input;
    input.wind_error_deg = model.ap.heading_error_deg.value;
    input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    input.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    input.wind_gust_kn = delta_kn;
    pypilot_algorithms::WindPilotGains<Real> gains;
    gains.P = model.pilots.wind.P.gain.value;
    gains.D = model.pilots.wind.D.gain.value;
    gains.DD = model.pilots.wind.DD.gain.value;
    gains.WG = model.pilots.wind.WG.gain.value;
    auto output = pypilot_algorithms::compute_wind_pilot(input, gains, pypilot_algorithms::CommandClamp::raw);
    model.pilots.wind.P.contribution.set(output.Pgain, now_us);
    model.pilots.wind.D.contribution.set(output.Dgain, now_us);
    model.pilots.wind.DD.contribution.set(output.DDgain, now_us);
    model.pilots.wind.WG.contribution.set(output.WGgain, now_us);
    result.command_norm = output.command_norm;
    result.valid = true;
    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }
    return result;
}
PilotResult compute_wind_pilot(DataModel& model, uint64_t now_us) { return run_wind(model, now_us, 0.0f); }
PilotResult compute_wind_pilot(DataModel& model, uint64_t now_us, Real& last_wind_speed_kn, bool& has_last_wind_speed) {
    Real delta_kn = 0.0f;
    if (model.wind.apparent.speed_kn.valid) {
        if (has_last_wind_speed) {
            delta_kn = model.wind.apparent.speed_kn.value - last_wind_speed_kn;
        }
        last_wind_speed_kn = model.wind.apparent.speed_kn.value;
        has_last_wind_speed = true;
    }
    return run_wind(model, now_us, delta_kn);
}
} // namespace pypilot_pilots_logic
