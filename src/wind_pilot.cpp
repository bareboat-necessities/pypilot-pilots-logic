#include <pypilot_pilots_logic/wind_pilot.hpp>

#include <pypilot_algorithms/wind_pilot.hpp>

namespace pypilot_pilots_logic {

static PilotResult compute_wind_pilot_with_gust(DataModel& model,
                                                uint64_t now_us,
                                                Real wind_gust) {
    PilotResult result;

    if (model.wind.apparent.source.value == pypilot_data_model::SensorSource::none) {
        model.ap.pilot.value = pypilot_data_model::PilotName::basic;
        return result;
    }

    if (model.wind.apparent.direction_deg.valid &&
        model.wind.apparent.direction_deg.value < Real(0)) {
        wind_gust = -wind_gust;
    }

    pypilot_algorithms::WindPilotInput<Real> input;
    input.wind_error_deg = model.ap.heading_error_deg.value;
    input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    input.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    input.wind_gust_kn = wind_gust;

    pypilot_algorithms::WindPilotGains<Real> gains;
    gains.P = model.pilots.wind.P.gain.value;
    gains.D = model.pilots.wind.D.gain.value;
    gains.DD = model.pilots.wind.DD.gain.value;
    gains.WG = model.pilots.wind.WG.gain.value;

    pypilot_algorithms::WindPilotOutput<Real> output =
        pypilot_algorithms::compute_wind_pilot(input,
                                               gains,
                                               pypilot_algorithms::CommandClamp::raw);

    model.pilots.wind.P.contribution.set(output.Pgain, now_us);
    model.pilots.wind.D.contribution.set(output.Dgain, now_us);
    model.pilots.wind.DD.contribution.set(output.DDgain, now_us);
    model.pilots.wind.WG.contribution.set(output.WGgain, now_us);

    result.command_norm = output.command_norm;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
        model.servo.position_command_deg.set_internal_command(Real(0), now_us);
    }

    return result;
}

PilotResult compute_wind_pilot(DataModel& model, uint64_t now_us) {
    return compute_wind_pilot_with_gust(model, now_us, Real(0));
}

PilotResult compute_wind_pilot(DataModel& model,
                               uint64_t now_us,
                               Real& last_wind_speed_kn,
                               bool& has_last_wind_speed) {
    Real wind_gust = Real(0);

    if (model.wind.apparent.speed_kn.valid) {
        if (has_last_wind_speed) {
            wind_gust = model.wind.apparent.speed_kn.value - last_wind_speed_kn;
        }
        last_wind_speed_kn = model.wind.apparent.speed_kn.value;
        has_last_wind_speed = true;
    }

    return compute_wind_pilot_with_gust(model, now_us, wind_gust);
}

} // namespace pypilot_pilots_logic
