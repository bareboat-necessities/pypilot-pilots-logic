#include <pypilot_pilots_logic/wind_pilot.hpp>

#include <pypilot_pilots_logic/gain.hpp>
#include <pypilot_pilots_logic/math_helpers.hpp>

namespace pypilot_pilots_logic {

static PilotResult compute_wind_pilot_with_gust(DataModel& model,
                                                uint64_t now_us,
                                                Real wind_gust) {
    PilotResult result;

    if (model.wind.apparent.source.value == pypilot_data_model::SensorSource::none) {
        model.ap.pilot.value = pypilot_data_model::PilotName::basic;
        return result;
    }

    if (model.wind.apparent.direction_deg.valid && model.wind.apparent.direction_deg.value < 0.0f) {
        wind_gust = -wind_gust;
    }

    Real p = compute_gain(model.ap.heading_error_deg.value, model.pilots.wind.P.gain.value);
    Real d = compute_gain(model.imu.heading_rate_lowpass_deg_s.value, model.pilots.wind.D.gain.value);
    Real dd = compute_gain(model.imu.heading_rate_rate_lowpass_deg_s2.value,
                           model.pilots.wind.DD.gain.value);
    Real wg = compute_gain(wind_gust, model.pilots.wind.WG.gain.value);

    model.pilots.wind.P.contribution.set(p, now_us);
    model.pilots.wind.D.contribution.set(d, now_us);
    model.pilots.wind.DD.contribution.set(dd, now_us);
    model.pilots.wind.WG.contribution.set(wg, now_us);

    result.command_norm = p + d + dd + wg;
    result.valid = true;

    if (model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(result.command_norm, now_us);
    }

    return result;
}

PilotResult compute_wind_pilot(DataModel& model, uint64_t now_us) {
    return compute_wind_pilot_with_gust(model, now_us, 0.0f);
}

PilotResult compute_wind_pilot(DataModel& model,
                               uint64_t now_us,
                               Real& last_wind_speed_kn,
                               bool& has_last_wind_speed) {
    Real gust = 0.0f;

    if (model.wind.apparent.speed_kn.valid) {
        if (has_last_wind_speed) {
            gust = model.wind.apparent.speed_kn.value - last_wind_speed_kn;
        }
        last_wind_speed_kn = model.wind.apparent.speed_kn.value;
        has_last_wind_speed = true;
    }

    return compute_wind_pilot_with_gust(model, now_us, gust);
}

} // namespace pypilot_pilots_logic
