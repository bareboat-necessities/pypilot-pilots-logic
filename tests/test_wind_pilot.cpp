#include <cassert>
#include <cmath>

#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

int main() {
    DataModel model;
    set_gain_defaults(model);

    model.ap.enabled.value = true;
    model.ap.heading_error_deg.set(10.0f, 1);
    model.imu.heading_rate_lowpass_deg_s.set(1.0f, 1);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(0.5f, 1);
    model.wind.apparent.source.value = pypilot_data_model::SensorSource::serial;
    model.wind.apparent.direction_deg.set(-30.0f, 1);
    model.wind.apparent.speed_kn.set(12.0f, 1);
    model.pilots.wind.WG.gain.value = 0.1f;

    Real last_speed = 10.0f;
    bool has_last_speed = true;
    PilotResult result = compute_wind_pilot(model, 10, last_speed, has_last_speed);

    float expected = 10.0f * 0.003f + 1.0f * 0.1f + 0.5f * 0.05f - 2.0f * 0.1f;
    assert(result.valid);
    assert(std::fabs(result.command_norm - expected) < 0.0001f);
    assert(model.pilots.wind.WG.contribution.valid);
    return 0;
}
