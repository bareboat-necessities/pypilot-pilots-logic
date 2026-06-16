#include <cassert>
#include <cmath>

#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

int main() {
    DataModel model;
    set_gain_defaults(model);

    model.ap.enabled.value = true;
    model.ap.heading_error_deg.set(10.0f, 1);
    model.ap.heading_command_rate_deg_s.set(0.5f, 1);
    model.imu.heading_rate_lowpass_deg_s.set(1.0f, 1);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(0.5f, 1);
    model.navigation.gps.source.value = pypilot_data_model::SensorSource::serial;

    PilotResult result = compute_gps_pilot(model, 10);
    float expected = 10.0f * 0.003f + 1.0f * 0.1f + 0.5f * 0.05f + 0.5f * 0.6f;
    assert(result.valid);
    assert(std::fabs(result.command_norm - expected) < 0.0001f);
    assert(model.pilots.gps.FF.contribution.valid);
    return 0;
}
