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

    PilotResult result = compute_vmg_pilot(model, 10);
    float expected = 10.0f * 0.003f + 1.0f * 0.09f + 0.5f * 0.075f;
    assert(result.valid);
    assert(std::fabs(result.command_norm - expected) < 0.0001f);
    assert(model.pilots.vmg.DD.contribution.valid);
    return 0;
}
