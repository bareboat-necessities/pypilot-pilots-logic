#include <cassert>
#include <cmath>

#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

int main() {
    DataModel model;
    set_gain_defaults(model);

    model.ap.heading_error_deg.set(20.0f, 1);
    model.imu.heading_rate_lowpass_deg_s.set(1.0f, 1);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(0.5f, 1);
    model.ap.heading_command_rate_deg_s.set(0.25f, 1);

    PilotResult result = compute_rate_pilot(model, 10);
    float expected = (1.0f - 2.0f) * 0.075f + 0.5f * 0.075f + 0.25f * 0.6f;
    assert(result.valid);
    assert(std::fabs(result.command_norm - expected) < 0.0001f);
    return 0;
}
