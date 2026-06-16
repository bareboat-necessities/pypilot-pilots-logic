#include <cassert>
#include <cmath>

#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

int main() {
    DataModel model;
    set_gain_defaults(model);

    model.ap.heading_error_deg.set(10.0f, 1);
    model.ap.heading_error_int_deg.set(3.0f, 1);
    model.imu.heading_rate_lowpass_deg_s.set(2.0f, 1);

    PilotResult result = compute_simple_pilot(model, 10);
    float expected = 10.0f * 0.005f + 3.0f * 0.0f + 2.0f * 0.15f;
    assert(result.valid);
    assert(std::fabs(result.command_norm - expected) < 0.0001f);
    return 0;
}
