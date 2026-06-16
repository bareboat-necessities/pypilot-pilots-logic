#include <cassert>
#include <cmath>

#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

int main() {
    DataModel model;
    set_gain_defaults(model);

    model.ap.enabled.value = true;
    model.ap.heading_error_deg.set(10.0f, 100);
    model.ap.heading_command_rate_deg_s.set(0.5f, 100);
    model.imu.heading_rate_lowpass_deg_s.set(1.0f, 100);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(2.0f, 100);

    PilotResult result = compute_basic_pilot(model, 200);
    assert(result.valid);
    assert(model.servo.command_norm.valid);

    float expected = 10.0f * 0.003f + 1.0f * 0.09f + 2.0f * 0.075f +
                     std::sqrt(10.0f) * 0.005f + 0.5f * 0.6f;
    assert(std::fabs(model.servo.command_norm.value - expected) < 0.0002f);
    assert(model.pilots.basic.PR.contribution.valid);

    model.ap.heading_error_deg.set(1000.0f, 300);
    model.imu.heading_rate_lowpass_deg_s.set(0.0f, 300);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(0.0f, 300);
    model.ap.heading_command_rate_deg_s.set(0.0f, 300);
    PilotResult unclamped = compute_basic_pilot(model, 400);
    assert(unclamped.command_norm > 1.0f);
    return 0;
}
