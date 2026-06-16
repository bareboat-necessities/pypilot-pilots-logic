#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>
#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    DataModel model;
    model.ap.mode.value = pypilot_data_model::AutopilotMode::compass;
    model.ap.enabled.value = true;
    model.ap.heading_command_deg.set(0.0f, 1000000);
    model.ap.heading_command_rate_deg_s.set(0.0f, 1000000);
    model.ap.heading_error_int_deg.set(0.0f, 1000000);

    model.imu.heading_lowpass_constant_0_1.value = 0.2f;
    model.imu.headingrate_lowpass_constant_0_1.value = 0.2f;
    model.imu.headingraterate_lowpass_constant_0_1.value = 0.1f;

    model.imu.heading_lowpass_deg.set(10.0f, 1000000);
    model.imu.heading_rate_lowpass_deg_s.set(0.0f, 1000000);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(0.0f, 1000000);

    model.imu.heading_deg.set(350.0f, 2000000);
    model.imu.heading_rate_deg_s.set(10.0f, 2000000);
    model.imu.heading_rate_rate_deg_s2.set(20.0f, 2000000);

    PilotsLogic logic;
    assert(logic.update_inputs(model, 2000000));

    assert(nearf(model.imu.heading_lowpass_deg.value, 6.0f));
    assert(nearf(model.imu.heading_rate_lowpass_deg_s.value, 2.0f));
    assert(nearf(model.imu.heading_rate_rate_lowpass_deg_s2.value, 2.0f));
    assert(nearf(model.ap.heading_deg.value, 6.0f));
    assert(nearf(model.ap.heading_error_deg.value, 6.0f));

    model.ap.heading_command_deg.set(10.0f, 3000000);
    model.imu.heading_deg.set(10.0f, 3000000);
    assert(logic.update_inputs(model, 3000000));
    assert(nearf(model.ap.heading_command_rate_deg_s.value, -1.0f));

    return 0;
}
