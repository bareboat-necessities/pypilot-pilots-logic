#include <cassert>
#include <cmath>
#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    DataModel model;
    model.ap.enabled.value = true;
    model.ap.mode.value = pypilot_data_model::AutopilotMode::compass;
    model.ap.preferred_mode.value = pypilot_data_model::AutopilotMode::compass;
    model.ap.heading_command_deg.set(100.0f, 1000000);
    model.ap.heading_deg.set(80.0f, 1000000);
    model.imu.heading_lowpass_deg.set(80.0f, 1000000);
    model.imu.heading_rate_lowpass_deg_s.set(0.0f, 1000000);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(0.0f, 1000000);

    PilotsLogic logic;
    logic.set_tack_config(0.0f, 100.0f, 15.0f, 50.0f);
    logic.begin_tack(PypilotTackDirection::port);

    assert(logic.compute_command(model, 1000000));
    assert(model.servo.command_norm.valid);
    assert(nearf(model.servo.command_norm.value, 1.0f));
    assert(logic.tack_state() == PypilotTackState::tacking);

    model.ap.heading_deg.set(40.0f, 2000000);
    model.imu.heading_lowpass_deg.set(40.0f, 2000000);
    assert(logic.compute_command(model, 2000000));
    assert(logic.tack_state() == PypilotTackState::none);
    assert(nearf(model.ap.heading_command_deg.value, 0.0f));

    logic.begin_tack(PypilotTackDirection::starboard);
    model.ap.heading_command_deg.set(-100.0f, 3000000);
    model.ap.heading_deg.set(-80.0f, 3000000);
    model.imu.heading_lowpass_deg.set(-80.0f, 3000000);
    assert(logic.compute_command(model, 3000000));
    assert(nearf(model.servo.command_norm.value, -1.0f));

    model.ap.enabled.value = false;
    assert(logic.compute_command(model, 4000000));
    assert(logic.tack_state() == PypilotTackState::none);

    return 0;
}
