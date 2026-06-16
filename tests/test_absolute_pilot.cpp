#include <cassert>
#include <cmath>

#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

int main() {
    DataModel model;
    set_gain_defaults(model);

    model.ap.enabled.value = true;
    model.rudder.angle_deg.set(0.0f, 100);
    model.rudder.range_deg.value = 30.0f;
    model.ap.heading_error_deg.set(10.0f, 100);
    model.ap.heading_error_int_deg.set(0.0f, 100);
    model.ap.heading_command_rate_deg_s.set(0.0f, 100);
    model.imu.heading_rate_lowpass_deg_s.set(0.0f, 100);

    PilotResult result = compute_absolute_pilot(model, 200);
    assert(result.valid);
    assert(result.use_position_command);
    assert(model.servo.position_command_deg.valid);
    assert(std::fabs(model.servo.position_command_deg.value - 15.0f) < 0.0001f);

    DataModel no_rudder;
    set_gain_defaults(no_rudder);
    no_rudder.ap.pilot.value = pypilot_data_model::PilotName::absolute;
    PilotResult invalid = compute_absolute_pilot(no_rudder, 300);
    assert(!invalid.valid);
    assert(no_rudder.ap.pilot.value == pypilot_data_model::PilotName::basic);
    return 0;
}
