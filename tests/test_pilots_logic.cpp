#include <cassert>
#include <cmath>
#include <cstring>

#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

static void populate_basic_inputs(DataModel& model) {
    model.ap.pilot.value = pypilot_data_model::PilotName::basic;
    model.ap.heading_command_deg.set(100.0f, 100);
    model.ap.heading_command_rate_deg_s.set(0.0f, 100);
    model.imu.heading_lowpass_deg.set(90.0f, 100);
    model.imu.heading_rate_lowpass_deg_s.set(0.0f, 100);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(0.0f, 100);
}

int main() {
    DataModel model;

    model.ap.enabled.value = true;
    populate_basic_inputs(model);

    PilotsLogic logic;
    assert(logic.step(model, 200));
    assert(model.ap.heading_error_deg.valid);
    assert(std::fabs(model.ap.heading_error_deg.value + 10.0f) < 0.0001f);
    assert(model.servo.command_norm.valid);

    DataModel disabled;
    populate_basic_inputs(disabled);
    disabled.ap.enabled.value = false;
    disabled.servo.command_norm.set_internal_command(0.75f, 100);
    disabled.servo.position_command_deg.set_internal_command(12.0f, 100);
    assert(logic.compute_command(disabled, 300));
    assert(disabled.servo.command_norm.value == 0.0f);
    assert(disabled.servo.position_command_deg.value == 0.0f);

    DataModel faulted;
    populate_basic_inputs(faulted);
    faulted.ap.enabled.value = true;
    faulted.servo.flags.value = 2u;
    faulted.servo.command_norm.set_internal_command(0.5f, 100);
    faulted.servo.position_command_deg.set_internal_command(10.0f, 100);
    assert(!logic.compute_command(faulted, 400));
    assert(faulted.servo.command_norm.value == 0.0f);
    assert(faulted.servo.position_command_deg.value == 0.0f);
    assert(std::strcmp(logic.last_error(), "servo fault") == 0);

    return 0;
}
