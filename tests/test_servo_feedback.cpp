#include <cassert>
#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

int main() {
    DataModel disabled;
    disabled.ap.enabled.value = false;
    disabled.servo.command_norm.set_internal_command(0.7f, 100);
    disabled.servo.position_command_deg.set_internal_command(20.0f, 100);
    PilotsLogic disabled_logic;
    assert(disabled_logic.compute_command(disabled, 200));
    assert(disabled.servo.command_norm.value == 0.0f);
    assert(disabled.servo.position_command_deg.value == 0.0f);

    DataModel faulted;
    faulted.ap.enabled.value = true;
    faulted.servo.flags.value = pypilot_data_model::servo_overtemp_fault;
    PilotsLogic faulted_logic;
    assert(!faulted_logic.compute_command(faulted, 300));
    assert(faulted.servo.command_norm.value == 0.0f);
    assert(faulted.servo.position_command_deg.value == 0.0f);

    return 0;
}
