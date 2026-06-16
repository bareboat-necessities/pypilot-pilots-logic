#include <cassert>
#include <pypilot_pilots_logic/servo_rudder.hpp>

using namespace pypilot_pilots_logic;

int main() {
    assert(!pypilot_servo_faulted(0));
    assert(pypilot_servo_faulted(pypilot_servo_protocol::OVERTEMP_FAULT));
    assert(!pypilot_servo_allows_command(pypilot_servo_protocol::OVERTEMP_FAULT));
    assert(pypilot_rudder_feedback_valid(true, true, 30.0f));
    assert(!pypilot_rudder_feedback_valid(false, true, 30.0f));
    assert(!pypilot_rudder_feedback_valid(true, false, 30.0f));
    assert(!pypilot_rudder_feedback_valid(true, true, 0.0f));
    assert(pypilot_rudder_position_command(0.5f, 30.0f) == 15.0f);
    assert(pypilot_reset_command<float>() == 0.0f);
    return 0;
}
