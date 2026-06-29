#include <cassert>
#include <pypilot_servo_protocol.hpp>

int main() {
    assert(!pypilot_servo_protocol::is_fault_flag(0));
    assert(pypilot_servo_protocol::is_fault_flag(pypilot_servo_protocol::OVERTEMP_FAULT));
    assert(!pypilot_servo_protocol::is_fault_flag(pypilot_servo_protocol::ENGAGED_FLAG));

    const float command = pypilot_servo_protocol::decode_command_value(
        pypilot_servo_protocol::encode_command(0.5f));
    assert(command > 0.49f && command < 0.51f);

    return 0;
}
