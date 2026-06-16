#include <cassert>

#include <pypilot_pilots_logic/stale.hpp>

int main() {
    pypilot_data_model::Stamped<float> value;
    assert(!pypilot_pilots_logic::value_is_fresh(value, 1000, 100));

    value.set(1.0f, 1000);
    assert(pypilot_pilots_logic::value_is_fresh(value, 1050, 100));
    assert(!pypilot_pilots_logic::value_is_fresh(value, 1201, 100));
    return 0;
}
