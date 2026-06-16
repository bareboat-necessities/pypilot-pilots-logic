#include <cassert>
#include <cmath>

#include <pypilot_pilots_logic/gain.hpp>
#include <pypilot_algorithms/angles.hpp>
#include <pypilot_algorithms/control.hpp>

int main() {
    assert(std::fabs(pypilot_pilots_logic::compute_gain(10.0f, 0.003f) - 0.03f) < 0.00001f);
    assert(std::fabs(pypilot_algorithms::signed_sqrt_abs(-9.0f) + 3.0f) < 0.00001f);
    assert(std::fabs(pypilot_algorithms::wrap_180_deg(190.0f) + 170.0f) < 0.00001f);
    return 0;
}
