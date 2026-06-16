#include <pypilot_pilots_logic/math_helpers.hpp>

#include <pypilot_algorithms/angles.hpp>
#include <pypilot_algorithms/control.hpp>

namespace pypilot_pilots_logic {

Real clamp(Real value, Real min_value, Real max_value) {
    return pypilot_algorithms::clamp(value, min_value, max_value);
}

Real wrap_360(Real degrees) {
    return pypilot_algorithms::wrap_360_deg(degrees);
}

Real wrap_180(Real degrees) {
    return pypilot_algorithms::wrap_180_deg(degrees);
}

Real signed_sqrt_abs(Real value) {
    return pypilot_algorithms::signed_sqrt_abs(value);
}

} // namespace pypilot_pilots_logic
