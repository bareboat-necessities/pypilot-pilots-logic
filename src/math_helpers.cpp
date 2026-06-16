#include <pypilot_pilots_logic/math_helpers.hpp>

namespace pypilot_pilots_logic {

Real clamp(Real value, Real min_value, Real max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

Real wrap_360(Real degrees) {
    while (degrees >= Real(360)) {
        degrees -= Real(360);
    }
    while (degrees < Real(0)) {
        degrees += Real(360);
    }
    return degrees;
}

Real wrap_180(Real degrees) {
    degrees = wrap_360(degrees + Real(180)) - Real(180);
    if (degrees <= Real(-180)) {
        degrees += Real(360);
    }
    return degrees;
}

Real signed_sqrt_abs(Real value) {
    Real root = sqrtf(fabsf(value));
    return value < Real(0) ? -root : root;
}

} // namespace pypilot_pilots_logic
