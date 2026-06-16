#pragma once
#include <math.h>
#include "types.hpp"
namespace pypilot_pilots_logic { Real clamp(Real value, Real min_value, Real max_value); Real wrap_360(Real degrees); Real wrap_180(Real degrees); Real signed_sqrt_abs(Real value); }
