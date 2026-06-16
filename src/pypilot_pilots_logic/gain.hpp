#pragma once

#include "types.hpp"

namespace pypilot_pilots_logic {

Real compute_gain(Real input, Real gain);
void set_gain_defaults(DataModel& model);

} // namespace pypilot_pilots_logic
