#pragma once

#include "types.hpp"

namespace pypilot_pilots_logic {

PilotResult compute_gps_pilot(DataModel& model, uint64_t now_us);

} // namespace pypilot_pilots_logic
