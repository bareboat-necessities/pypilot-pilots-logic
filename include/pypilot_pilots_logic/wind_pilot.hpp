#pragma once

#include "types.hpp"

namespace pypilot_pilots_logic {

PilotResult compute_wind_pilot(DataModel& model, uint64_t now_us);
PilotResult compute_wind_pilot(DataModel& model,
                               uint64_t now_us,
                               Real& last_wind_speed_kn,
                               bool& has_last_wind_speed);

} // namespace pypilot_pilots_logic
