#pragma once

#include <stdint.h>

#include <ship_data_model.hpp>

namespace pypilot_data_model = ship_data_model;

namespace pypilot_pilots_logic {

using Real = float;
using DataModel = pypilot_data_model::DataModel<Real>;

struct PilotResult {
    bool valid = false;
    Real command_norm = Real(0);
    Real position_command_deg = Real(0);
    bool use_position_command = false;
};

} // namespace pypilot_pilots_logic
