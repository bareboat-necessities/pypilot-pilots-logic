#pragma once

#include <pypilot_algorithms/source.hpp>
#include <pypilot_data_model.hpp>

namespace pypilot_pilots_logic {

pypilot_algorithms::PypilotSource to_algorithm_source(pypilot_data_model::SensorSource source);

bool source_can_update(pypilot_data_model::SensorSource current_source,
                       pypilot_data_model::SensorSource incoming_source,
                       bool current_source_live = true);

bool source_can_update(pypilot_data_model::SensorSource current_source,
                       pypilot_data_model::SensorSource incoming_source,
                       uint64_t now_us,
                       uint64_t current_last_update_us,
                       uint64_t timeout_us = 8000000ULL);

bool source_is_stale(uint64_t now_us,
                     uint64_t last_update_us,
                     uint64_t timeout_us = 8000000ULL);

} // namespace pypilot_pilots_logic
