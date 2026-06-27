#pragma once

#include <stdint.h>
#include "types.hpp"
#include <pypilot_algorithms/pypilot_filters.hpp>

namespace pypilot_pilots_logic {

inline int source_priority(pypilot_data_model::SensorSource source) {
    switch (source) {
    case pypilot_data_model::SensorSource::gpsd:       return 1;
    case pypilot_data_model::SensorSource::servo:      return 1;
    case pypilot_data_model::SensorSource::serial:     return 2;
    case pypilot_data_model::SensorSource::tcp:        return 3;
    case pypilot_data_model::SensorSource::signalk:    return 4;
    case pypilot_data_model::SensorSource::water_wind: return 5;
    case pypilot_data_model::SensorSource::gps_wind:   return 6;
    case pypilot_data_model::SensorSource::none:       return 7;
    }
    return 7;
}

inline bool source_can_update(pypilot_data_model::SensorSource current_source,
                              pypilot_data_model::SensorSource incoming_source,
                              bool current_source_live = true) {
    if (incoming_source == pypilot_data_model::SensorSource::none) {
        return true;
    }
    if (current_source == pypilot_data_model::SensorSource::none || !current_source_live) {
        return true;
    }
    return pypilot_pilots_logic::source_priority(incoming_source) <= pypilot_pilots_logic::source_priority(current_source);
}

inline bool source_can_update(pypilot_data_model::SensorSource current_source,
                              pypilot_data_model::SensorSource incoming_source,
                              uint64_t now_us,
                              uint64_t current_last_update_us,
                              uint64_t timeout_us = 8000000ULL) {
    bool live = current_source != pypilot_data_model::SensorSource::none &&
                !pypilot_algorithms::pypilot_source_is_stale(now_us, current_last_update_us, timeout_us);
    return source_can_update(current_source, incoming_source, live);
}

inline bool source_is_stale(uint64_t now_us,
                            uint64_t last_update_us,
                            uint64_t timeout_us = 8000000ULL) {
    return pypilot_algorithms::pypilot_source_is_stale(now_us, last_update_us, timeout_us);
}

} // namespace pypilot_pilots_logic
