#include <pypilot_pilots_logic/source_priority.hpp>

namespace pypilot_pilots_logic {

pypilot_algorithms::PypilotSource to_algorithm_source(pypilot_data_model::SensorSource source) {
    switch (source) {
    case pypilot_data_model::SensorSource::gpsd:
        return pypilot_algorithms::PypilotSource::gpsd;
    case pypilot_data_model::SensorSource::servo:
        return pypilot_algorithms::PypilotSource::servo;
    case pypilot_data_model::SensorSource::serial:
        return pypilot_algorithms::PypilotSource::serial;
    case pypilot_data_model::SensorSource::tcp:
        return pypilot_algorithms::PypilotSource::tcp;
    case pypilot_data_model::SensorSource::signalk:
        return pypilot_algorithms::PypilotSource::signalk;
    case pypilot_data_model::SensorSource::water_wind:
        return pypilot_algorithms::PypilotSource::water_wind;
    case pypilot_data_model::SensorSource::gps_wind:
        return pypilot_algorithms::PypilotSource::gps_wind;
    case pypilot_data_model::SensorSource::none:
        return pypilot_algorithms::PypilotSource::none;
    }
    return pypilot_algorithms::PypilotSource::none;
}

bool source_can_update(pypilot_data_model::SensorSource current_source,
                       pypilot_data_model::SensorSource incoming_source,
                       bool current_source_live) {
    return pypilot_algorithms::pypilot_source_can_update(to_algorithm_source(current_source),
                                                         to_algorithm_source(incoming_source),
                                                         current_source_live);
}

bool source_can_update(pypilot_data_model::SensorSource current_source,
                       pypilot_data_model::SensorSource incoming_source,
                       uint64_t now_us,
                       uint64_t current_last_update_us,
                       uint64_t timeout_us) {
    return pypilot_algorithms::pypilot_source_can_update(to_algorithm_source(current_source),
                                                         to_algorithm_source(incoming_source),
                                                         now_us,
                                                         current_last_update_us,
                                                         timeout_us);
}

bool source_is_stale(uint64_t now_us, uint64_t last_update_us, uint64_t timeout_us) {
    return pypilot_algorithms::pypilot_source_is_stale(now_us, last_update_us, timeout_us);
}

} // namespace pypilot_pilots_logic
