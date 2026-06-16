#include <pypilot_pilots_logic/source_priority.hpp>

namespace pypilot_pilots_logic {

bool source_can_update(pypilot_data_model::SensorSource current_source,
                       pypilot_data_model::SensorSource incoming_source) {
    return pypilot_data_model::source_priority(incoming_source) <=
           pypilot_data_model::source_priority(current_source);
}

} // namespace pypilot_pilots_logic
