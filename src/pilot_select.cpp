#include <pypilot_pilots_logic/pilot_select.hpp>

#include <pypilot_pilots_logic/absolute_pilot.hpp>
#include <pypilot_pilots_logic/basic_pilot.hpp>
#include <pypilot_pilots_logic/gps_pilot.hpp>
#include <pypilot_pilots_logic/rate_pilot.hpp>
#include <pypilot_pilots_logic/simple_pilot.hpp>
#include <pypilot_pilots_logic/vmg_pilot.hpp>
#include <pypilot_pilots_logic/wind_pilot.hpp>

namespace pypilot_pilots_logic {

static PilotResult fallback_to_basic_if_invalid(PilotResult result,
                                                DataModel& model,
                                                uint64_t now_us) {
    return result.valid ? result : compute_basic_pilot(model, now_us);
}

PilotResult compute_selected_pilot(DataModel& model, uint64_t now_us) {
    switch (model.ap.pilot.value) {
    case pypilot_data_model::PilotName::absolute:
        return fallback_to_basic_if_invalid(compute_absolute_pilot(model, now_us), model, now_us);
    case pypilot_data_model::PilotName::simple:
        return compute_simple_pilot(model, now_us);
    case pypilot_data_model::PilotName::rate:
        return compute_rate_pilot(model, now_us);
    case pypilot_data_model::PilotName::wind:
        return fallback_to_basic_if_invalid(compute_wind_pilot(model, now_us), model, now_us);
    case pypilot_data_model::PilotName::gps:
        return fallback_to_basic_if_invalid(compute_gps_pilot(model, now_us), model, now_us);
    case pypilot_data_model::PilotName::vmg:
        return fallback_to_basic_if_invalid(compute_vmg_pilot(model, now_us), model, now_us);
    case pypilot_data_model::PilotName::basic:
    default:
        return compute_basic_pilot(model, now_us);
    }
}

PilotResult compute_selected_pilot(DataModel& model,
                                   uint64_t now_us,
                                   Real& last_wind_speed_kn,
                                   bool& has_last_wind_speed) {
    if (model.ap.pilot.value == pypilot_data_model::PilotName::wind) {
        return fallback_to_basic_if_invalid(
            compute_wind_pilot(model, now_us, last_wind_speed_kn, has_last_wind_speed),
            model,
            now_us);
    }
    return compute_selected_pilot(model, now_us);
}

} // namespace pypilot_pilots_logic
