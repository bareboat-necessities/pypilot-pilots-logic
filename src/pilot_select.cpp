#include <pypilot_pilots_logic/pilot_select.hpp>

#include <pypilot_pilots_logic/absolute_pilot.hpp>
#include <pypilot_pilots_logic/basic_pilot.hpp>
#include <pypilot_pilots_logic/gps_pilot.hpp>
#include <pypilot_pilots_logic/logging.hpp>
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

static PilotResult log_pilot_result(PilotResult result, uint64_t now_us) {
    if (result.valid) {
        pypilot_syslib::log_if(pilots_logic_logger(), now_us,
                               pypilot_syslib::LogLevel::Debug,
                               pypilot_syslib::LogModule::PilotsLogic,
                               pypilot_syslib::LogEvent::PilotCommandComputed,
                               "pilot command computed",
                               0,
                               static_cast<float>(result.command_norm));
    }
    return result;
}

PilotResult compute_selected_pilot(DataModel& model, uint64_t now_us) {
    PilotResult result;
    switch (model.ap.pilot.value) {
    case pypilot_data_model::PilotName::absolute:
        result = fallback_to_basic_if_invalid(compute_absolute_pilot(model, now_us), model, now_us);
        break;
    case pypilot_data_model::PilotName::simple:
        result = compute_simple_pilot(model, now_us);
        break;
    case pypilot_data_model::PilotName::rate:
        result = compute_rate_pilot(model, now_us);
        break;
    case pypilot_data_model::PilotName::wind:
        result = fallback_to_basic_if_invalid(compute_wind_pilot(model, now_us), model, now_us);
        break;
    case pypilot_data_model::PilotName::gps:
        result = fallback_to_basic_if_invalid(compute_gps_pilot(model, now_us), model, now_us);
        break;
    case pypilot_data_model::PilotName::vmg:
        result = fallback_to_basic_if_invalid(compute_vmg_pilot(model, now_us), model, now_us);
        break;
    case pypilot_data_model::PilotName::basic:
    default:
        result = compute_basic_pilot(model, now_us);
        break;
    }
    return log_pilot_result(result, now_us);
}

PilotResult compute_selected_pilot(DataModel& model,
                                   uint64_t now_us,
                                   Real& last_wind_speed_kn,
                                   bool& has_last_wind_speed) {
    if (model.ap.pilot.value == pypilot_data_model::PilotName::wind) {
        return log_pilot_result(
            fallback_to_basic_if_invalid(
                compute_wind_pilot(model, now_us, last_wind_speed_kn, has_last_wind_speed),
                model,
                now_us),
            now_us);
    }
    return compute_selected_pilot(model, now_us);
}

} // namespace pypilot_pilots_logic
