#include <pypilot_pilots_logic/pilots_logic.hpp>

#include <pypilot_pilots_logic/mode_state.hpp>
#include <pypilot_pilots_logic/pilot_select.hpp>
#include <pypilot_pilots_logic/apb_nav.hpp>
#include <pypilot_pilots_logic/servo_rudder.hpp>
#include <pypilot_pilots_logic/source_arbitration.hpp>
#include <pypilot_pilots_logic/tack.hpp>

#include <pypilot_algorithms/pypilot_filters.hpp>

namespace pypilot_pilots_logic {

static PypilotMode to_logic_mode(pypilot_data_model::AutopilotMode mode) {
    switch (mode) {
    case pypilot_data_model::AutopilotMode::gps:       return PypilotMode::gps;
    case pypilot_data_model::AutopilotMode::nav:       return PypilotMode::nav;
    case pypilot_data_model::AutopilotMode::wind:      return PypilotMode::wind;
    case pypilot_data_model::AutopilotMode::true_wind: return PypilotMode::true_wind;
    case pypilot_data_model::AutopilotMode::compass:   return PypilotMode::compass;
    }
    return PypilotMode::compass;
}

static pypilot_data_model::AutopilotMode to_data_mode(PypilotMode mode) {
    switch (mode) {
    case PypilotMode::gps:       return pypilot_data_model::AutopilotMode::gps;
    case PypilotMode::nav:       return pypilot_data_model::AutopilotMode::nav;
    case PypilotMode::wind:      return pypilot_data_model::AutopilotMode::wind;
    case PypilotMode::true_wind: return pypilot_data_model::AutopilotMode::true_wind;
    case PypilotMode::compass:   return pypilot_data_model::AutopilotMode::compass;
    }
    return pypilot_data_model::AutopilotMode::compass;
}

static uint32_t to_data_mode_mask(uint32_t logic_mask) {
    uint32_t mask = 0;
    if (logic_mask & pypilot_mode_mask_compass) mask |= pypilot_data_model::mode_mask_compass;
    if (logic_mask & pypilot_mode_mask_gps) mask |= pypilot_data_model::mode_mask_gps;
    if (logic_mask & pypilot_mode_mask_nav) mask |= pypilot_data_model::mode_mask_nav;
    if (logic_mask & pypilot_mode_mask_wind) mask |= pypilot_data_model::mode_mask_wind;
    if (logic_mask & pypilot_mode_mask_true_wind) mask |= pypilot_data_model::mode_mask_true_wind;
    return mask;
}

static void invalidate_stale_sources(DataModel& model, uint64_t now_us) {
    if (model.navigation.gps.source.value != pypilot_data_model::SensorSource::none &&
        source_is_stale(now_us, model.navigation.gps.last_update_us)) {
        model.navigation.gps.source.value = pypilot_data_model::SensorSource::none;
        model.navigation.gps.speed_kn.valid = false;
        model.navigation.gps.track_deg.valid = false;
    }
    if (model.navigation.apb.source.value != pypilot_data_model::SensorSource::none &&
        source_is_stale(now_us, model.navigation.apb.last_update_us)) {
        model.navigation.apb.source.value = pypilot_data_model::SensorSource::none;
        model.navigation.apb.track_deg.valid = false;
    }
    if (model.wind.apparent.source.value != pypilot_data_model::SensorSource::none &&
        source_is_stale(now_us, model.wind.apparent.last_update_us)) {
        model.wind.apparent.source.value = pypilot_data_model::SensorSource::none;
        model.wind.apparent.speed_kn.valid = false;
        model.wind.apparent.direction_deg.valid = false;
        model.wind.apparent.filtered_speed_kn.valid = false;
        model.wind.apparent.filtered_direction_deg.valid = false;
        model.wind.apparent.filter_factor_0_1.valid = false;
    }
    if (model.wind.truewind.source.value != pypilot_data_model::SensorSource::none &&
        source_is_stale(now_us, model.wind.truewind.last_update_us)) {
        model.wind.truewind.source.value = pypilot_data_model::SensorSource::none;
