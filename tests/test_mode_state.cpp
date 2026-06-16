#include <cassert>
#include <cmath>
#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    DataModel gps_model;
    gps_model.imu.heading_lowpass_deg.set(80.0f, 1000000);
    gps_model.navigation.gps.source.value = pypilot_data_model::SensorSource::serial;
    gps_model.navigation.gps.track_deg.set(100.0f, 1000000);
    gps_model.navigation.gps.speed_kn.set(5.0f, 1000000);
    gps_model.navigation.gps.last_update_us = 1000000;
    gps_model.ap.gps_compass_offset_deg.set(20.0f, 1000000);
    gps_model.ap.preferred_mode.value = pypilot_data_model::AutopilotMode::nav;
    gps_model.ap.mode.value = pypilot_data_model::AutopilotMode::nav;
    gps_model.ap.heading_command_deg.set(100.0f, 1000000);

    PilotsLogic gps_logic;
    assert(gps_logic.update_inputs(gps_model, 1000000));
    assert((gps_model.ap.available_modes_mask & pypilot_data_model::mode_mask_compass) != 0);
    assert((gps_model.ap.available_modes_mask & pypilot_data_model::mode_mask_gps) != 0);
    assert((gps_model.ap.available_modes_mask & pypilot_data_model::mode_mask_nav) == 0);
    assert(gps_model.ap.mode.value == pypilot_data_model::AutopilotMode::gps);
    assert(nearf(gps_model.ap.heading_deg.value, 100.0f));

    DataModel nav_model;
    nav_model.imu.heading_lowpass_deg.set(80.0f, 1000000);
    nav_model.navigation.gps.source.value = pypilot_data_model::SensorSource::serial;
    nav_model.navigation.gps.track_deg.set(100.0f, 1000000);
    nav_model.navigation.gps.speed_kn.set(5.0f, 1000000);
    nav_model.navigation.gps.last_update_us = 1000000;
    nav_model.navigation.apb.source.value = pypilot_data_model::SensorSource::serial;
    nav_model.navigation.apb.track_deg.set(110.0f, 1000000);
    nav_model.navigation.apb.last_update_us = 1000000;
    nav_model.ap.gps_and_nav_modes.value = true;
    nav_model.ap.gps_compass_offset_deg.set(20.0f, 1000000);
    nav_model.ap.preferred_mode.value = pypilot_data_model::AutopilotMode::nav;
    nav_model.ap.mode.value = pypilot_data_model::AutopilotMode::nav;
    nav_model.ap.heading_command_deg.set(100.0f, 1000000);

    PilotsLogic nav_logic;
    assert(nav_logic.update_inputs(nav_model, 1000000));
    assert((nav_model.ap.available_modes_mask & pypilot_data_model::mode_mask_gps) != 0);
    assert((nav_model.ap.available_modes_mask & pypilot_data_model::mode_mask_nav) != 0);
    assert(nav_model.ap.mode.value == pypilot_data_model::AutopilotMode::nav);
    assert(nearf(nav_model.ap.heading_deg.value, 100.0f));

    DataModel wind_model;
    wind_model.imu.heading_lowpass_deg.set(80.0f, 1000000);
    wind_model.wind.apparent.source.value = pypilot_data_model::SensorSource::serial;
    wind_model.wind.apparent.filtered_direction_deg.set(-20.0f, 1000000);
    wind_model.wind.apparent.last_update_us = 1000000;
    wind_model.ap.wind_compass_offset_deg.set(60.0f, 1000000);
    wind_model.ap.preferred_mode.value = pypilot_data_model::AutopilotMode::true_wind;
    wind_model.ap.mode.value = pypilot_data_model::AutopilotMode::true_wind;
    wind_model.ap.heading_command_deg.set(-20.0f, 1000000);

    PilotsLogic wind_logic;
    assert(wind_logic.update_inputs(wind_model, 1000000));
    assert((wind_model.ap.available_modes_mask & pypilot_data_model::mode_mask_wind) != 0);
    assert((wind_model.ap.available_modes_mask & pypilot_data_model::mode_mask_true_wind) == 0);
    assert(wind_model.ap.mode.value == pypilot_data_model::AutopilotMode::wind);
    assert(nearf(wind_model.ap.heading_deg.value, -20.0f));

    return 0;
}
