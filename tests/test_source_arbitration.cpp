#include <cassert>
#include <cmath>
#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

int main() {
    DataModel protected_truewind;
    protected_truewind.wind.truewind.source.value = pypilot_data_model::SensorSource::serial;
    protected_truewind.wind.truewind.speed_kn.set(7.0f, 1000000);
    protected_truewind.wind.truewind.direction_deg.set(30.0f, 1000000);
    protected_truewind.wind.truewind.last_update_us = 1000000;
    protected_truewind.wind.apparent.source.value = pypilot_data_model::SensorSource::serial;
    protected_truewind.wind.apparent.speed_kn.set(10.0f, 1000000);
    protected_truewind.wind.apparent.direction_deg.set(0.0f, 1000000);
    protected_truewind.wind.apparent.last_update_us = 1000000;
    protected_truewind.water.source.value = pypilot_data_model::SensorSource::serial;
    protected_truewind.water.speed_kn.set(5.0f, 1000000);
    protected_truewind.water.last_update_us = 1000000;

    PilotsLogic logic;
    assert(logic.update_inputs(protected_truewind, 1000000));
    assert(protected_truewind.wind.truewind.source.value == pypilot_data_model::SensorSource::serial);

    DataModel stale_truewind = protected_truewind;
    stale_truewind.wind.truewind.last_update_us = 1;
    stale_truewind.wind.apparent.last_update_us = 9000002;
    stale_truewind.water.last_update_us = 9000002;
    PilotsLogic stale_logic;
    assert(stale_logic.update_inputs(stale_truewind, 9000002));
    assert(stale_truewind.wind.truewind.source.value == pypilot_data_model::SensorSource::water_wind);
    assert(stale_truewind.wind.truewind.speed_kn.valid);

    DataModel fallback;
    fallback.imu.heading_lowpass_deg.set(80.0f, 1000000);
    fallback.navigation.gps.source.value = pypilot_data_model::SensorSource::serial;
    fallback.navigation.gps.track_deg.set(100.0f, 1000000);
    fallback.navigation.gps.speed_kn.set(5.0f, 1000000);
    fallback.navigation.gps.last_update_us = 1;
    fallback.ap.gps_compass_offset_deg.set(20.0f, 1000000);
    fallback.ap.preferred_mode.value = pypilot_data_model::AutopilotMode::gps;
    fallback.ap.mode.value = pypilot_data_model::AutopilotMode::gps;
    fallback.ap.heading_command_deg.set(100.0f, 1000000);

    PilotsLogic fallback_logic;
    assert(fallback_logic.update_inputs(fallback, 9000002));
    assert(fallback.navigation.gps.source.value == pypilot_data_model::SensorSource::none);
    assert((fallback.ap.available_modes_mask & pypilot_data_model::mode_mask_gps) == 0);
    assert(fallback.ap.mode.value == pypilot_data_model::AutopilotMode::compass);

    return 0;
}
