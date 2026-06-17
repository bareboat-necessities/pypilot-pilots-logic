#include <cassert>
#include <cmath>

#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;
using pypilot_data_model::AutopilotMode;
using pypilot_data_model::SensorSource;

static bool nearf(float a, float b) {
    return std::fabs(a - b) < 0.0001f;
}

int main() {
    DataModel model;

    // Default APB gain is PyPilot's 300 deg/nmi.
    assert(nearf(pypilot_apb_xte_gain_or_default(0.0f), 300.0f));
    assert(nearf(pypilot_apb_nav_heading_command(90.0f, 0.1f, 0.0f), 120.0f));

    model.ap.enabled.value = true;
    model.ap.mode.value = AutopilotMode::nav;
    model.navigation.apb.source.value = SensorSource::serial;
    model.navigation.apb.track_deg.set(90.0f, 100);
    model.navigation.apb.xte_nmi.set(0.1f, 100);

    assert(pypilot_apb_nav_command_available(model));
    assert(apply_apb_nav_heading_command(model, 200));
    assert(model.ap.heading_command_deg.valid);
    assert(nearf(model.ap.heading_command_deg.value, 120.0f));

    const uint64_t previous_update = model.ap.heading_command_deg.last_update_us;
    assert(!apply_apb_nav_heading_command(model, 300));
    assert(model.ap.heading_command_deg.last_update_us == previous_update);

    model.navigation.apb.xte_gain_deg_per_nmi.value = 100.0f;
    model.navigation.apb.xte_nmi.set(-0.2f, 400);
    assert(apply_apb_nav_heading_command(model, 500));
    assert(nearf(model.ap.heading_command_deg.value, 70.0f));

    model.ap.enabled.value = false;
    model.navigation.apb.xte_nmi.set(0.5f, 600);
    assert(!apply_apb_nav_heading_command(model, 700));
    assert(nearf(model.ap.heading_command_deg.value, 70.0f));

    model.ap.enabled.value = true;
    model.ap.mode.value = AutopilotMode::gps;
    assert(!apply_apb_nav_heading_command(model, 800));
    assert(nearf(model.ap.heading_command_deg.value, 70.0f));

    model.ap.mode.value = AutopilotMode::nav;
    model.navigation.apb.source.value = SensorSource::none;
    assert(!apply_apb_nav_heading_command(model, 900));

    model.navigation.apb.source.value = SensorSource::serial;
    model.navigation.apb.sender_id[0] = 'G';
    model.navigation.apb.sender_id[1] = 'P';
    model.navigation.apb.sender_id[2] = '\0';
    model.navigation.apb.mode_hint.value = AutopilotMode::compass;
    assert(!pypilot_apb_nav_command_available(model));
    assert(!apply_apb_nav_heading_command(model, 1000));

    model.navigation.apb.mode_hint.value = AutopilotMode::gps;
    assert(pypilot_apb_nav_command_available(model));

    return 0;
}
