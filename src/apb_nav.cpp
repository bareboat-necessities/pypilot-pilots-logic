#include <pypilot_pilots_logic/apb_nav.hpp>

#include <cmath>

namespace pypilot_pilots_logic {

bool apply_apb_nav_heading_command(DataModel& model, uint64_t now_us) {
    if (!pypilot_apb_nav_command_available(model)) {
        return false;
    }

    const Real xte = model.navigation.apb.xte_nmi.valid ? model.navigation.apb.xte_nmi.value : Real(0);
    const Real command = pypilot_apb_nav_heading_command(model.navigation.apb.track_deg.value,
                                                         xte,
                                                         model.navigation.apb.xte_gain_deg_per_nmi.value);

    if (!model.ap.heading_command_deg.valid ||
        std::fabs(model.ap.heading_command_deg.value - command) > Real(0.1)) {
        model.ap.heading_command_deg.set(command, now_us);
        return true;
    }

    return false;
}

} // namespace pypilot_pilots_logic
