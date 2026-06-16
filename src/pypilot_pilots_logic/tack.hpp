#pragma once

#include <pypilot_algorithms/angles.hpp>
#include <pypilot_algorithms/control.hpp>
#include <pypilot_algorithms/pypilot_filters.hpp>

namespace pypilot_pilots_logic {

enum class PypilotTackState : unsigned char { none, begin, waiting, tacking };
enum class PypilotTackDirection : unsigned char { none, port, starboard };

template<typename Real = float>
struct PypilotTackConfig { Real delay_s = Real(0); Real angle_deg = Real(100); Real rate_deg_s = Real(15); Real threshold_percent = Real(50); };

template<typename Real = float>
struct PypilotTackInput {
    PypilotTackDirection direction = PypilotTackDirection::none;
    Real heading_command_deg = Real(0);
    Real heading_deg = Real(0);
    Real headingrate_deg_s = Real(0);
    Real headingraterate_deg_s2 = Real(0);
    bool wind_mode = false;
    Real apparent_wind_direction_deg = Real(0);
};

template<typename Real = float>
struct PypilotTackOutput { bool override_pilot = false; bool completed = false; Real command_norm = Real(0); Real new_heading_command_deg = Real(0); };

template<typename Real = float>
inline PypilotTackOutput<Real> pypilot_tack_compute(const PypilotTackInput<Real>& input,
                                                    const PypilotTackConfig<Real>& config) {
    PypilotTackOutput<Real> output;
    if (input.direction == PypilotTackDirection::none || config.rate_deg_s == Real(0)) return output;
    Real direction = input.direction == PypilotTackDirection::port ? Real(1) : Real(-1);
    Real final_heading_command = input.heading_command_deg - direction * config.angle_deg;
    Real progress = Real(0);
    if (input.wind_mode) {
        Real winddir = pypilot_algorithms::pypilot_resolv(input.apparent_wind_direction_deg);
        final_heading_command = -input.heading_command_deg;
        if (pypilot_algorithms::abs_real(input.heading_command_deg) < Real(90)) {
            direction = input.heading_command_deg < Real(0) ? Real(1) : Real(-1);
            if (input.heading_command_deg != Real(0)) progress = (Real(1) - winddir / input.heading_command_deg) / Real(2);
        } else {
            direction = input.heading_command_deg > Real(0) ? Real(1) : Real(-1);
            Real pcommand = pypilot_algorithms::pypilot_resolv_to(input.heading_command_deg, Real(180));
            Real denominator = Real(180) - pcommand;
            if (denominator != Real(0)) progress = (pypilot_algorithms::pypilot_resolv_to(winddir, Real(180)) - pcommand) / denominator / Real(2);
        }
    } else {
        Real resolved_heading = pypilot_algorithms::pypilot_resolv_to(input.heading_deg, input.heading_command_deg);
        progress = direction * (input.heading_command_deg - resolved_heading) / config.angle_deg;
    }
    if (Real(100) * progress > config.threshold_percent) {
        output.completed = true;
        output.new_heading_command_deg = final_heading_command;
        return output;
    }
    Real turn_command = (input.headingrate_deg_s + input.headingraterate_deg_s2 / Real(2)) / config.rate_deg_s + direction;
    output.override_pilot = true;
    output.command_norm = pypilot_algorithms::clamp(turn_command, Real(-1), Real(1));
    output.new_heading_command_deg = final_heading_command;
    return output;
}

} // namespace pypilot_pilots_logic
