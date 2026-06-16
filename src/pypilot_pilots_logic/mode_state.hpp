#pragma once

#include <stdint.h>
#include <pypilot_algorithms/pypilot_filters.hpp>

namespace pypilot_pilots_logic {

enum class PypilotMode : uint8_t {
    compass,
    gps,
    nav,
    wind,
    true_wind
};

enum PypilotModeMask : uint32_t {
    pypilot_mode_mask_compass   = 1u << 0,
    pypilot_mode_mask_gps       = 1u << 1,
    pypilot_mode_mask_nav       = 1u << 2,
    pypilot_mode_mask_wind      = 1u << 3,
    pypilot_mode_mask_true_wind = 1u << 4
};

inline uint32_t pypilot_mode_mask(PypilotMode mode) {
    switch (mode) {
    case PypilotMode::compass:   return pypilot_mode_mask_compass;
    case PypilotMode::gps:       return pypilot_mode_mask_gps;
    case PypilotMode::nav:       return pypilot_mode_mask_nav;
    case PypilotMode::wind:      return pypilot_mode_mask_wind;
    case PypilotMode::true_wind: return pypilot_mode_mask_true_wind;
    }
    return pypilot_mode_mask_compass;
}

inline bool pypilot_mode_available(uint32_t available_modes_mask, PypilotMode mode) {
    return (available_modes_mask & pypilot_mode_mask(mode)) != 0;
}

inline uint32_t pypilot_available_modes(bool compass_available,
                                        bool gps_available,
                                        bool nav_available,
                                        bool wind_available,
                                        bool true_wind_available,
                                        bool gps_and_nav_modes) {
    uint32_t mask = 0;
    if (compass_available) mask |= pypilot_mode_mask_compass;
    if (gps_available) {
        if (gps_and_nav_modes || !nav_available) mask |= pypilot_mode_mask_gps;
        if (nav_available) mask |= pypilot_mode_mask_nav;
    }
    if (wind_available) mask |= pypilot_mode_mask_wind;
    if (true_wind_available) mask |= pypilot_mode_mask_true_wind;
    return mask;
}

inline PypilotMode pypilot_fallback_mode(PypilotMode mode) {
    switch (mode) {
    case PypilotMode::nav:       return PypilotMode::gps;
    case PypilotMode::gps:       return PypilotMode::compass;
    case PypilotMode::wind:      return PypilotMode::compass;
    case PypilotMode::true_wind: return PypilotMode::wind;
    case PypilotMode::compass:   return PypilotMode::compass;
    }
    return PypilotMode::compass;
}

inline PypilotMode pypilot_best_mode(PypilotMode preferred_mode,
                                     uint32_t available_modes_mask) {
    PypilotMode mode = preferred_mode;
    for (int i = 0; i < 5; ++i) {
        if (pypilot_mode_available(available_modes_mask, mode)) return mode;
        PypilotMode next = pypilot_fallback_mode(mode);
        if (next == mode) break;
        mode = next;
    }
    if (pypilot_mode_available(available_modes_mask, PypilotMode::compass)) return PypilotMode::compass;
    return preferred_mode;
}

template<typename Real = float>
inline Real pypilot_mode_heading(PypilotMode mode,
                                 Real compass_heading_deg,
                                 Real gps_compass_offset_deg,
                                 Real wind_compass_offset_deg,
                                 Real true_wind_compass_offset_deg) {
    switch (mode) {
    case PypilotMode::true_wind: return pypilot_algorithms::pypilot_resolv(true_wind_compass_offset_deg - compass_heading_deg);
    case PypilotMode::wind:      return pypilot_algorithms::pypilot_resolv(wind_compass_offset_deg - compass_heading_deg);
    case PypilotMode::gps:
    case PypilotMode::nav:       return pypilot_algorithms::pypilot_resolv_to(compass_heading_deg + gps_compass_offset_deg, Real(180));
    case PypilotMode::compass:   return compass_heading_deg;
    }
    return compass_heading_deg;
}

inline bool pypilot_wind_mode(PypilotMode mode) {
    return mode == PypilotMode::wind || mode == PypilotMode::true_wind;
}

} // namespace pypilot_pilots_logic
