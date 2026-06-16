#include <pypilot_pilots_logic/pilots_logic.hpp>

#include <pypilot_pilots_logic/math_helpers.hpp>
#include <pypilot_pilots_logic/pilot_select.hpp>

#include <pypilot_algorithms/mode_state.hpp>
#include <pypilot_algorithms/tack.hpp>
#include <pypilot_algorithms/pypilot_filters.hpp>

namespace pypilot_pilots_logic {

static pypilot_algorithms::PypilotMode to_algorithm_mode(pypilot_data_model::AutopilotMode mode) {
    switch (mode) {
    case pypilot_data_model::AutopilotMode::gps:
        return pypilot_algorithms::PypilotMode::gps;
    case pypilot_data_model::AutopilotMode::nav:
        return pypilot_algorithms::PypilotMode::nav;
    case pypilot_data_model::AutopilotMode::wind:
        return pypilot_algorithms::PypilotMode::wind;
    case pypilot_data_model::AutopilotMode::true_wind:
        return pypilot_algorithms::PypilotMode::true_wind;
    case pypilot_data_model::AutopilotMode::compass:
        return pypilot_algorithms::PypilotMode::compass;
    }
    return pypilot_algorithms::PypilotMode::compass;
}

static pypilot_data_model::AutopilotMode to_data_mode(pypilot_algorithms::PypilotMode mode) {
    switch (mode) {
    case pypilot_algorithms::PypilotMode::gps:
        return pypilot_data_model::AutopilotMode::gps;
    case pypilot_algorithms::PypilotMode::nav:
        return pypilot_data_model::AutopilotMode::nav;
    case pypilot_algorithms::PypilotMode::wind:
        return pypilot_data_model::AutopilotMode::wind;
    case pypilot_algorithms::PypilotMode::true_wind:
        return pypilot_data_model::AutopilotMode::true_wind;
    case pypilot_algorithms::PypilotMode::compass:
        return pypilot_data_model::AutopilotMode::compass;
    }
    return pypilot_data_model::AutopilotMode::compass;
}

static uint32_t to_data_mode_mask(uint32_t algorithm_mask) {
    uint32_t mask = 0;
    if (algorithm_mask & pypilot_algorithms::pypilot_mode_mask_compass) {
        mask |= pypilot_data_model::mode_mask_compass;
    }
    if (algorithm_mask & pypilot_algorithms::pypilot_mode_mask_gps) {
        mask |= pypilot_data_model::mode_mask_gps;
    }
    if (algorithm_mask & pypilot_algorithms::pypilot_mode_mask_nav) {
        mask |= pypilot_data_model::mode_mask_nav;
    }
    if (algorithm_mask & pypilot_algorithms::pypilot_mode_mask_wind) {
        mask |= pypilot_data_model::mode_mask_wind;
    }
    if (algorithm_mask & pypilot_algorithms::pypilot_mode_mask_true_wind) {
        mask |= pypilot_data_model::mode_mask_true_wind;
    }
    return mask;
}

static void invalidate_stale_sources(DataModel& model, uint64_t now_us) {
    if (model.navigation.gps.source.value != pypilot_data_model::SensorSource::none &&
        pypilot_algorithms::pypilot_source_is_stale(now_us, model.navigation.gps.last_update_us)) {
        model.navigation.gps.source.value = pypilot_data_model::SensorSource::none;
    }
    if (model.navigation.apb.source.value != pypilot_data_model::SensorSource::none &&
        pypilot_algorithms::pypilot_source_is_stale(now_us, model.navigation.apb.last_update_us)) {
        model.navigation.apb.source.value = pypilot_data_model::SensorSource::none;
    }
    if (model.wind.apparent.source.value != pypilot_data_model::SensorSource::none &&
        pypilot_algorithms::pypilot_source_is_stale(now_us, model.wind.apparent.last_update_us)) {
        model.wind.apparent.source.value = pypilot_data_model::SensorSource::none;
        model.wind.apparent.speed_kn.valid = false;
        model.wind.apparent.direction_deg.valid = false;
    }
    if (model.wind.truewind.source.value != pypilot_data_model::SensorSource::none &&
        pypilot_algorithms::pypilot_source_is_stale(now_us, model.wind.truewind.last_update_us)) {
        model.wind.truewind.source.value = pypilot_data_model::SensorSource::none;
        model.wind.truewind.speed_kn.valid = false;
        model.wind.truewind.direction_deg.valid = false;
    }
    if (model.water.source.value != pypilot_data_model::SensorSource::none &&
        pypilot_algorithms::pypilot_source_is_stale(now_us, model.water.last_update_us)) {
        model.water.source.value = pypilot_data_model::SensorSource::none;
        model.water.speed_kn.valid = false;
        model.water.leeway_deg.valid = false;
    }
}

PilotsLogic::PilotsLogic()
    : last_error_(""),
      last_wind_speed_kn_(0.0f),
      has_last_wind_speed_(false),
      last_heading_command_deg_(0.0f),
      has_last_heading_command_(false),
      last_heading_error_int_us_(0),
      gps_speed_filter_kn_(0.0f),
      stored_preferred_command_deg_(0.0f),
      stored_preferred_command_us_(0),
      has_stored_preferred_command_(false),
      tack_state_(pypilot_algorithms::PypilotTackState::none),
      tack_direction_(pypilot_algorithms::PypilotTackDirection::none),
      tack_current_direction_(pypilot_algorithms::PypilotTackDirection::none),
      tack_delay_s_(0.0f),
      tack_angle_deg_(100.0f),
      tack_rate_deg_s_(15.0f),
      tack_threshold_percent_(50.0f),
      tack_state_start_us_(0) {}

void PilotsLogic::begin_tack(pypilot_algorithms::PypilotTackDirection direction) {
    tack_direction_ = direction;
    tack_state_ = pypilot_algorithms::PypilotTackState::begin;
}

void PilotsLogic::cancel_tack() {
    tack_state_ = pypilot_algorithms::PypilotTackState::none;
    tack_direction_ = pypilot_algorithms::PypilotTackDirection::none;
    tack_current_direction_ = pypilot_algorithms::PypilotTackDirection::none;
    tack_state_start_us_ = 0;
}

pypilot_algorithms::PypilotTackState PilotsLogic::tack_state() const {
    return tack_state_;
}

pypilot_algorithms::PypilotTackDirection PilotsLogic::tack_direction() const {
    return tack_direction_;
}

void PilotsLogic::set_tack_config(Real delay_s,
                                  Real angle_deg,
                                  Real rate_deg_s,
                                  Real threshold_percent) {
    tack_delay_s_ = delay_s;
    tack_angle_deg_ = angle_deg;
    tack_rate_deg_s_ = rate_deg_s;
    tack_threshold_percent_ = threshold_percent;
}

bool PilotsLogic::update_inputs(DataModel& model, uint64_t now_us) {
    last_error_ = "";

    invalidate_stale_sources(model, now_us);

    if (model.wind.apparent.source.value != pypilot_data_model::SensorSource::none &&
        model.wind.apparent.speed_kn.valid) {
        Real previous_speed = model.wind.apparent.filtered_speed_kn.valid
                                  ? model.wind.apparent.filtered_speed_kn.value
                                  : Real(0);
        Real filtered_speed = pypilot_algorithms::pypilot_wind_filtered_speed(
            previous_speed,
            model.wind.apparent.speed_kn.value);
        model.wind.apparent.filtered_speed_kn.set(filtered_speed, now_us);

        if (model.wind.apparent.direction_deg.valid) {
            Real filter_constant = model.wind.apparent.filter_constant_0_1.value;
            if (filter_constant == Real(0)) {
                filter_constant = Real(0.1);
            }
            Real factor = pypilot_algorithms::pypilot_wind_filter_factor(filter_constant,
                                                                         filtered_speed);
            model.wind.apparent.filter_factor_0_1.set(factor, now_us);
            Real previous_direction = model.wind.apparent.filtered_direction_deg.valid
                                          ? model.wind.apparent.filtered_direction_deg.value
                                          : Real(0);
            model.wind.apparent.filtered_direction_deg.set(
                pypilot_algorithms::pypilot_wind_filtered_direction(
                    previous_direction,
                    model.wind.apparent.filtered_direction_deg.valid,
                    model.wind.apparent.direction_deg.value,
                    factor),
                now_us);
        }
    }

    Real boat_speed_kn = Real(0);
    bool has_boat_speed = false;
    pypilot_data_model::SensorSource synthesized_truewind_source = pypilot_data_model::SensorSource::none;
    if (model.water.source.value != pypilot_data_model::SensorSource::none &&
        model.water.speed_kn.valid) {
        boat_speed_kn = model.water.speed_kn.value;
        has_boat_speed = true;
        synthesized_truewind_source = pypilot_data_model::SensorSource::water_wind;
        if (model.water.leeway_source.value != pypilot_data_model::SensorSource::serial &&
            model.imu.heel_deg.valid) {
            Real leeway = pypilot_algorithms::pypilot_leeway_deg(model.imu.heel_deg.value,
                                                                 model.water.speed_kn.value);
            if (leeway != Real(0)) {
                model.water.leeway_deg.set(leeway, now_us);
                model.water.leeway_source.value = pypilot_data_model::SensorSource::water_wind;
            }
        }
    } else if (model.navigation.gps.source.value != pypilot_data_model::SensorSource::none &&
               model.navigation.gps.speed_kn.valid) {
        boat_speed_kn = gps_speed_filter_kn_;
        has_boat_speed = true;
        synthesized_truewind_source = pypilot_data_model::SensorSource::gps_wind;
    }

    bool can_synthesize_truewind =
        model.wind.truewind.source.value == pypilot_data_model::SensorSource::none ||
        model.wind.truewind.source.value == pypilot_data_model::SensorSource::water_wind ||
        model.wind.truewind.source.value == pypilot_data_model::SensorSource::gps_wind;
    if (can_synthesize_truewind && has_boat_speed &&
        model.wind.apparent.source.value != pypilot_data_model::SensorSource::none &&
        model.wind.apparent.speed_kn.valid && model.wind.apparent.direction_deg.valid) {
        model.wind.truewind.source.value = synthesized_truewind_source;
        model.wind.truewind.speed_kn.set(
            pypilot_algorithms::pypilot_true_wind_speed(boat_speed_kn,
                                                        model.wind.apparent.speed_kn.value,
                                                        model.wind.apparent.direction_deg.value),
            now_us);
        model.wind.truewind.direction_deg.set(
            pypilot_algorithms::pypilot_true_wind_direction(boat_speed_kn,
                                                            model.wind.apparent.speed_kn.value,
                                                            model.wind.apparent.direction_deg.value),
            now_us);
        model.wind.truewind.last_update_us = now_us;
    }

    if (model.wind.truewind.source.value != pypilot_data_model::SensorSource::none &&
        model.wind.truewind.speed_kn.valid) {
        Real previous_speed = model.wind.truewind.filtered_speed_kn.valid
                                  ? model.wind.truewind.filtered_speed_kn.value
                                  : Real(0);
        Real filtered_speed = pypilot_algorithms::pypilot_wind_filtered_speed(
            previous_speed,
            model.wind.truewind.speed_kn.value);
        model.wind.truewind.filtered_speed_kn.set(filtered_speed, now_us);

        if (model.wind.truewind.direction_deg.valid) {
            Real filter_constant = model.wind.truewind.filter_constant_0_1.value;
            if (filter_constant == Real(0)) {
                filter_constant = Real(0.1);
            }
            Real factor = pypilot_algorithms::pypilot_wind_filter_factor(filter_constant,
                                                                         filtered_speed);
            model.wind.truewind.filter_factor_0_1.set(factor, now_us);
            Real previous_direction = model.wind.truewind.filtered_direction_deg.valid
                                          ? model.wind.truewind.filtered_direction_deg.value
                                          : Real(0);
            model.wind.truewind.filtered_direction_deg.set(
                pypilot_algorithms::pypilot_wind_filtered_direction(
                    previous_direction,
                    model.wind.truewind.filtered_direction_deg.valid,
                    model.wind.truewind.direction_deg.value,
                    factor),
                now_us);
        }
    }

    if (model.imu.heading_deg.valid) {
        Real alpha = model.imu.heading_lowpass_constant_0_1.value;
        if (alpha == Real(0)) {
            alpha = Real(0.2);
        }
        Real previous = model.imu.heading_lowpass_deg.valid ? model.imu.heading_lowpass_deg.value
                                                            : model.imu.heading_deg.value;
        model.imu.heading_lowpass_deg.set(
            pypilot_algorithms::pypilot_heading_filter(alpha,
                                                       model.imu.heading_deg.value,
                                                       previous),
            now_us);
    }

    if (model.imu.heading_rate_deg_s.valid) {
        Real alpha = model.imu.headingrate_lowpass_constant_0_1.value;
        if (alpha == Real(0)) {
            alpha = Real(0.2);
        }
        Real previous = model.imu.heading_rate_lowpass_deg_s.valid
                            ? model.imu.heading_rate_lowpass_deg_s.value
                            : model.imu.heading_rate_deg_s.value;
        model.imu.heading_rate_lowpass_deg_s.set(
            pypilot_algorithms::pypilot_lowpass(alpha,
                                                model.imu.heading_rate_deg_s.value,
                                                previous),
            now_us);
    }

    if (model.imu.heading_rate_rate_deg_s2.valid) {
        Real alpha = model.imu.headingraterate_lowpass_constant_0_1.value;
        if (alpha == Real(0)) {
            alpha = Real(0.1);
        }
        Real previous = model.imu.heading_rate_rate_lowpass_deg_s2.valid
                            ? model.imu.heading_rate_rate_lowpass_deg_s2.value
                            : model.imu.heading_rate_rate_deg_s2.value;
        model.imu.heading_rate_rate_lowpass_deg_s2.set(
            pypilot_algorithms::pypilot_lowpass(alpha,
                                                model.imu.heading_rate_rate_deg_s2.value,
                                                previous),
            now_us);
    }

    if (model.imu.heading_lowpass_deg.valid) {
        if (model.navigation.gps.source.value != pypilot_data_model::SensorSource::none &&
            model.navigation.gps.speed_kn.valid) {
            gps_speed_filter_kn_ =
                pypilot_algorithms::pypilot_gps_speed_filter(gps_speed_filter_kn_,
                                                             model.navigation.gps.speed_kn.value);
            if (model.navigation.gps.track_deg.valid && model.navigation.gps.speed_kn.value > Real(1)) {
                Real previous_offset = model.ap.gps_compass_offset_deg.valid
                                           ? model.ap.gps_compass_offset_deg.value
                                           : Real(0);
                Real measured_offset = pypilot_algorithms::pypilot_gps_heading_offset_measurement(
                    model.navigation.gps.track_deg.value,
                    model.imu.heading_lowpass_deg.value);
                model.ap.gps_compass_offset_deg.set(
                    pypilot_algorithms::pypilot_heading_offset_filter(
                        previous_offset,
                        measured_offset,
                        pypilot_algorithms::pypilot_gps_heading_offset_alpha(gps_speed_filter_kn_)),
                    now_us);
            }
        }

        if (model.wind.apparent.source.value != pypilot_data_model::SensorSource::none &&
            model.wind.apparent.filtered_direction_deg.valid) {
            Real previous_offset = model.ap.wind_compass_offset_deg.valid
                                       ? model.ap.wind_compass_offset_deg.value
                                       : Real(0);
            Real alpha = model.ap.wind_offset_filter_0_1.value;
            if (alpha == Real(0)) {
                alpha = Real(0.1);
            }
            Real measured_offset = pypilot_algorithms::pypilot_wind_heading_offset_measurement(
                model.wind.apparent.filtered_direction_deg.value,
                model.imu.heading_lowpass_deg.value);
            model.ap.wind_compass_offset_deg.set(
                pypilot_algorithms::pypilot_heading_offset_filter(previous_offset,
                                                                  measured_offset,
                                                                  alpha),
                now_us);
        }

        if (model.wind.truewind.source.value != pypilot_data_model::SensorSource::none &&
            model.wind.truewind.filtered_direction_deg.valid) {
            Real previous_offset = model.ap.true_wind_compass_offset_deg.valid
                                       ? model.ap.true_wind_compass_offset_deg.value
                                       : Real(0);
            Real alpha = model.ap.wind_offset_filter_0_1.value;
            if (alpha == Real(0)) {
                alpha = Real(0.1);
            }
            Real measured_offset = pypilot_algorithms::pypilot_wind_heading_offset_measurement(
                model.wind.truewind.filtered_direction_deg.value,
                model.imu.heading_lowpass_deg.value);
            model.ap.true_wind_compass_offset_deg.set(
                pypilot_algorithms::pypilot_heading_offset_filter(previous_offset,
                                                                  measured_offset,
                                                                  alpha),
                now_us);
        }

        bool compass_available = model.imu.heading_lowpass_deg.valid;
        bool gps_available = model.navigation.gps.source.value != pypilot_data_model::SensorSource::none &&
                             model.navigation.gps.track_deg.valid;
        bool nav_available = model.navigation.apb.source.value != pypilot_data_model::SensorSource::none &&
                             model.navigation.apb.track_deg.valid;
        bool wind_available = model.wind.apparent.source.value != pypilot_data_model::SensorSource::none &&
                              model.wind.apparent.filtered_direction_deg.valid;
        bool true_wind_available = model.wind.truewind.source.value != pypilot_data_model::SensorSource::none &&
                                   model.wind.truewind.filtered_direction_deg.valid;

        uint32_t algorithm_modes = pypilot_algorithms::pypilot_available_modes(
            compass_available,
            gps_available,
            nav_available,
            wind_available,
            true_wind_available,
            model.ap.gps_and_nav_modes.value);
        model.ap.available_modes_mask = to_data_mode_mask(algorithm_modes);

        pypilot_algorithms::PypilotMode preferred_mode = to_algorithm_mode(model.ap.preferred_mode.value);
        pypilot_algorithms::PypilotMode active_mode =
            pypilot_algorithms::pypilot_best_mode(preferred_mode, algorithm_modes);
        pypilot_data_model::AutopilotMode old_mode = model.ap.mode.value;
        pypilot_data_model::AutopilotMode new_mode = to_data_mode(active_mode);

        Real gps_offset = model.ap.gps_compass_offset_deg.valid ? model.ap.gps_compass_offset_deg.value
                                                                : Real(0);
        Real wind_offset = model.ap.wind_compass_offset_deg.valid ? model.ap.wind_compass_offset_deg.value
                                                                  : Real(0);
        Real true_wind_offset = model.ap.true_wind_compass_offset_deg.valid
                                    ? model.ap.true_wind_compass_offset_deg.value
                                    : Real(0);
        Real mode_heading = pypilot_algorithms::pypilot_mode_heading(active_mode,
                                                                     model.imu.heading_lowpass_deg.value,
                                                                     gps_offset,
                                                                     wind_offset,
                                                                     true_wind_offset);
        model.ap.heading_deg.set(mode_heading, now_us);

        if (old_mode != new_mode) {
            if (old_mode == model.ap.preferred_mode.value && model.ap.heading_command_deg.valid) {
                stored_preferred_command_deg_ = model.ap.heading_command_deg.value;
                stored_preferred_command_us_ = now_us;
                has_stored_preferred_command_ = true;
            }
            model.ap.mode.value = new_mode;
            has_last_heading_command_ = false;
            last_heading_error_int_us_ = 0;

            if (new_mode == model.ap.preferred_mode.value && has_stored_preferred_command_ &&
                now_us - stored_preferred_command_us_ < 30000000ULL) {
                model.ap.heading_command_deg.set(stored_preferred_command_deg_, now_us);
                has_stored_preferred_command_ = false;
            } else if (model.ap.heading_error_deg.valid) {
                Real previous_error = model.ap.heading_error_deg.value;
                if (pypilot_algorithms::pypilot_wind_mode(active_mode)) {
                    previous_error = -previous_error;
                }
                model.ap.heading_command_deg.set(mode_heading - previous_error, now_us);
            } else if (!model.ap.heading_command_deg.valid) {
                model.ap.heading_command_deg.set(mode_heading, now_us);
            }
        } else if (!model.ap.heading_command_deg.valid) {
            model.ap.heading_command_deg.set(mode_heading, now_us);
        }
    }

    if (model.ap.heading_deg.valid && model.ap.heading_command_deg.valid) {
        pypilot_algorithms::PypilotMode active_mode = to_algorithm_mode(model.ap.mode.value);
        bool wind_mode = pypilot_algorithms::pypilot_wind_mode(active_mode);
        model.ap.heading_error_deg.set(
            pypilot_algorithms::pypilot_heading_error(model.ap.heading_deg.value,
                                                      model.ap.heading_command_deg.value,
                                                      wind_mode),
            now_us);

        if (model.ap.enabled.value) {
            if (!has_last_heading_command_) {
                last_heading_command_deg_ = model.ap.heading_command_deg.value;
                has_last_heading_command_ = true;
            }
            Real previous_rate = model.ap.heading_command_rate_deg_s.valid
                                     ? model.ap.heading_command_rate_deg_s.value
                                     : Real(0);
            model.ap.heading_command_rate_deg_s.set(
                pypilot_algorithms::pypilot_heading_command_rate(previous_rate,
                                                                 model.ap.heading_command_deg.value,
                                                                 last_heading_command_deg_,
                                                                 wind_mode),
                now_us);
        } else {
            model.ap.heading_command_rate_deg_s.set(Real(0), now_us);
            has_last_heading_command_ = false;
        }

        if (model.ap.heading_error_deg.valid) {
            Real dt_s = last_heading_error_int_us_ == 0
                            ? Real(0)
                            : Real(now_us - last_heading_error_int_us_) / Real(1000000);
            Real previous_integral = model.ap.heading_error_int_deg.valid
                                         ? model.ap.heading_error_int_deg.value
                                         : Real(0);
            model.ap.heading_error_int_deg.set(
                pypilot_algorithms::pypilot_heading_error_integral(previous_integral,
                                                                   model.ap.heading_error_deg.value,
                                                                   dt_s),
                now_us);
            last_heading_error_int_us_ = now_us;
        }

        last_heading_command_deg_ = model.ap.heading_command_deg.value;
    }

    return true;
}

bool PilotsLogic::compute_command(DataModel& model, uint64_t now_us) {
    last_error_ = "";

    if (!model.ap.enabled.value) {
        model.servo.command_norm.set_internal_command(0.0f, now_us);
        cancel_tack();
        return true;
    }

    if (tack_state_ == pypilot_algorithms::PypilotTackState::begin) {
        if (tack_direction_ == pypilot_algorithms::PypilotTackDirection::none) {
            cancel_tack();
        } else {
            tack_current_direction_ = tack_direction_;
            tack_state_ = pypilot_algorithms::PypilotTackState::waiting;
            tack_state_start_us_ = now_us;
        }
    }

    if (tack_state_ == pypilot_algorithms::PypilotTackState::waiting) {
        Real elapsed_s = Real(now_us - tack_state_start_us_) / Real(1000000);
        if (elapsed_s >= tack_delay_s_) {
            tack_state_ = pypilot_algorithms::PypilotTackState::tacking;
            tack_state_start_us_ = now_us;
        }
    }

    if (tack_state_ == pypilot_algorithms::PypilotTackState::tacking) {
        pypilot_algorithms::PypilotTackInput<Real> input;
        input.direction = tack_current_direction_;
        input.heading_command_deg = model.ap.heading_command_deg.value;
        input.heading_deg = model.ap.heading_deg.valid ? model.ap.heading_deg.value
                                                       : model.imu.heading_lowpass_deg.value;
        input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.valid
                                      ? model.imu.heading_rate_lowpass_deg_s.value
                                      : Real(0);
        input.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.valid
                                           ? model.imu.heading_rate_rate_lowpass_deg_s2.value
                                           : Real(0);
        input.wind_mode = model.ap.mode.value == pypilot_data_model::AutopilotMode::wind ||
                          model.ap.mode.value == pypilot_data_model::AutopilotMode::true_wind;
        input.apparent_wind_direction_deg = model.wind.apparent.direction_deg.valid
                                                ? model.wind.apparent.direction_deg.value
                                                : (model.wind.apparent.filtered_direction_deg.valid
                                                       ? model.wind.apparent.filtered_direction_deg.value
                                                       : Real(0));

        pypilot_algorithms::PypilotTackConfig<Real> config;
        config.delay_s = tack_delay_s_;
        config.angle_deg = tack_angle_deg_;
        config.rate_deg_s = tack_rate_deg_s_;
        config.threshold_percent = tack_threshold_percent_;

        pypilot_algorithms::PypilotTackOutput<Real> tack =
            pypilot_algorithms::pypilot_tack_compute(input, config);
        if (tack.completed) {
            model.ap.heading_command_deg.set(tack.new_heading_command_deg, now_us);
            cancel_tack();
        } else if (tack.override_pilot) {
            model.servo.command_norm.set_internal_command(tack.command_norm, now_us);
            return true;
        }
    }

    PilotResult result = compute_selected_pilot(model,
                                                now_us,
                                                last_wind_speed_kn_,
                                                has_last_wind_speed_);
    if (!result.valid) {
        last_error_ = "pilot command invalid";
        return false;
    }

    return true;
}

bool PilotsLogic::step(DataModel& model, uint64_t now_us) {
    return update_inputs(model, now_us) && compute_command(model, now_us);
}

const char* PilotsLogic::last_error() const {
    return last_error_;
}

} // namespace pypilot_pilots_logic
