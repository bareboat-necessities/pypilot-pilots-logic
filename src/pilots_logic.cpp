#include <pypilot_pilots_logic/pilots_logic.hpp>

#include <pypilot_pilots_logic/math_helpers.hpp>
#include <pypilot_pilots_logic/pilot_select.hpp>

#include <pypilot_algorithms/pypilot_filters.hpp>

namespace pypilot_pilots_logic {

PilotsLogic::PilotsLogic()
    : last_error_(""),
      last_wind_speed_kn_(0.0f),
      has_last_wind_speed_(false),
      last_heading_command_deg_(0.0f),
      has_last_heading_command_(false),
      last_heading_error_int_us_(0),
      gps_speed_filter_kn_(0.0f) {}

bool PilotsLogic::update_inputs(DataModel& model, uint64_t now_us) {
    last_error_ = "";

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

        model.ap.heading_deg.set(model.imu.heading_lowpass_deg.value, now_us);
    }

    if (model.ap.heading_deg.valid && model.ap.heading_command_deg.valid) {
        bool wind_mode = model.ap.mode.value == pypilot_data_model::AutopilotMode::wind ||
                         model.ap.mode.value == pypilot_data_model::AutopilotMode::true_wind;
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
        return true;
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
