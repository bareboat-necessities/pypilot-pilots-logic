
        if (old_mode != new_mode) {
            if (old_mode == model.ap.preferred_mode.value && model.ap.heading_command_deg.valid) {
                stored_preferred_command_deg_ = model.ap.heading_command_deg.value;
                stored_preferred_command_us_ = now_us;
                has_stored_preferred_command_ = true;
            }
            model.ap.mode.value = new_mode;
            has_last_heading_command_ = false;
            last_heading_error_int_us_ = 0;
            if (new_mode == model.ap.preferred_mode.value && has_stored_preferred_command_ && now_us - stored_preferred_command_us_ < 30000000ULL) {
                model.ap.heading_command_deg.set(stored_preferred_command_deg_, now_us);
                has_stored_preferred_command_ = false;
            } else if (model.ap.heading_error_deg.valid) {
                Real previous_error = model.ap.heading_error_deg.value;
                if (pypilot_wind_mode(active_mode)) previous_error = -previous_error;
                model.ap.heading_command_deg.set(mode_heading - previous_error, now_us);
            } else if (!model.ap.heading_command_deg.valid) {
                model.ap.heading_command_deg.set(mode_heading, now_us);
            }
        } else if (!model.ap.heading_command_deg.valid) {
            model.ap.heading_command_deg.set(mode_heading, now_us);
        }
    }

    // PyPilot APB/NAV behavior: an active APB route in NAV mode steers to
    // track + xte_gain*xte before heading error and command-rate calculation.
    apply_apb_nav_heading_command(model, now_us);

    if (model.ap.heading_deg.valid && model.ap.heading_command_deg.valid) {
        PypilotMode active_mode = to_logic_mode(model.ap.mode.value);
        bool wind_mode = pypilot_wind_mode(active_mode);
        model.ap.heading_error_deg.set(pypilot_algorithms::pypilot_heading_error(model.ap.heading_deg.value,
                                                                                 model.ap.heading_command_deg.value,
                                                                                 wind_mode),
                                       now_us);
        if (model.ap.enabled.value) {
            if (!has_last_heading_command_) {
                last_heading_command_deg_ = model.ap.heading_command_deg.value;
                has_last_heading_command_ = true;
            }
            Real previous_rate = model.ap.heading_command_rate_deg_s.valid ? model.ap.heading_command_rate_deg_s.value : Real(0);
            model.ap.heading_command_rate_deg_s.set(pypilot_algorithms::pypilot_heading_command_rate(previous_rate,
                                                                                                     model.ap.heading_command_deg.value,
                                                                                                     last_heading_command_deg_,
                                                                                                     wind_mode),
                                                    now_us);
            Real dt_s = last_heading_error_int_us_ == 0 ? Real(0) : Real(now_us - last_heading_error_int_us_) / Real(1000000);
            Real previous_integral = model.ap.heading_error_int_deg.valid ? model.ap.heading_error_int_deg.value : Real(0);
            model.ap.heading_error_int_deg.set(pypilot_algorithms::pypilot_heading_error_integral(previous_integral,
                                                                                                  model.ap.heading_error_deg.value,
                                                                                                  dt_s),
                                               now_us);
            last_heading_error_int_us_ = now_us;
        } else {
            model.ap.heading_command_rate_deg_s.set(Real(0), now_us);
            model.ap.heading_error_int_deg.set(Real(0), now_us);
            has_last_heading_command_ = false;
            last_heading_error_int_us_ = 0;
        }
        last_heading_command_deg_ = model.ap.heading_command_deg.value;
    }
    return true;
}

bool PilotsLogic::compute_command(DataModel& model, uint64_t now_us) {
    last_error_ = "";
    if (!model.ap.enabled.value) {
        Real reset = pypilot_reset_command<Real>();
        model.servo.command_norm.set_internal_command(reset, now_us);
