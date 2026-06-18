        model.servo.position_command_deg.set_internal_command(reset, now_us);
        cancel_tack();
        return true;
    }
    if (!pypilot_servo_allows_command(model.servo.flags.value)) {
        Real reset = pypilot_reset_command<Real>();
        model.servo.command_norm.set_internal_command(reset, now_us);
        model.servo.position_command_deg.set_internal_command(reset, now_us);
        cancel_tack();
        last_error_ = "servo fault";
        return false;
    }
    if (tack_state_ == PypilotTackState::begin) {
        if (tack_direction_ == PypilotTackDirection::none) {
            cancel_tack();
        } else {
            tack_current_direction_ = tack_direction_;
            tack_state_ = PypilotTackState::waiting;
            tack_state_start_us_ = now_us;
        }
    }
    if (tack_state_ == PypilotTackState::waiting) {
        Real elapsed_s = Real(now_us - tack_state_start_us_) / Real(1000000);
        if (elapsed_s >= tack_delay_s_) {
            tack_state_ = PypilotTackState::tacking;
            tack_state_start_us_ = now_us;
        }
    }
    if (tack_state_ == PypilotTackState::tacking) {
        PypilotTackInput<Real> input;
        input.direction = tack_current_direction_;
        input.heading_command_deg = model.ap.heading_command_deg.value;
        input.heading_deg = model.ap.heading_deg.valid ? model.ap.heading_deg.value : model.imu.heading_lowpass_deg.value;
        input.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.valid ? model.imu.heading_rate_lowpass_deg_s.value : Real(0);
        input.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.valid ? model.imu.heading_rate_rate_lowpass_deg_s2.value : Real(0);
        input.wind_mode = model.ap.mode.value == pypilot_data_model::AutopilotMode::wind || model.ap.mode.value == pypilot_data_model::AutopilotMode::true_wind;
        input.apparent_wind_direction_deg = model.wind.apparent.direction_deg.valid ? model.wind.apparent.direction_deg.value : (model.wind.apparent.filtered_direction_deg.valid ? model.wind.apparent.filtered_direction_deg.value : Real(0));
        PypilotTackConfig<Real> config;
        config.delay_s = tack_delay_s_;
        config.angle_deg = tack_angle_deg_;
        config.rate_deg_s = tack_rate_deg_s_;
        config.threshold_percent = tack_threshold_percent_;
        PypilotTackOutput<Real> tack = pypilot_tack_compute(input, config);
        if (tack.completed) {
            model.ap.heading_command_deg.set(tack.new_heading_command_deg, now_us);
            cancel_tack();
        } else if (tack.override_pilot) {
            model.servo.command_norm.set_internal_command(tack.command_norm, now_us);
            model.servo.position_command_deg.set_internal_command(Real(0), now_us);
            return true;
        }
    }
    PilotResult result = compute_selected_pilot(model, now_us, last_wind_speed_kn_, has_last_wind_speed_);
    if (!result.valid) {
        Real reset = pypilot_reset_command<Real>();
        model.servo.command_norm.set_internal_command(reset, now_us);
        model.servo.position_command_deg.set_internal_command(reset, now_us);
        last_error_ = "pilot command invalid";
        return false;
    }
    return true;
}

bool PilotsLogic::step(DataModel& model, uint64_t now_us) {
    return update_inputs(model, now_us) && compute_command(model, now_us);
}

const char* PilotsLogic::last_error() const { return last_error_; }

} // namespace pypilot_pilots_logic
