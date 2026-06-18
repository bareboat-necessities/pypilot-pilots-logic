    if (model.imu.heading_deg.valid) {
        Real alpha = model.imu.heading_lowpass_constant_0_1.value == Real(0) ? Real(0.2) : model.imu.heading_lowpass_constant_0_1.value;
        Real previous = model.imu.heading_lowpass_deg.valid ? model.imu.heading_lowpass_deg.value : model.imu.heading_deg.value;
        model.imu.heading_lowpass_deg.set(pypilot_algorithms::pypilot_heading_filter(alpha, model.imu.heading_deg.value, previous), now_us);
    }
    if (model.imu.heading_rate_deg_s.valid) {
        Real alpha = model.imu.headingrate_lowpass_constant_0_1.value == Real(0) ? Real(0.2) : model.imu.headingrate_lowpass_constant_0_1.value;
        Real previous = model.imu.heading_rate_lowpass_deg_s.valid ? model.imu.heading_rate_lowpass_deg_s.value : model.imu.heading_rate_deg_s.value;
        model.imu.heading_rate_lowpass_deg_s.set(pypilot_algorithms::pypilot_lowpass(alpha, model.imu.heading_rate_deg_s.value, previous), now_us);
    }
    if (model.imu.heading_rate_rate_deg_s2.valid) {
        Real alpha = model.imu.headingraterate_lowpass_constant_0_1.value == Real(0) ? Real(0.1) : model.imu.headingraterate_lowpass_constant_0_1.value;
        Real previous = model.imu.heading_rate_rate_lowpass_deg_s2.valid ? model.imu.heading_rate_rate_lowpass_deg_s2.value : model.imu.heading_rate_rate_deg_s2.value;
        model.imu.heading_rate_rate_lowpass_deg_s2.set(pypilot_algorithms::pypilot_lowpass(alpha, model.imu.heading_rate_rate_deg_s2.value, previous), now_us);
    }

    if (model.imu.heading_lowpass_deg.valid) {
        if (model.navigation.gps.source.value != pypilot_data_model::SensorSource::none && model.navigation.gps.speed_kn.valid) {
            gps_speed_filter_kn_ = pypilot_algorithms::pypilot_gps_speed_filter(gps_speed_filter_kn_, model.navigation.gps.speed_kn.value);
            if (model.navigation.gps.track_deg.valid && model.navigation.gps.speed_kn.value > Real(1)) {
                bool has_offset = model.ap.gps_compass_offset_deg.valid;
                Real previous_offset = has_offset ? model.ap.gps_compass_offset_deg.value : Real(0);
                Real measured_offset = pypilot_algorithms::pypilot_gps_heading_offset_measurement(model.navigation.gps.track_deg.value,
                                                                                                  model.imu.heading_lowpass_deg.value);
                model.ap.gps_compass_offset_deg.set(
                    pypilot_algorithms::pypilot_heading_offset_align_or_filter(has_offset,
                                                                               previous_offset,
                                                                               measured_offset,
                                                                               pypilot_algorithms::pypilot_gps_heading_offset_alpha(gps_speed_filter_kn_)),
                    now_us);
            }
        }
        if (model.wind.apparent.source.value != pypilot_data_model::SensorSource::none && model.wind.apparent.filtered_direction_deg.valid) {
            bool has_offset = model.ap.wind_compass_offset_deg.valid;
            Real previous_offset = has_offset ? model.ap.wind_compass_offset_deg.value : Real(0);
            Real alpha = model.ap.wind_offset_filter_0_1.value == Real(0) ? Real(0.1) : model.ap.wind_offset_filter_0_1.value;
            Real measured_offset = pypilot_algorithms::pypilot_wind_heading_offset_measurement(model.wind.apparent.filtered_direction_deg.value,
                                                                                               model.imu.heading_lowpass_deg.value);
            model.ap.wind_compass_offset_deg.set(pypilot_algorithms::pypilot_heading_offset_align_or_filter(has_offset, previous_offset, measured_offset, alpha), now_us);
        }
        if (model.wind.truewind.source.value != pypilot_data_model::SensorSource::none && model.wind.truewind.filtered_direction_deg.valid) {
            bool has_offset = model.ap.true_wind_compass_offset_deg.valid;
            Real previous_offset = has_offset ? model.ap.true_wind_compass_offset_deg.value : Real(0);
            Real alpha = model.ap.wind_offset_filter_0_1.value == Real(0) ? Real(0.1) : model.ap.wind_offset_filter_0_1.value;
            Real measured_offset = pypilot_algorithms::pypilot_wind_heading_offset_measurement(model.wind.truewind.filtered_direction_deg.value,
                                                                                               model.imu.heading_lowpass_deg.value);
            model.ap.true_wind_compass_offset_deg.set(pypilot_algorithms::pypilot_heading_offset_align_or_filter(has_offset, previous_offset, measured_offset, alpha), now_us);
        }

        bool compass_available = model.imu.heading_lowpass_deg.valid;
        bool gps_available = model.navigation.gps.source.value != pypilot_data_model::SensorSource::none && model.navigation.gps.track_deg.valid;
        bool nav_available = model.navigation.apb.source.value != pypilot_data_model::SensorSource::none && model.navigation.apb.track_deg.valid;
        bool wind_available = model.wind.apparent.source.value != pypilot_data_model::SensorSource::none && model.wind.apparent.filtered_direction_deg.valid;
        bool true_wind_available = model.wind.truewind.source.value != pypilot_data_model::SensorSource::none && model.wind.truewind.filtered_direction_deg.valid;
        uint32_t logic_modes = pypilot_available_modes(compass_available,
                                                       gps_available,
                                                       nav_available,
                                                       wind_available,
                                                       true_wind_available,
                                                       model.ap.gps_and_nav_modes.value);
        model.ap.available_modes_mask = to_data_mode_mask(logic_modes);
        PypilotMode preferred_mode = to_logic_mode(model.ap.preferred_mode.value);
        PypilotMode active_mode = pypilot_best_mode(preferred_mode, logic_modes);
        pypilot_data_model::AutopilotMode old_mode = model.ap.mode.value;
        pypilot_data_model::AutopilotMode new_mode = to_data_mode(active_mode);
        Real gps_offset = model.ap.gps_compass_offset_deg.valid ? model.ap.gps_compass_offset_deg.value : Real(0);
        Real wind_offset = model.ap.wind_compass_offset_deg.valid ? model.ap.wind_compass_offset_deg.value : Real(0);
        Real true_wind_offset = model.ap.true_wind_compass_offset_deg.valid ? model.ap.true_wind_compass_offset_deg.value : Real(0);
        Real mode_heading = pypilot_mode_heading(active_mode, model.imu.heading_lowpass_deg.value, gps_offset, wind_offset, true_wind_offset);
        model.ap.heading_deg.set(mode_heading, now_us);
