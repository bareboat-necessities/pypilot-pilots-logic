            Real factor = pypilot_algorithms::pypilot_wind_filter_factor(filter_constant, filtered_speed);
            model.wind.apparent.filter_factor_0_1.set(factor, now_us);
            Real previous_direction = model.wind.apparent.filtered_direction_deg.valid ? model.wind.apparent.filtered_direction_deg.value : Real(0);
            model.wind.apparent.filtered_direction_deg.set(
                pypilot_algorithms::pypilot_wind_filtered_direction(previous_direction,
                                                                    model.wind.apparent.filtered_direction_deg.valid,
                                                                    model.wind.apparent.direction_deg.value,
                                                                    factor),
                now_us);
        }
    }

    Real boat_speed_kn = Real(0);
    bool has_boat_speed = false;
    pypilot_data_model::SensorSource synthesized_truewind_source = pypilot_data_model::SensorSource::none;
    if (model.water.source.value != pypilot_data_model::SensorSource::none && model.water.speed_kn.valid) {
        boat_speed_kn = model.water.speed_kn.value;
        has_boat_speed = true;
        synthesized_truewind_source = pypilot_data_model::SensorSource::water_wind;
        if (source_can_update(model.water.leeway_source.value, pypilot_data_model::SensorSource::water_wind) && model.imu.heel_deg.valid) {
            Real leeway = pypilot_algorithms::pypilot_leeway_deg(model.imu.heel_deg.value, model.water.speed_kn.value);
            if (leeway != Real(0)) {
                model.water.leeway_deg.set(leeway, now_us);
                model.water.leeway_source.value = pypilot_data_model::SensorSource::water_wind;
            }
        }
    } else if (model.navigation.gps.source.value != pypilot_data_model::SensorSource::none && model.navigation.gps.speed_kn.valid) {
        gps_speed_filter_kn_ = pypilot_algorithms::pypilot_gps_speed_filter(gps_speed_filter_kn_, model.navigation.gps.speed_kn.value);
        boat_speed_kn = gps_speed_filter_kn_;
        has_boat_speed = true;
        synthesized_truewind_source = pypilot_data_model::SensorSource::gps_wind;
    }

    bool can_synthesize_truewind = source_can_update(model.wind.truewind.source.value,
                                                     synthesized_truewind_source,
                                                     now_us,
                                                     model.wind.truewind.last_update_us);
    if (can_synthesize_truewind && has_boat_speed &&
        model.wind.apparent.source.value != pypilot_data_model::SensorSource::none &&
        model.wind.apparent.speed_kn.valid && model.wind.apparent.direction_deg.valid) {
        model.wind.truewind.source.value = synthesized_truewind_source;
        model.wind.truewind.speed_kn.set(pypilot_algorithms::pypilot_true_wind_speed(boat_speed_kn,
                                                                                     model.wind.apparent.speed_kn.value,
                                                                                     model.wind.apparent.direction_deg.value),
                                         now_us);
        model.wind.truewind.direction_deg.set(pypilot_algorithms::pypilot_true_wind_direction(boat_speed_kn,
                                                                                              model.wind.apparent.speed_kn.value,
                                                                                              model.wind.apparent.direction_deg.value),
                                              now_us);
        model.wind.truewind.last_update_us = now_us;
    }

    if (model.wind.truewind.source.value != pypilot_data_model::SensorSource::none && model.wind.truewind.speed_kn.valid) {
        Real previous_speed = model.wind.truewind.filtered_speed_kn.valid ? model.wind.truewind.filtered_speed_kn.value : Real(0);
        Real filtered_speed = pypilot_algorithms::pypilot_wind_filtered_speed(previous_speed, model.wind.truewind.speed_kn.value);
        model.wind.truewind.filtered_speed_kn.set(filtered_speed, now_us);
        if (model.wind.truewind.direction_deg.valid) {
            Real filter_constant = model.wind.truewind.filter_constant_0_1.value == Real(0) ? Real(0.1) : model.wind.truewind.filter_constant_0_1.value;
            Real factor = pypilot_algorithms::pypilot_wind_filter_factor(filter_constant, filtered_speed);
            model.wind.truewind.filter_factor_0_1.set(factor, now_us);
            Real previous_direction = model.wind.truewind.filtered_direction_deg.valid ? model.wind.truewind.filtered_direction_deg.value : Real(0);
            model.wind.truewind.filtered_direction_deg.set(
                pypilot_algorithms::pypilot_wind_filtered_direction(previous_direction,
                                                                    model.wind.truewind.filtered_direction_deg.valid,
                                                                    model.wind.truewind.direction_deg.value,
                                                                    factor),
                now_us);
        }
    }

