        model.wind.truewind.speed_kn.valid = false;
        model.wind.truewind.direction_deg.valid = false;
        model.wind.truewind.filtered_speed_kn.valid = false;
        model.wind.truewind.filtered_direction_deg.valid = false;
        model.wind.truewind.filter_factor_0_1.valid = false;
    }
    if (model.water.source.value != pypilot_data_model::SensorSource::none &&
        source_is_stale(now_us, model.water.last_update_us)) {
        model.water.source.value = pypilot_data_model::SensorSource::none;
        model.water.speed_kn.valid = false;
        model.water.leeway_deg.valid = false;
        model.water.current_speed_kn.valid = false;
        model.water.current_direction_deg.valid = false;
        model.water.wind_speed_kn.valid = false;
        model.water.wind_direction_deg.valid = false;
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
      tack_state_(PypilotTackState::none),
      tack_direction_(PypilotTackDirection::none),
      tack_current_direction_(PypilotTackDirection::none),
      tack_delay_s_(0.0f),
      tack_angle_deg_(100.0f),
      tack_rate_deg_s_(15.0f),
      tack_threshold_percent_(50.0f),
      tack_state_start_us_(0) {}

void PilotsLogic::begin_tack(PypilotTackDirection direction) {
    tack_direction_ = direction;
    tack_state_ = PypilotTackState::begin;
}

void PilotsLogic::cancel_tack() {
    tack_state_ = PypilotTackState::none;
    tack_direction_ = PypilotTackDirection::none;
    tack_current_direction_ = PypilotTackDirection::none;
    tack_state_start_us_ = 0;
}

PypilotTackState PilotsLogic::tack_state() const { return tack_state_; }
PypilotTackDirection PilotsLogic::tack_direction() const { return tack_direction_; }

void PilotsLogic::set_tack_config(Real delay_s, Real angle_deg, Real rate_deg_s, Real threshold_percent) {
    tack_delay_s_ = delay_s;
    tack_angle_deg_ = angle_deg;
    tack_rate_deg_s_ = rate_deg_s;
    tack_threshold_percent_ = threshold_percent;
}

bool PilotsLogic::update_inputs(DataModel& model, uint64_t now_us) {
    last_error_ = "";
    invalidate_stale_sources(model, now_us);

    if (model.wind.apparent.source.value != pypilot_data_model::SensorSource::none && model.wind.apparent.speed_kn.valid) {
        Real previous_speed = model.wind.apparent.filtered_speed_kn.valid ? model.wind.apparent.filtered_speed_kn.value : Real(0);
        Real filtered_speed = pypilot_algorithms::pypilot_wind_filtered_speed(previous_speed, model.wind.apparent.speed_kn.value);
        model.wind.apparent.filtered_speed_kn.set(filtered_speed, now_us);
        if (model.wind.apparent.direction_deg.valid) {
            Real filter_constant = model.wind.apparent.filter_constant_0_1.value == Real(0) ? Real(0.1) : model.wind.apparent.filter_constant_0_1.value;
