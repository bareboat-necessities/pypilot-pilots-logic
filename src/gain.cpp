#include <pypilot_pilots_logic/gain.hpp>

namespace pypilot_pilots_logic {

Real compute_gain(Real input, Real gain) {
    return input * gain;
}

static void set_range(pypilot_data_model::RangeSetting<Real>& setting,
                      Real value,
                      Real min_value,
                      Real max_value) {
    setting.value = value;
    setting.min = min_value;
    setting.max = max_value;
}

void set_gain_defaults(DataModel& model) {
    set_range(model.pilots.basic.P.gain, 0.003f, 0.0f, 0.03f);
    set_range(model.pilots.basic.D.gain, 0.09f, 0.0f, 0.24f);
    set_range(model.pilots.basic.DD.gain, 0.075f, 0.0f, 0.24f);
    set_range(model.pilots.basic.PR.gain, 0.005f, 0.0f, 0.02f);
    set_range(model.pilots.basic.FF.gain, 0.6f, 0.0f, 2.4f);

    set_range(model.pilots.absolute.P.gain, 0.05f, 0.0f, 0.3f);
    set_range(model.pilots.absolute.I.gain, 0.0f, 0.0f, 0.8f);
    set_range(model.pilots.absolute.D.gain, 0.05f, 0.0f, 0.5f);
    set_range(model.pilots.absolute.FF.gain, 0.0f, 0.0f, 1.0f);

    set_range(model.pilots.simple.P.gain, 0.005f, 0.0f, 0.025f);
    set_range(model.pilots.simple.I.gain, 0.0f, 0.0f, 0.05f);
    set_range(model.pilots.simple.D.gain, 0.15f, 0.0f, 0.5f);

    set_range(model.pilots.rate.D.gain, 0.075f, 0.0f, 0.3f);
    set_range(model.pilots.rate.DD.gain, 0.075f, 0.0f, 0.3f);
    set_range(model.pilots.rate.FF.gain, 0.6f, 0.0f, 3.0f);
    set_range(model.pilots.rate.max_turn_rate_deg_s, 2.0f, 0.5f, 30.0f);
    set_range(model.pilots.rate.turn_rate_rate_deg_s2, 0.5f, 0.1f, 5.0f);

    set_range(model.pilots.wind.P.gain, 0.003f, 0.0f, 0.02f);
    set_range(model.pilots.wind.D.gain, 0.1f, 0.0f, 1.0f);
    set_range(model.pilots.wind.DD.gain, 0.05f, 0.0f, 1.0f);
    set_range(model.pilots.wind.WG.gain, 0.0f, -0.1f, 0.1f);

    set_range(model.pilots.gps.P.gain, 0.003f, 0.0f, 0.02f);
    set_range(model.pilots.gps.D.gain, 0.1f, 0.0f, 1.0f);
    set_range(model.pilots.gps.DD.gain, 0.05f, 0.0f, 1.0f);
    set_range(model.pilots.gps.FF.gain, 0.6f, 0.0f, 3.0f);

    set_range(model.pilots.vmg.P.gain, 0.003f, 0.0f, 0.02f);
    set_range(model.pilots.vmg.D.gain, 0.09f, 0.0f, 0.8f);
    set_range(model.pilots.vmg.DD.gain, 0.075f, 0.0f, 0.8f);
}

} // namespace pypilot_pilots_logic
