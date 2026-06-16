#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>
#include <pypilot_pilots_logic.hpp>

using namespace pypilot_pilots_logic;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    DataModel model;
    set_gain_defaults(model);
    model.ap.heading_error_deg.set(10.0f, 1);
    model.ap.heading_error_int_deg.set(2.0f, 1);
    model.ap.heading_command_rate_deg_s.set(0.25f, 1);
    model.imu.heading_rate_lowpass_deg_s.set(1.0f, 1);
    model.imu.heading_rate_rate_lowpass_deg_s2.set(0.5f, 1);
    model.rudder.angle_deg.set(0.0f, 1);
    model.rudder.range_deg.value = 30.0f;
    model.wind.apparent.source.value = pypilot_data_model::SensorSource::serial;

    pypilot_algorithms::BasicPilotInput<float> basic_in;
    basic_in.heading_error_deg = model.ap.heading_error_deg.value;
    basic_in.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    basic_in.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    basic_in.heading_command_rate_deg_s = model.ap.heading_command_rate_deg_s.value;
    pypilot_algorithms::BasicPilotGains<float> basic_gains;
    basic_gains.P = model.pilots.basic.P.gain.value;
    basic_gains.D = model.pilots.basic.D.gain.value;
    basic_gains.DD = model.pilots.basic.DD.gain.value;
    basic_gains.PR = model.pilots.basic.PR.gain.value;
    basic_gains.FF = model.pilots.basic.FF.gain.value;
    auto basic_expected = pypilot_algorithms::compute_basic_pilot(basic_in, basic_gains, pypilot_algorithms::CommandClamp::raw);
    auto basic_actual = compute_basic_pilot(model, 10);
    assert(nearf(basic_actual.command_norm, basic_expected.command_norm));
    assert(nearf(model.pilots.basic.PR.contribution.value, basic_expected.PRgain));

    pypilot_algorithms::AbsolutePilotInput<float> absolute_in;
    absolute_in.heading_error_deg = model.ap.heading_error_deg.value;
    absolute_in.heading_error_int_deg = model.ap.heading_error_int_deg.value;
    absolute_in.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    absolute_in.heading_command_rate_deg_s = model.ap.heading_command_rate_deg_s.value;
    pypilot_algorithms::AbsolutePilotGains<float> absolute_gains;
    absolute_gains.P = model.pilots.absolute.P.gain.value;
    absolute_gains.I = model.pilots.absolute.I.gain.value;
    absolute_gains.D = model.pilots.absolute.D.gain.value;
    absolute_gains.FF = model.pilots.absolute.FF.gain.value;
    auto absolute_expected = pypilot_algorithms::compute_absolute_pilot(absolute_in, absolute_gains, pypilot_algorithms::CommandClamp::raw);
    auto absolute_actual = compute_absolute_pilot(model, 20);
    assert(nearf(absolute_actual.position_command_deg, absolute_expected.command_norm * model.rudder.range_deg.value));

    pypilot_algorithms::SimplePilotInput<float> simple_in;
    simple_in.heading_error_deg = model.ap.heading_error_deg.value;
    simple_in.heading_error_int_deg = model.ap.heading_error_int_deg.value;
    simple_in.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    pypilot_algorithms::SimplePilotGains<float> simple_gains;
    simple_gains.P = model.pilots.simple.P.gain.value;
    simple_gains.I = model.pilots.simple.I.gain.value;
    simple_gains.D = model.pilots.simple.D.gain.value;
    auto simple_expected = pypilot_algorithms::compute_simple_pilot(simple_in, simple_gains, pypilot_algorithms::CommandClamp::raw);
    auto simple_actual = compute_simple_pilot(model, 30);
    assert(nearf(simple_actual.command_norm, simple_expected.command_norm));

    pypilot_algorithms::RatePilotInput<float> rate_in;
    rate_in.heading_error_deg = model.ap.heading_error_deg.value;
    rate_in.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    rate_in.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    rate_in.heading_command_rate_deg_s = model.ap.heading_command_rate_deg_s.value;
    pypilot_algorithms::RatePilotGains<float> rate_gains;
    rate_gains.D = model.pilots.rate.D.gain.value;
    rate_gains.DD = model.pilots.rate.DD.gain.value;
    rate_gains.FF = model.pilots.rate.FF.gain.value;
    rate_gains.max_turn_rate_deg_s = model.pilots.rate.max_turn_rate_deg_s.value;
    rate_gains.turn_rate_rate_deg_s2 = model.pilots.rate.turn_rate_rate_deg_s2.value;
    auto rate_expected = pypilot_algorithms::compute_rate_pilot(rate_in, rate_gains, pypilot_algorithms::CommandClamp::raw);
    auto rate_actual = compute_rate_pilot(model, 40);
    assert(nearf(rate_actual.command_norm, rate_expected.command_norm));

    pypilot_algorithms::WindPilotInput<float> wind_in;
    wind_in.wind_error_deg = model.ap.heading_error_deg.value;
    wind_in.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    wind_in.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    wind_in.wind_gust_kn = 0.0f;
    pypilot_algorithms::WindPilotGains<float> wind_gains;
    wind_gains.P = model.pilots.wind.P.gain.value;
    wind_gains.D = model.pilots.wind.D.gain.value;
    wind_gains.DD = model.pilots.wind.DD.gain.value;
    wind_gains.WG = model.pilots.wind.WG.gain.value;
    auto wind_expected = pypilot_algorithms::compute_wind_pilot(wind_in, wind_gains, pypilot_algorithms::CommandClamp::raw);
    auto wind_actual = compute_wind_pilot(model, 50);
    assert(nearf(wind_actual.command_norm, wind_expected.command_norm));

    pypilot_algorithms::GpsPilotGains<float> gps_gains;
    gps_gains.P = model.pilots.gps.P.gain.value;
    gps_gains.D = model.pilots.gps.D.gain.value;
    gps_gains.DD = model.pilots.gps.DD.gain.value;
    gps_gains.PR = 0.0f;
    gps_gains.FF = model.pilots.gps.FF.gain.value;
    auto gps_expected = pypilot_algorithms::compute_gps_pilot(basic_in, gps_gains, pypilot_algorithms::CommandClamp::raw);
    auto gps_actual = compute_gps_pilot(model, 60);
    assert(nearf(gps_actual.command_norm, gps_expected.command_norm));

    pypilot_algorithms::VmgPilotInput<float> vmg_in;
    vmg_in.heading_error_deg = model.ap.heading_error_deg.value;
    vmg_in.headingrate_deg_s = model.imu.heading_rate_lowpass_deg_s.value;
    vmg_in.headingraterate_deg_s2 = model.imu.heading_rate_rate_lowpass_deg_s2.value;
    pypilot_algorithms::VmgPilotGains<float> vmg_gains;
    vmg_gains.P = model.pilots.vmg.P.gain.value;
    vmg_gains.D = model.pilots.vmg.D.gain.value;
    vmg_gains.DD = model.pilots.vmg.DD.gain.value;
    auto vmg_expected = pypilot_algorithms::compute_vmg_pilot(vmg_in, vmg_gains, pypilot_algorithms::CommandClamp::raw);
    auto vmg_actual = compute_vmg_pilot(model, 70);
    assert(nearf(vmg_actual.command_norm, vmg_expected.command_norm));

    return 0;
}
