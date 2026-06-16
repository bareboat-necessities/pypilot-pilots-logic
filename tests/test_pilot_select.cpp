#include <cassert>
#include <cmath>
#include <pypilot_pilots_logic.hpp>
using namespace pypilot_pilots_logic;
using pypilot_data_model::PilotName;
int main() { DataModel model; set_gain_defaults(model); model.ap.heading_error_deg.set(10.0f, 1); model.imu.heading_rate_lowpass_deg_s.set(1.0f, 1); model.ap.pilot.value = PilotName::simple; PilotResult r = compute_selected_pilot(model, 10); assert(r.valid); assert(std::fabs(r.command_norm - (10.0f*0.005f + 1.0f*0.15f)) < 0.0001f); return 0; }
