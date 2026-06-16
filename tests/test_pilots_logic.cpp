#include <cassert>
#include <cmath>
#include <pypilot_pilots_logic.hpp>
using namespace pypilot_pilots_logic;
int main(){ pypilot_pilots_logic::DataModel model; model.ap.enabled.value=true; model.ap.pilot.value=pypilot_data_model::PilotName::basic; model.ap.heading_command_deg.set(100.0f,100); model.ap.heading_command_rate_deg_s.set(0.0f,100); model.imu.heading_lowpass_deg.set(90.0f,100); model.imu.heading_rate_lowpass_deg_s.set(0.0f,100); model.imu.heading_rate_rate_lowpass_deg_s2.set(0.0f,100); PilotsLogic logic; assert(logic.step(model,200)); assert(model.ap.heading_error_deg.valid); assert(std::fabs(model.ap.heading_error_deg.value-10.0f)<0.0001f); assert(model.servo.command_norm.valid); return 0; }
