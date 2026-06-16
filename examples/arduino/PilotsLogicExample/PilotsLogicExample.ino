#include <Arduino.h>
#include <pypilot_pilots_logic.hpp>

pypilot_pilots_logic::DataModel model;
pypilot_pilots_logic::PilotsLogic logic;

void setup() {
  Serial.begin(115200);

  model.ap.enabled.value = true;
  model.ap.pilot.value = pypilot_data_model::PilotName::basic;
  model.ap.heading_command_deg.set(100.0f, 1000000);
  model.ap.heading_command_rate_deg_s.set(0.0f, 1000000);
  model.imu.heading_lowpass_deg.set(90.0f, 1000000);
  model.imu.heading_rate_lowpass_deg_s.set(0.0f, 1000000);
  model.imu.heading_rate_rate_lowpass_deg_s2.set(0.0f, 1000000);

  logic.step(model, 2000000);
  Serial.println(model.servo.command_norm.value);
}

void loop() {}
