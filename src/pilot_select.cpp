#include <pypilot_pilots_logic/pilot_select.hpp>
#include <pypilot_pilots_logic/basic_pilot.hpp>
#include <pypilot_pilots_logic/absolute_pilot.hpp>
#include <pypilot_pilots_logic/simple_pilot.hpp>
#include <pypilot_pilots_logic/rate_pilot.hpp>
namespace pypilot_pilots_logic { PilotResult compute_selected_pilot(DataModel& model, uint64_t now_us) { switch(model.ap.pilot.value) { case pypilot_data_model::PilotName::absolute: { PilotResult r=compute_absolute_pilot(model,now_us); return r.valid ? r : compute_basic_pilot(model,now_us); } case pypilot_data_model::PilotName::simple: return compute_simple_pilot(model,now_us); case pypilot_data_model::PilotName::rate: return compute_rate_pilot(model,now_us); case pypilot_data_model::PilotName::basic: default: return compute_basic_pilot(model,now_us); } } }
