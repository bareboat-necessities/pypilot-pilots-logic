#pragma once
#include "types.hpp"
#include "source_priority.hpp"
#include "stale.hpp"
#include "gain.hpp"
#include "basic_pilot.hpp"
#include "absolute_pilot.hpp"
#include "simple_pilot.hpp"
#include "rate_pilot.hpp"
#include "pilot_select.hpp"
namespace pypilot_pilots_logic {
class PilotsLogic { public: PilotsLogic(); bool update_inputs(DataModel& model, uint64_t now_us); bool compute_command(DataModel& model, uint64_t now_us); bool step(DataModel& model, uint64_t now_us); const char* last_error() const; private: const char* last_error_; };
}
