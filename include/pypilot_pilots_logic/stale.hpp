#pragma once
#include <stdint.h>
#include <pypilot_data_model.hpp>
namespace pypilot_pilots_logic { template<typename T> inline bool value_is_fresh(const pypilot_data_model::Stamped<T>& value, uint64_t now_us, uint64_t max_age_us) { return value.valid && !value.stale(now_us, max_age_us); } }
