#include <cassert>
#include <pypilot_pilots_logic/stale.hpp>
int main(){ pypilot_data_model::Stamped<float> v; assert(!pypilot_pilots_logic::value_is_fresh(v,1000,100)); v.set(1.0f,1000); assert(pypilot_pilots_logic::value_is_fresh(v,1050,100)); assert(!pypilot_pilots_logic::value_is_fresh(v,1201,100)); return 0; }
