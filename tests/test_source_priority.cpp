#include <cassert>

#include <pypilot_pilots_logic/source_priority.hpp>

using namespace pypilot_pilots_logic;
using namespace pypilot_data_model;

int main() {
    assert(source_can_update(SensorSource::signalk, SensorSource::serial));
    assert(!source_can_update(SensorSource::serial, SensorSource::signalk));
    assert(source_can_update(SensorSource::tcp, SensorSource::gpsd));
    assert(!source_can_update(SensorSource::gpsd, SensorSource::tcp));
    return 0;
}
