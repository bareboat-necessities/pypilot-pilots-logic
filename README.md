# pypilot-pilots-logic

Compiled C++11 pypilot pilot logic library for Linux and Arduino.

This is not a header-only module. It builds a real library target named `pypilot_pilots_logic`.

## Public include

```cpp
#include <pypilot_pilots_logic.hpp>
```

## Scope

Implemented in this project:

- source arbitration and stale handling
- pypilot gain defaults
- basic pilot
- absolute pilot
- simple pilot
- rate pilot
- wind pilot
- gps pilot
- vmg pilot command core
- selected pilot dispatch
- `PilotsLogic` facade writing `servo.command_norm` or `servo.position_command_deg`

It deliberately does not parse NMEA, SignalK, TCP, JSON, or servo UART packets.

## Dependencies

- `pypilot-data-model`
- `pypilot-algorithms`
- `pypilot-servo-protocol`

## Build

The CMake build expects the dependency repositories checked out as siblings unless their paths are overridden:

```bash
cmake -S pypilot-pilots-logic -B build \
  -DPYPILOT_DATA_MODEL_DIR=$PWD/pypilot-data-model/src \
  -DPYPILOT_ALGORITHMS_DIR=$PWD/pypilot-algorithms/src \
  -DPYPILOT_SERVO_PROTOCOL_DIR=$PWD/pypilot-servo-protocol/src
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Arduino example build

`pypilot-servo-protocol` must be passed as an Arduino library because `servo_rudder.hpp` includes the real servo protocol header:

```bash
arduino-cli compile \
  --fqbn arduino:avr:mega \
  --libraries pypilot-pilots-logic \
  --libraries pypilot-data-model \
  --libraries pypilot-algorithms \
  --libraries pypilot-servo-protocol \
  pypilot-pilots-logic/examples/arduino/PilotsLogicExample
```
