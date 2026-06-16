# pypilot-pilots-logic

Compiled C++11 pypilot pilot logic library for Linux and Arduino.

This is not a header-only module. It builds a real library target named `pypilot_pilots_logic`.

## Public include

```cpp
#include <pypilot_pilots_logic.hpp>
```

## Scope

Implemented in this project:

- source-priority helper
- stale helper
- pypilot gain defaults
- basic pilot
- absolute pilot
- simple pilot
- rate pilot
- selected pilot dispatch
- `PilotsLogic` facade writing `servo.command_norm` or `servo.position_command_deg`

It deliberately does not parse NMEA, SignalK, TCP, JSON, or servo UART packets.

## Dependencies

- `pypilot-data-model`
- `pypilot-algorithms` include directory reserved for future extensions

## Build

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```
