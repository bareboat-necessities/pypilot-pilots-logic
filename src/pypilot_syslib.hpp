#pragma once

#if defined(__has_include_next)
#  if __has_include_next(<pypilot_syslib.hpp>)
#    include_next <pypilot_syslib.hpp>
#    define PYPILOT_LOCAL_SYSLIB_FORWARDER_USED_REAL 1
#  endif
#endif

#ifndef PYPILOT_LOCAL_SYSLIB_FORWARDER_USED_REAL
#include <stdint.h>
namespace pypilot_syslib {
class Logger {};
enum class LogLevel : uint8_t { Debug, Info, Warn, Error, Critical };
enum class LogModule : uint8_t { Algorithms, DataModel, Sensors, PilotsLogic, GpsAdapter, SteeringSignaling, ServoProtocol, Nmea0183, SignalK, Runtime };
enum class LogEvent : uint16_t {
    SystemStartup = 1,
    SystemShutdown = 2,
    ConfigurationLoaded = 3,
    ConfigurationInvalid = 4,
    SourceSelected = 5,
    SourceRejected = 6,
    SourceTimedOut = 7,
    SensorSampleRejected = 8,
    GpsFixAccepted = 9,
    GpsFixRejected = 10,
    GpsFilterReset = 11,
    GpsFilterPredictionReset = 12,
    WmmEvaluationInvalid = 13,
    PilotModeChanged = 14,
    PilotCommandComputed = 15,
    ApbNavCommandAccepted = 16,
    ApbNavCommandRateLimited = 17,
    RudderCalibrationInvalid = 18,
    ServoCommandBlocked = 19,
    ServoFeedbackFault = 20
};
inline void log_if(Logger*, uint64_t, LogLevel, LogModule, LogEvent, const char*, int32_t = 0, float = 0.0f) {}
}
#endif
