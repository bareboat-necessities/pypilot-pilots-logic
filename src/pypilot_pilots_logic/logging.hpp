#pragma once

#if defined(__has_include)
#  if __has_include(<pypilot_syslib.hpp>)
#    include <pypilot_syslib.hpp>
#    define PYPILOT_PILOTS_LOGIC_HAVE_SYSLIB 1
#  endif
#endif

#ifndef PYPILOT_PILOTS_LOGIC_HAVE_SYSLIB
#include <pypilot_algorithms/gps_filter.hpp>
#endif

namespace pypilot_pilots_logic {

inline pypilot_syslib::Logger*& pilots_logic_logger_ref() {
    static pypilot_syslib::Logger* logger = 0;
    return logger;
}

inline void set_pilots_logic_logger(pypilot_syslib::Logger* logger) {
    pilots_logic_logger_ref() = logger;
}

inline pypilot_syslib::Logger* pilots_logic_logger() {
    return pilots_logic_logger_ref();
}

} // namespace pypilot_pilots_logic
