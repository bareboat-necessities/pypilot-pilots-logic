cmake_minimum_required(VERSION 3.16)
project(pypilot_pilots_logic LANGUAGES CXX)

option(PYPILOT_PILOTS_LOGIC_BUILD_TESTS "Build tests" ON)
option(PYPILOT_PILOTS_LOGIC_BUILD_EXAMPLES "Build examples" ON)

set(PYPILOT_DATA_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-data-model/src" CACHE PATH "Path to pypilot-data-model src")
set(PYPILOT_ALGORITHMS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-algorithms/src" CACHE PATH "Path to pypilot-algorithms src")

add_library(pypilot_pilots_logic
  src/math_helpers.cpp
  src/source_priority.cpp
  src/gain.cpp
  src/basic_pilot.cpp
  src/absolute_pilot.cpp
  src/simple_pilot.cpp
  src/rate_pilot.cpp
  src/wind_pilot.cpp
  src/gps_pilot.cpp
  src/vmg_pilot.cpp
  src/pilot_select.cpp
  src/pilots_logic.cpp
)

target_include_directories(pypilot_pilots_logic PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PYPILOT_DATA_MODEL_DIR}
  ${PYPILOT_ALGORITHMS_DIR}
)

target_compile_features(pypilot_pilots_logic PUBLIC cxx_std_11)

if(PYPILOT_PILOTS_LOGIC_BUILD_EXAMPLES)
  add_executable(compute_basic examples/linux/compute_basic.cpp)
  target_link_libraries(compute_basic PRIVATE pypilot_pilots_logic)
endif()

if(PYPILOT_PILOTS_LOGIC_BUILD_TESTS)
  enable_testing()
  foreach(test_name
      test_source_priority
      test_stale
      test_gain
      test_basic_pilot
      test_absolute_pilot
      test_simple_pilot
      test_rate_pilot
      test_wind_pilot
      test_gps_pilot
      test_vmg_pilot
      test_shared_algorithm_usage
      test_sensor_filters
      test_mode_state
      test_pilot_select
      test_pilots_logic)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE pypilot_pilots_logic)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
