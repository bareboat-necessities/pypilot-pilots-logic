cmake_minimum_required(VERSION 3.16)
project(pypilot_pilots_logic LANGUAGES CXX)

option(PYPILOT_PILOTS_LOGIC_BUILD_TESTS "Build tests" ON)
option(PYPILOT_PILOTS_LOGIC_BUILD_EXAMPLES "Build examples" ON)

set(PYPILOT_DATA_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-data-model/src" CACHE PATH "Path to pypilot-data-model src")
set(PYPILOT_ALGORITHMS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-algorithms/src" CACHE PATH "Path to pypilot-algorithms src")
set(PYPILOT_SERVO_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-servo-protocol/src" CACHE PATH "Path to pypilot-servo-protocol src")
set(PYPILOT_SYSLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pypilot-syslib/src" CACHE PATH "Path to pypilot-syslib src")

set(PYPILOT_REPO_PREFIX "https://github.com")
set(PYPILOT_DATA_MODEL_REPO "${PYPILOT_REPO_PREFIX}/bareboat-necessities/pypilot-data-model.git")
set(PYPILOT_SERVO_PROTOCOL_REPO "${PYPILOT_REPO_PREFIX}/bareboat-necessities/pypilot-servo-protocol.git")
set(PYPILOT_SYSLIB_REPO "${PYPILOT_REPO_PREFIX}/bareboat-necessities/pypilot-syslib.git")

# If sibling checkouts are not present, FetchContent pulls the current shared libraries.
if(NOT EXISTS "${PYPILOT_DATA_MODEL_DIR}/ship_data_model.hpp")
  include(FetchContent)
  FetchContent_Declare(
    pypilot_data_model
    GIT_REPOSITORY ${PYPILOT_DATA_MODEL_REPO}
    GIT_TAG main
  )
  FetchContent_GetProperties(pypilot_data_model)
  if(NOT pypilot_data_model_POPULATED)
    FetchContent_Populate(pypilot_data_model)
  endif()
  set(PYPILOT_DATA_MODEL_DIR "${pypilot_data_model_SOURCE_DIR}/src")
endif()

if(NOT EXISTS "${PYPILOT_SERVO_PROTOCOL_DIR}/pypilot_servo_protocol.hpp")
  include(FetchContent)
  FetchContent_Declare(
    pypilot_servo_protocol
    GIT_REPOSITORY ${PYPILOT_SERVO_PROTOCOL_REPO}
    GIT_TAG main
  )
  FetchContent_GetProperties(pypilot_servo_protocol)
  if(NOT pypilot_servo_protocol_POPULATED)
    FetchContent_Populate(pypilot_servo_protocol)
  endif()
  set(PYPILOT_SERVO_PROTOCOL_DIR "${pypilot_servo_protocol_SOURCE_DIR}/src")
endif()

if(NOT EXISTS "${PYPILOT_SYSLIB_DIR}/pypilot_syslib.hpp")
  include(FetchContent)
  FetchContent_Declare(
    pypilot_syslib
    GIT_REPOSITORY ${PYPILOT_SYSLIB_REPO}
    GIT_TAG main
  )
  FetchContent_GetProperties(pypilot_syslib)
  if(NOT pypilot_syslib_POPULATED)
    FetchContent_Populate(pypilot_syslib)
  endif()
  set(PYPILOT_SYSLIB_DIR "${pypilot_syslib_SOURCE_DIR}/src")
endif()

add_library(pypilot_pilots_logic
  src/pypilot_pilots_logic/gain.cpp
  src/pypilot_pilots_logic/basic_pilot.cpp
  src/pypilot_pilots_logic/absolute_pilot.cpp
  src/pypilot_pilots_logic/simple_pilot.cpp
  src/pypilot_pilots_logic/rate_pilot.cpp
  src/pypilot_pilots_logic/wind_pilot.cpp
  src/pypilot_pilots_logic/gps_pilot.cpp
  src/pypilot_pilots_logic/vmg_pilot.cpp
  src/pypilot_pilots_logic/pilot_select.cpp
  src/pypilot_pilots_logic/pilots_logic.cpp
)

target_include_directories(pypilot_pilots_logic PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PYPILOT_DATA_MODEL_DIR}
  ${PYPILOT_ALGORITHMS_DIR}
  ${PYPILOT_SERVO_PROTOCOL_DIR}
  ${PYPILOT_SYSLIB_DIR}
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
      test_tack
      test_source_arbitration
      test_servo_feedback
      test_apb_nav
      test_pilot_select
      test_pilots_logic)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE pypilot_pilots_logic)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
