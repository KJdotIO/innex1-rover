cmake_minimum_required(VERSION 3.16)
project(teensy_motor_io LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(teensy_motor_io_core
  src/protocol.cpp
  src/safety_state.cpp
)
target_include_directories(teensy_motor_io_core PUBLIC include)

enable_testing()

add_executable(test_protocol tests/test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE teensy_motor_io_core)
add_test(NAME protocol COMMAND test_protocol)

add_executable(test_safety_state tests/test_safety_state.cpp)
target_link_libraries(test_safety_state PRIVATE teensy_motor_io_core)
add_test(NAME safety_state COMMAND test_safety_state)
