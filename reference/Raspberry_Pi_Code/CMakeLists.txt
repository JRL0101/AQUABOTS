cmake_minimum_required(VERSION 3.10)
project(aquabots)

set(CMAKE_CXX_STANDARD 17)

add_library(sensor_fusion src/sensor_fusion.cpp)
target_include_directories(sensor_fusion PUBLIC include)

add_library(control src/control.cpp Guidance.cpp PIcontrol.cpp)
target_include_directories(control PUBLIC include)

add_library(hardware src/hardware.cpp)
target_include_directories(hardware PUBLIC include)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(runTests tests/control_test.cpp)
target_link_libraries(runTests gtest_main sensor_fusion control hardware)
include(GoogleTest)
gtest_discover_tests(runTests)
