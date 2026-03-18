cmake_minimum_required(VERSION 3.16)
project(robotx_light_buoy)

find_package(gz-sim7    REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7   REQUIRED)
find_package(sdformat13 REQUIRED)

add_library(RobotXLightBuoySystem SHARED
  RobotXLightBuoySystem.cc
)

target_include_directories(RobotXLightBuoySystem PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(RobotXLightBuoySystem PUBLIC
  gz-sim7::gz-sim7
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
  sdformat13::sdformat13
)

set_target_properties(RobotXLightBuoySystem PROPERTIES
  OUTPUT_NAME gz-sim-robotx-light-buoy-system
)
